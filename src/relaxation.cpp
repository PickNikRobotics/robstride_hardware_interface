// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "robstride_hardware_interface/relaxation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace robstride_hardware_interface
{
namespace
{
void Keys(
  const YAML::Node & node, const std::set<std::string> & required,
  const std::set<std::string> & optional = {})
{
  if (!node.IsMap()) {throw std::invalid_argument("expected a YAML mapping");}
  std::set<std::string> seen;
  for (const auto & item : node) {
    const auto key = item.first.as<std::string>();
    if (!seen.insert(key).second || (required.count(key) == 0 && optional.count(key) == 0)) {
      throw std::invalid_argument("duplicate or unknown YAML key: " + key);
    }
  }
  for (const auto & key : required) {
    if (seen.count(key) == 0) {throw std::invalid_argument("missing YAML key: " + key);}
  }
}

double Finite(const YAML::Node & node, const std::string & key, bool positive = false)
{
  const double value = node[key].as<double>();
  if (!std::isfinite(value) || (positive && value <= 0.0)) {
    throw std::invalid_argument(key + " must be finite" + (positive ? " and positive" : ""));
  }
  return value;
}

bool Different(double value, double baseline)
{
  // A nonfinite command is not permitted to release a seeded hold.
  return std::isfinite(value) &&
         (!std::isfinite(baseline) || std::abs(value - baseline) > kStillCommandEpsilon);
}
}  // namespace

std::vector<RelaxSafePose> LoadRelaxSafePoses(
  const std::string & file, const std::vector<std::string> & joint_names)
{
  if (file.empty() || file.front() != '/') {
    throw std::invalid_argument("relax_safe_poses_file must be an absolute path");
  }
  const auto documents = YAML::LoadAllFromFile(file);
  if (documents.size() != 1) {throw std::invalid_argument("expected exactly one YAML document");}
  const auto root = documents.front();
  Keys(root, {"version", "relax_safe_poses"});
  if (root["version"].as<int>() != 1) {
    throw std::invalid_argument("unsupported safe-pose version");
  }
  const auto entries = root["relax_safe_poses"];
  if (!entries.IsSequence() || entries.size() == 0 || entries.size() > 32) {
    throw std::invalid_argument("relax_safe_poses must contain 1..32 poses");
  }
  const std::set<std::string> names(joint_names.begin(), joint_names.end());
  if (names.empty() || names.size() != joint_names.size()) {
    throw std::invalid_argument("configured joint names must be unique and nonempty");
  }
  std::set<std::string> pose_names;
  std::vector<RelaxSafePose> poses;
  for (const auto & entry : entries) {
    Keys(entry, {"name", "joints", "stillness"});
    RelaxSafePose pose;
    pose.name = entry["name"].as<std::string>();
    if (pose.name.empty() || !pose_names.insert(pose.name).second) {
      throw std::invalid_argument("pose names must be unique and nonempty");
    }
    Keys(entry["joints"], names);
    for (const auto & joint : joint_names) {
      const auto values = entry["joints"][joint];
      Keys(values, {"position_rad", "tolerance_rad"});
      pose.position.push_back(Finite(values, "position_rad"));
      pose.tolerance_rad.push_back(Finite(values, "tolerance_rad", true));
    }
    const auto stillness = entry["stillness"];
    Keys(stillness, {"window_s", "max_span_rad"}, {"max_abs_velocity_rad_s"});
    pose.window_s = Finite(stillness, "window_s", true);
    if (pose.window_s > kMaxStillnessWindowS) {
      throw std::invalid_argument("stillness.window_s must be <= 10 seconds");
    }
    pose.max_span_rad = Finite(stillness, "max_span_rad", true);
    if (stillness["max_abs_velocity_rad_s"]) {
      pose.max_abs_velocity_rad_s = Finite(stillness, "max_abs_velocity_rad_s", true);
    }
    poses.push_back(std::move(pose));
  }
  return poses;
}

StillnessHistory::StillnessHistory(size_t joints, size_t capacity)
: joints_(joints), capacity_(capacity), stamp_(capacity), healthy_(capacity),
  position_(capacity * joints), velocity_(capacity * joints),
  command_position_(capacity * joints), command_velocity_(capacity * joints)
{
  if (joints == 0 || capacity < 2) {throw std::invalid_argument("invalid stillness history size");}
}

void StillnessHistory::Reset()
{
  count_ = 0;
  next_ = 0;
}

void StillnessHistory::Observe(
  uint64_t now_ns, const std::vector<double> & position,
  const std::vector<double> & velocity, const std::vector<double> & command_position,
  const std::vector<double> & command_velocity, bool healthy)
{
  if (count_ != 0) {
    const uint64_t previous = stamp_[(next_ + capacity_ - 1) % capacity_];
    if (now_ns <= previous || now_ns - previous > kMaxStillnessSampleGapNs) {Reset();}
  }
  stamp_[next_] = now_ns;
  healthy_[next_] = healthy;
  for (size_t joint = 0; joint < joints_; ++joint) {
    const size_t index = next_ * joints_ + joint;
    position_[index] = position[joint];
    velocity_[index] = velocity[joint];
    command_position_[index] = command_position[joint];
    command_velocity_[index] = command_velocity[joint];
  }
  next_ = (next_ + 1) % capacity_;
  count_ = std::min(count_ + 1, capacity_);
}

RelaxCheck StillnessHistory::CheckWindow(const RelaxSafePose & pose) const
{
  if (count_ == 0) {return RelaxCheck::HistoryMissing;}
  const size_t newest = (next_ + capacity_ - 1) % capacity_;
  const uint64_t window_ns = static_cast<uint64_t>(pose.window_s * 1.0e9);
  const uint64_t now_ns = stamp_[newest];
  if (now_ns < window_ns) {return RelaxCheck::HistoryMissing;}
  const uint64_t cutoff = now_ns - window_ns;
  size_t first = newest;
  size_t samples = 1;
  while (stamp_[first] > cutoff && samples < count_) {
    first = (first + capacity_ - 1) % capacity_;
    ++samples;
  }
  if (stamp_[first] > cutoff) {return RelaxCheck::HistoryMissing;}
  bool moving = false;
  bool commanded = false;
  for (size_t joint = 0; joint < joints_; ++joint) {
    double lower = std::numeric_limits<double>::infinity();
    double upper = -lower;
    double command_lower = lower;
    double command_upper = upper;
    // Least-squares slope of measured position over the window. Raw motor
    // velocity feedback is too noisy at rest to gate on (bench: |v| up to
    // 0.15 rad/s, median 0.026 rad/s, with position static within 0.0004 rad).
    double sum_t = 0.0, sum_p = 0.0, sum_tt = 0.0, sum_tp = 0.0;
    const size_t origin_index = first * joints_ + joint;
    for (size_t sample = 0; sample < samples; ++sample) {
      const size_t slot = (first + sample) % capacity_;
      if (!healthy_[slot]) {return RelaxCheck::Unhealthy;}
      const size_t index = slot * joints_ + joint;
      const double p = position_[index];
      const double v = velocity_[index];
      if (!std::isfinite(p) || !std::isfinite(v)) {return RelaxCheck::Unhealthy;}
      lower = std::min(lower, p);
      upper = std::max(upper, p);
      const double t = static_cast<double>(stamp_[slot] - stamp_[first]) * 1.0e-9;
      const double dp = p - position_[origin_index];
      sum_t += t;
      sum_p += dp;
      sum_tt += t * t;
      sum_tp += t * dp;
      const double command = command_position_[index];
      const double command_v = command_velocity_[index];
      if (!std::isfinite(command) || !std::isfinite(command_v)) {
        commanded = true;
      } else {
        command_lower = std::min(command_lower, command);
        command_upper = std::max(command_upper, command);
        // A constant nonzero velocity command is motion, not a still command.
        commanded |= std::abs(command_v) > kStillCommandEpsilon;
      }
    }
    // Stamps strictly increase (Observe resets otherwise) and the window
    // holds at least two samples; fail closed if that ever stops holding.
    const double n = static_cast<double>(samples);
    const double denominator = n * sum_tt - sum_t * sum_t;
    if (!(denominator > 0.0)) {return RelaxCheck::HistoryMissing;}
    const double slope = (n * sum_tp - sum_t * sum_p) / denominator;
    moving |= upper - lower > pose.max_span_rad;
    moving |= std::abs(slope) > pose.max_abs_velocity_rad_s;
    commanded |= command_upper - command_lower > kStillCommandEpsilon;
  }
  if (moving) {return RelaxCheck::MeasuredMoving;}
  return commanded ? RelaxCheck::CommandMoving : RelaxCheck::Safe;
}

RelaxVerdict StillnessHistory::Check(
  const std::vector<RelaxSafePose> & poses, const std::vector<double> & measured) const
{
  RelaxVerdict result{poses.empty() ? RelaxCheck::NotConfigured : RelaxCheck::PoseMismatch, 0};
  for (size_t p = 0; p < poses.size(); ++p) {
    const auto & pose = poses[p];
    bool match = true;
    for (size_t joint = 0; joint < joints_; ++joint) {
      match &= std::isfinite(measured[joint]) &&
        std::abs(measured[joint] - pose.position[joint]) <= pose.tolerance_rad[joint];
    }
    if (!match) {continue;}
    result = {CheckWindow(pose), p};
    if (result.check == RelaxCheck::Safe) {return result;}
  }
  return result;
}

MeasuredHold::MeasuredHold(size_t joints)
: position_(joints), blocked_command_(joints), blocked_velocity_(joints), seeded_command_(joints),
  latched_(joints)
{
}

void MeasuredHold::Reset() {mode_ = Mode::Normal;}

void MeasuredHold::Relax(
  const std::vector<double> & command, const std::vector<double> & velocity)
{
  std::copy(command.begin(), command.end(), blocked_command_.begin());
  std::copy(velocity.begin(), velocity.end(), blocked_velocity_.begin());
  mode_ = Mode::Relaxed;
}

void MeasuredHold::Engage(
  const std::vector<double> & measured, const std::vector<double> & command,
  const std::vector<double> & velocity)
{
  std::copy(measured.begin(), measured.end(), position_.begin());
  std::copy(measured.begin(), measured.end(), seeded_command_.begin());
  std::fill(latched_.begin(), latched_.end(), true);
  std::copy(command.begin(), command.end(), blocked_command_.begin());
  std::copy(velocity.begin(), velocity.end(), blocked_velocity_.begin());
  mode_ = Mode::Engaging;
}

bool MeasuredHold::ObserveCommand(
  const std::vector<double> & command, const std::vector<double> & velocity)
{
  if (mode_ == Mode::Normal) {return false;}
  bool changed = false;
  const bool finite = std::all_of(command.begin(), command.end(),
      [](double v) {return std::isfinite(v);}) &&
    std::all_of(velocity.begin(), velocity.end(), [](double v) {return std::isfinite(v);});
  for (size_t i = 0; i < command.size(); ++i) {
    const bool new_position = Different(command[i], blocked_command_[i]) &&
      (mode_ == Mode::Relaxed || Different(command[i], seeded_command_[i]));
    const bool new_velocity = Different(velocity[i], blocked_velocity_[i]) &&
      (mode_ == Mode::Relaxed || std::abs(velocity[i]) > kStillCommandEpsilon);
    changed |= new_position || new_velocity;
    if (mode_ == Mode::Holding && finite && (new_position || new_velocity)) {
      // A new J7 target must not release stale holds on J1..J6 (or vice versa).
      latched_[i] = false;
    }
    if (mode_ == Mode::Engaging) {
      // Ignore the hardware's own reseed; remember commands from producers
      // during enable so completion cannot release them as fresh motion.
      if (Different(command[i], seeded_command_[i])) {blocked_command_[i] = command[i];}
      if (std::abs(velocity[i]) > kStillCommandEpsilon) {blocked_velocity_[i] = velocity[i];}
    }
  }
  if (mode_ == Mode::Holding &&
    std::none_of(latched_.begin(), latched_.end(), [](bool latch) {return latch;}))
  {
    mode_ = Mode::Normal;
  } else if (mode_ == Mode::Relaxed && changed) {
    std::copy(command.begin(), command.end(), blocked_command_.begin());
    std::copy(velocity.begin(), velocity.end(), blocked_velocity_.begin());
    return true;
  }
  return false;
}

void MeasuredHold::TrackDisabled(size_t joint, double measured)
{
  if (mode_ == Mode::Relaxed || mode_ == Mode::Engaging) {position_[joint] = measured;}
}

void MeasuredHold::ConfirmEngaged()
{
  if (mode_ == Mode::Engaging) {mode_ = Mode::Holding;}
}

void MeasuredHold::AbortEngage(bool restore_relaxed)
{
  mode_ = restore_relaxed ? Mode::Relaxed : Mode::Normal;
}

bool MeasuredHold::relaxed() const {return mode_ == Mode::Relaxed;}
bool MeasuredHold::holding() const {return mode_ == Mode::Engaging || mode_ == Mode::Holding;}
bool MeasuredHold::holding(size_t joint) const {return holding() && latched_[joint];}
double MeasuredHold::position(size_t joint) const {return position_[joint];}

}  // namespace robstride_hardware_interface
