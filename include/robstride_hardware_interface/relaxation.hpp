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

#ifndef ROBSTRIDE_HARDWARE_INTERFACE__RELAXATION_HPP_
#define ROBSTRIDE_HARDWARE_INTERFACE__RELAXATION_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robstride_hardware_interface
{

// Command stability is deliberately much tighter than the supported-pose
// tolerance. Effort-only gravity compensation is not a motion command.
constexpr double kStillCommandEpsilon = 1.0e-4;
constexpr double kMaxStillnessWindowS = 10.0;
constexpr uint64_t kMaxStillnessSampleGapNs = 100'000'000ull;

struct RelaxSafePose
{
  std::string name;
  std::vector<double> position;
  std::vector<double> tolerance_rad;
  double window_s = 0.0;
  double max_span_rad = 0.0;
  // Bound on the window's least-squares measured-position slope, not on raw
  // motor velocity feedback (which is only required to be finite).
  double max_abs_velocity_rad_s = 0.01;
};

// Called once, before starting the executor. No built-in robot poses. Throws
// on malformed/ambiguous YAML, including missing, extra or duplicate joints.
std::vector<RelaxSafePose> LoadRelaxSafePoses(
  const std::string & file, const std::vector<std::string> & joint_names);

enum class RelaxCheck
{
  Safe,
  NotConfigured,
  PoseMismatch,
  HistoryMissing,
  MeasuredMoving,
  CommandMoving,
  Unhealthy
};

struct RelaxVerdict
{
  RelaxCheck check = RelaxCheck::NotConfigured;
  size_t pose = 0;
};

// RT-owned, preallocated ring. Observe is O(joints); Check scans only on a
// service request. Overrun or an update gap cannot manufacture a still window.
class StillnessHistory
{
public:
  explicit StillnessHistory(size_t joints, size_t capacity = 10'002);
  void Reset();
  void Observe(
    uint64_t now_ns, const std::vector<double> & position,
    const std::vector<double> & velocity, const std::vector<double> & command_position,
    const std::vector<double> & command_velocity, bool healthy);
  RelaxVerdict Check(
    const std::vector<RelaxSafePose> & poses, const std::vector<double> & measured) const;

private:
  size_t joints_;
  size_t capacity_;
  size_t next_ = 0;
  size_t count_ = 0;
  std::vector<uint64_t> stamp_;
  std::vector<bool> healthy_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> command_position_;
  std::vector<double> command_velocity_;
  RelaxCheck CheckWindow(const RelaxSafePose & pose) const;
};

// Re-seed command AND hold state. Updating just the exported interface is not
// enough: an active controller may rewrite its pre-relax setpoint each cycle.
// After confirmation, that repeated setpoint stays suppressed until a genuinely
// new finite position/velocity command arrives. Effort alone does not release.
class MeasuredHold
{
public:
  explicit MeasuredHold(size_t joints);
  void Reset();
  void Relax(const std::vector<double> & command, const std::vector<double> & velocity);
  void Engage(
    const std::vector<double> & measured, const std::vector<double> & command,
    const std::vector<double> & velocity);
  // Returns true only for a changed command ignored while relaxed.
  bool ObserveCommand(const std::vector<double> & command, const std::vector<double> & velocity);
  void TrackDisabled(size_t joint, double measured);
  void ConfirmEngaged();
  void AbortEngage(bool restore_relaxed);
  bool relaxed() const;
  bool holding() const;
  bool holding(size_t joint) const;
  double position(size_t joint) const;

private:
  enum class Mode {Normal, Relaxed, Engaging, Holding};
  Mode mode_ = Mode::Normal;
  std::vector<double> position_;
  std::vector<double> blocked_command_;
  std::vector<double> blocked_velocity_;
  std::vector<double> seeded_command_;
  std::vector<bool> latched_;
};

}  // namespace robstride_hardware_interface

#endif  // ROBSTRIDE_HARDWARE_INTERFACE__RELAXATION_HPP_
