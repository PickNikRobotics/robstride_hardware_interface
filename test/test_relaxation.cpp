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

#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "robstride_hardware_interface/relaxation.hpp"
#include "robstride_hardware_interface/can_tx_retry.hpp"

namespace
{
using robstride_hardware_interface::LoadRelaxSafePoses;
using robstride_hardware_interface::MeasuredHold;
using robstride_hardware_interface::RelaxCheck;
using robstride_hardware_interface::RelaxSafePose;
using robstride_hardware_interface::StillnessHistory;

class YamlFile
{
public:
  explicit YamlFile(const std::string & contents)
  {
    char name[] = "/tmp/robstride-relax-unit-XXXXXX";
    const int fd = ::mkstemp(name);
    if (fd < 0) {throw std::runtime_error("mkstemp failed");}
    ::close(fd);
    path = name;
    std::ofstream(path) << contents;
  }
  ~YamlFile() {::unlink(path.c_str());}
  std::string path;
};

std::string Yaml(const std::string & position = "0.2", const std::string & tolerance = "0.08")
{
  return "version: 1\nrelax_safe_poses:\n"
         "  - name: supported\n    joints:\n      a: {position_rad: " + position +
         ", tolerance_rad: " + tolerance + "}\n"
         "      b: {position_rad: -0.1, tolerance_rad: 0.02}\n"
         "    stillness: {window_s: 0.5, max_span_rad: 0.005}\n";
}

TEST(RelaxConfiguration, LoadsNamedPosesInHardwareJointOrderWithoutBuiltInHome)
{
  YamlFile file(Yaml());
  const auto poses = LoadRelaxSafePoses(file.path, {"b", "a"});
  ASSERT_EQ(poses.size(), 1u);
  EXPECT_EQ(poses[0].name, "supported");
  EXPECT_EQ(poses[0].position, (std::vector<double>{-.1, .2}));
  EXPECT_EQ(poses[0].tolerance_rad, (std::vector<double>{.02, .08}));
  EXPECT_DOUBLE_EQ(poses[0].window_s, .5);
  EXPECT_DOUBLE_EQ(poses[0].max_abs_velocity_rad_s, .01);
}

TEST(RelaxConfiguration, RequiresAbsoluteExistingFileAndAllConfiguredJoints)
{
  EXPECT_THROW(LoadRelaxSafePoses("relative.yaml", {"a", "b"}), std::exception);
  EXPECT_THROW(LoadRelaxSafePoses("/does-not-exist/relax.yaml", {"a", "b"}), std::exception);
  YamlFile file(Yaml());
  EXPECT_THROW(LoadRelaxSafePoses(file.path, {"a"}), std::exception);
  EXPECT_THROW(LoadRelaxSafePoses(file.path, {"a", "b", "c"}), std::exception);
  EXPECT_THROW(LoadRelaxSafePoses(file.path, {"a", "a"}), std::exception);
}

TEST(RelaxConfiguration, RejectsNonfinitePoseAndNonpositiveTolerance)
{
  for (const std::string number : {".nan", ".inf", "-.inf", "bad"}) {
    YamlFile file(Yaml(number));
    EXPECT_THROW(LoadRelaxSafePoses(file.path, {"a", "b"}), std::exception) << number;
  }
  for (const std::string number : {"0", "-0.1", ".nan", ".inf"}) {
    YamlFile file(Yaml("0", number));
    EXPECT_THROW(LoadRelaxSafePoses(file.path, {"a", "b"}), std::exception) << number;
  }
}

TEST(RelaxConfiguration, RejectsDuplicateUnknownAndMissingKeysOrVersions)
{
  const std::vector<std::string> invalid{
    "version: 2\nrelax_safe_poses: []",
    "version: 1\nversion: 1\nrelax_safe_poses: []",
    "version: 1\nrelax_safe_poses: []",
    "version: 1\nrelax_safe_poses: {}",
    Yaml() + "unexpected: 1\n",
    Yaml() + "---\nversion: 1\nrelax_safe_poses: []\n"};
  for (const auto & contents : invalid) {
    YamlFile file(contents);
    EXPECT_THROW(LoadRelaxSafePoses(file.path, {"a", "b"}), std::exception);
  }
  auto duplicate = Yaml();
  duplicate.insert(duplicate.find("    stillness:"),
    "      a: {position_rad: 0, tolerance_rad: 0.08}\n");
  YamlFile file(duplicate);
  EXPECT_THROW(LoadRelaxSafePoses(file.path, {"a", "b"}), std::exception);
}

TEST(RelaxConfiguration, RejectsDuplicateOrEmptyPoseNamesAndInvalidStillness)
{
  auto duplicate = Yaml();
  duplicate += duplicate.substr(duplicate.find("  - name:"));
  YamlFile duplicate_file(duplicate);
  EXPECT_THROW(LoadRelaxSafePoses(duplicate_file.path, {"a", "b"}), std::exception);
  for (const auto & change : std::vector<std::pair<std::string, std::string>>{
      {"name: supported", "name: ''"}, {"window_s: 0.5", "window_s: 0"},
      {"window_s: 0.5", "window_s: 11"}, {"max_span_rad: 0.005", "max_span_rad: .nan"},
      {"max_span_rad: 0.005", "max_span_rad: -0.1"},
      {"max_span_rad: 0.005", "max_span_rad: 0.005, max_abs_velocity_rad_s: 0"}})
  {
    auto contents = Yaml();
    contents.replace(contents.find(change.first), change.first.size(), change.second);
    YamlFile file(contents);
    EXPECT_THROW(LoadRelaxSafePoses(file.path, {"a", "b"}), std::exception) << change.second;
  }
}

RelaxSafePose Pose()
{
  return {"supported", {0, 0, 0, 0, 0, 0, 0}, {.08, .08, .08, .08, .08, .08, .02},
    .5, .005, .01};
}

void Stationary(StillnessHistory & history, const std::vector<double> & measured)
{
  const std::vector<double> zero(7, 0);
  for (uint64_t time = 1'000'000'000ull; time <= 1'500'000'000ull; time += 10'000'000ull) {
    history.Observe(time, measured, zero, measured, zero, true);
  }
}

TEST(RelaxSafety, EveryJointUsesItsOwnInclusiveTolerance)
{
  const auto pose = Pose();
  for (size_t joint = 0; joint < 7; ++joint) {
    for (double sign : {-1., 1.}) {
      std::vector<double> measured(7, 0);
      measured[joint] = sign * pose.tolerance_rad[joint];
      StillnessHistory history(7);
      Stationary(history, measured);
      EXPECT_EQ(history.Check({pose}, measured).check, RelaxCheck::Safe) << joint;
      measured[joint] += sign * 1e-6;
      EXPECT_EQ(history.Check({pose}, measured).check, RelaxCheck::PoseMismatch) << joint;
    }
  }
}

TEST(RelaxSafety, NeedsCompleteWindowAndRejectsGapsClockResetAndRingOverrun)
{
  StillnessHistory history(7);
  const auto pose = Pose();
  const std::vector<double> zero(7, 0);
  EXPECT_EQ(history.Check({}, zero).check, RelaxCheck::NotConfigured);
  EXPECT_EQ(history.Check({pose}, zero).check, RelaxCheck::HistoryMissing);
  history.Observe(1'000'000'000ull, zero, zero, zero, zero, true);
  EXPECT_EQ(history.Check({pose}, zero).check, RelaxCheck::HistoryMissing);
  Stationary(history, zero);
  EXPECT_EQ(history.Check({pose}, zero).check, RelaxCheck::Safe);
  history.Observe(2'000'000'000ull, zero, zero, zero, zero, true);
  EXPECT_EQ(history.Check({pose}, zero).check, RelaxCheck::HistoryMissing);
  history.Observe(1'000'000'000ull, zero, zero, zero, zero, true);
  EXPECT_EQ(history.Check({pose}, zero).check, RelaxCheck::HistoryMissing);
  StillnessHistory small(7, 3);
  Stationary(small, zero);
  EXPECT_EQ(small.Check({pose}, zero).check, RelaxCheck::HistoryMissing);
}

TEST(RelaxSafety, RejectsMeasuredMotionAndVelocityIncludingSeventhJoint)
{
  const std::vector<double> zero(7, 0);
  for (bool velocity : {false, true}) {
    auto pose = Pose();
    // A wide span bound isolates the velocity gate: 0.011 rad/s drift over
    // 0.5 s is 0.0055 rad, inside 0.05 rad span but over 0.01 rad/s.
    if (velocity) {pose.max_span_rad = .05;}
    StillnessHistory history(7);
    std::vector<double> position = zero;
    for (uint64_t i = 0; i <= 50; ++i) {
      position = zero;
      if (velocity) {position[6] = i * .00011;} else if (i == 25) {position[6] = .006;}
      history.Observe(1'000'000'000ull + i * 10'000'000ull, position, zero, zero, zero, true);
    }
    EXPECT_EQ(history.Check({pose}, position).check, RelaxCheck::MeasuredMoving) << velocity;
  }
}

TEST(RelaxSafety, NoisyRawVelocityFeedbackAtRestIsStationary)
{
  // Bench idle at HOME: raw Type-2 velocity up to 0.15 rad/s (median 0.026)
  // while position stays within one 0.00038 rad feedback quantum.
  const std::vector<double> zero(7, 0);
  const double raw[] = {.127, -.045, -.080, .017, -.007, -.026, -.016, .149, -.149};
  StillnessHistory history(7);
  for (uint64_t i = 0; i <= 50; ++i) {
    std::vector<double> position(7, 0), speed(7, 0);
    for (size_t joint = 0; joint < 7; ++joint) {
      position[joint] = ((i + joint) % 3 == 0) ? .00038 : 0.0;
      speed[joint] = raw[(i + joint) % 9];
    }
    history.Observe(1'000'000'000ull + i * 10'000'000ull, position, speed, zero, zero, true);
  }
  EXPECT_EQ(history.Check({Pose()}, zero).check, RelaxCheck::Safe);
}

TEST(RelaxSafety, RejectsChangingCommandsEvenWithStationaryMeasurements)
{
  const std::vector<double> zero(7, 0);
  for (bool velocity : {false, true}) {
    StillnessHistory history(7);
    for (uint64_t i = 0; i <= 50; ++i) {
      auto command = zero;
      auto speed = zero;
      if (velocity) {speed[6] = .001;} else {command[6] = i * .00001;}
      history.Observe(1'000'000'000ull + i * 10'000'000ull, zero, zero, command, speed, true);
    }
    EXPECT_EQ(history.Check({Pose()}, zero).check, RelaxCheck::CommandMoving);
  }
}

TEST(RelaxSafety, RejectsUnhealthyOrNonfiniteSamplesAnywhereInWindow)
{
  const std::vector<double> zero(7, 0);
  for (bool nonfinite : {false, true}) {
    StillnessHistory history(7);
    for (uint64_t i = 0; i <= 50; ++i) {
      auto measured = zero;
      if (nonfinite && i == 25) {measured[6] = std::numeric_limits<double>::quiet_NaN();}
      history.Observe(1'000'000'000ull + i * 10'000'000ull, measured, zero, zero, zero,
        nonfinite || i != 25);
    }
    EXPECT_EQ(history.Check({Pose()}, zero).check, RelaxCheck::Unhealthy);
  }
}

TEST(RelaxSafety, ChoosesTheNamedMatchingPoseAndCanUseASecondWindow)
{
  auto other = Pose();
  other.name = "other";
  other.position[0] = .4;
  StillnessHistory history(7);
  Stationary(history, other.position);
  const auto verdict = history.Check({Pose(), other}, other.position);
  EXPECT_EQ(verdict.check, RelaxCheck::Safe);
  EXPECT_EQ(verdict.pose, 1u);
}

TEST(MeasuredHold, IgnoresRelaxedCommandsTracksMeasuredAndReportsChanges)
{
  const std::vector<double> old(7, 0);
  const std::vector<double> zero(7, 0);
  MeasuredHold hold(7);
  hold.Relax(old, zero);
  EXPECT_FALSE(hold.ObserveCommand(old, zero));
  auto commanded = old;
  commanded[6] = 1;
  EXPECT_TRUE(hold.ObserveCommand(commanded, zero));
  EXPECT_FALSE(hold.ObserveCommand(commanded, zero));
  hold.TrackDisabled(6, .03);
  EXPECT_DOUBLE_EQ(hold.position(6), .03);
  EXPECT_TRUE(hold.relaxed());
}

TEST(MeasuredHold, EngageHoldsMeasuredDespiteRepeatedStaleAndHardwareReseedCommands)
{
  const std::vector<double> old(7, 0);
  const std::vector<double> measured(7, .03);
  const std::vector<double> zero(7, 0);
  MeasuredHold hold(7);
  hold.Relax(old, zero);
  hold.Engage(measured, old, zero);
  hold.ObserveCommand(measured, zero);  // hardware reseeds exported command
  hold.ObserveCommand(old, zero);      // controller repeats obsolete target
  hold.ConfirmEngaged();
  for (int cycle = 0; cycle < 20; ++cycle) {
    hold.ObserveCommand(cycle % 2 ? old : measured, zero);
    ASSERT_TRUE(hold.holding());
    for (size_t i = 0; i < 7; ++i) {
      EXPECT_DOUBLE_EQ(hold.position(i), measured[i]);
    }
  }
  auto next = measured;
  next[6] = .1;
  hold.ObserveCommand(next, zero);
  EXPECT_FALSE(hold.holding(6));
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_TRUE(hold.holding(i));
  }
  hold.ObserveCommand(std::vector<double>(7, .2), zero);
  EXPECT_FALSE(hold.holding());
}

TEST(MeasuredHold, CommandsReceivedDuringEngageDoNotBecomeMotionOnConfirmation)
{
  const std::vector<double> zero(7, 0);
  const std::vector<double> measured(7, .03);
  const std::vector<double> queued(7, 1);
  MeasuredHold hold(7);
  hold.Engage(measured, zero, zero);
  hold.ObserveCommand(queued, zero);
  hold.ConfirmEngaged();
  hold.ObserveCommand(queued, zero);
  EXPECT_TRUE(hold.holding());
  auto invalid = queued;
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  hold.ObserveCommand(invalid, zero);
  EXPECT_TRUE(hold.holding());
  hold.AbortEngage(true);
  EXPECT_TRUE(hold.relaxed());
  hold.Reset();
  EXPECT_FALSE(hold.relaxed());
  EXPECT_FALSE(hold.holding());
}
TEST(CanTxRetry, QueuePressureHasBoundedBudgetAndSuccessfulSendClearsIt)
{
  robstride_hardware_interface::CanTxRetry retry;
  constexpr uint64_t start = 1'000'000'000ull;
  retry.Observe(start, false);
  EXPECT_FALSE(retry.Failed(start, ENOBUFS));
  EXPECT_FALSE(retry.Failed(start + 29'000'000ull, EAGAIN));
  EXPECT_TRUE(retry.Failed(start + 30'000'000ull, ENOBUFS));
  EXPECT_TRUE(retry.Failed(start, ENETDOWN));
  EXPECT_TRUE(retry.Failed(start, EBADF));
  retry.Observe(start + 31'000'000ull, true);
  EXPECT_FALSE(retry.Failed(start + 32'000'000ull, 0));
  retry.Observe(start + 33'000'000ull, false);
  EXPECT_FALSE(retry.Failed(start + 33'000'000ull, ENOBUFS));
}
}  // namespace
