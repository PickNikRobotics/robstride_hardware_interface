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
//
// The motion-mode torque feedforward is bounded by max_feedforward_effort,
// whose default stays below each bench motor's rated torque.

#include <gtest/gtest.h>

#include "robstride_hardware_interface/robstride_hardware_interface.hpp"

using robstride_hardware_interface::DefaultMaxFeedforwardEffort;
using robstride_hardware_interface::JointHandle;
using robstride_hardware_interface::MotorFeedforward;
using robstride_sdk::ActuatorType;

TEST(Feedforward, DefaultBoundIsBelowRatedTorque)
{
  EXPECT_DOUBLE_EQ(DefaultMaxFeedforwardEffort(ActuatorType::RS00), 3.5);
  EXPECT_LT(DefaultMaxFeedforwardEffort(ActuatorType::RS00), 5.0);
  EXPECT_DOUBLE_EQ(DefaultMaxFeedforwardEffort(ActuatorType::RS06), 9.0);
  EXPECT_LT(DefaultMaxFeedforwardEffort(ActuatorType::RS06), 11.0);
}

TEST(Feedforward, ClampsToTheBoundAndMapsToTheMotorFrame)
{
  JointHandle jh{};
  jh.max_feedforward_effort = 9.0;
  EXPECT_DOUBLE_EQ(MotorFeedforward(jh, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(MotorFeedforward(jh, 4.2), 4.2);
  EXPECT_DOUBLE_EQ(MotorFeedforward(jh, 20.0), 9.0);
  EXPECT_DOUBLE_EQ(MotorFeedforward(jh, -20.0), -9.0);

  jh.direction = -1.0;
  EXPECT_DOUBLE_EQ(MotorFeedforward(jh, 4.2), -4.2);
  EXPECT_DOUBLE_EQ(MotorFeedforward(jh, 20.0), -9.0);

  jh.max_feedforward_effort = 0.0;
  EXPECT_DOUBLE_EQ(MotorFeedforward(jh, 5.0), 0.0);
}
