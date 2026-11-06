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

#include <cmath>
#include <limits>

#include "robstride_hardware_interface/joint_coordinates.hpp"

using robstride_hardware_interface::JointToMotorTarget;
using robstride_hardware_interface::PositionWithinBounds;
using robstride_hardware_interface::WrapToWindow;

TEST(JointCoordinates, WrapWindowIsHalfOpenAndCenteredOnLimits)
{
  EXPECT_DOUBLE_EQ(WrapToWindow(M_PI, 0), -M_PI);
  EXPECT_DOUBLE_EQ(WrapToWindow(-M_PI, 0), -M_PI);
  EXPECT_NEAR(WrapToWindow(1.2 + 2 * M_PI, 1.5), 1.2, 1e-12);
  EXPECT_NEAR(WrapToWindow(1.2 - 4 * M_PI, 1.5), 1.2, 1e-12);
}

TEST(JointCoordinates, DirectionsAndWholeTurnOffsetsRoundTripWithoutWireEncoding)
{
  for (double direction : {-1., 1.}) {
    for (int turn = -1; turn <= 1; ++turn) {
      const double motor = direction * .4 + turn * 2 * M_PI;
      const double measured = WrapToWindow(direction * motor, 0);
      const double offset = direction * motor - measured;
      EXPECT_NEAR(measured, .4, 1e-12);
      EXPECT_NEAR(JointToMotorTarget(.4, measured, offset, direction, false), motor, 1e-12);
      EXPECT_NEAR(JointToMotorTarget(.6, measured, offset, direction, false),
        motor + direction * .2, 1e-12);
    }
  }
}

TEST(JointCoordinates, ContinuousTargetTakesShortestPathOnlyForUnboundedJoint)
{
  EXPECT_NEAR(JointToMotorTarget(-3., 3., 0, 1, true), 2 * M_PI - 3., 1e-12);
  EXPECT_DOUBLE_EQ(JointToMotorTarget(-3., 3., 0, 1, false), -3.);
}

TEST(JointCoordinates, BoundsAreInclusiveWithExplicitFeedbackTolerance)
{
  EXPECT_TRUE(PositionWithinBounds(-.02, -.02, 3.5, .0004));
  EXPECT_TRUE(PositionWithinBounds(3.5, -.02, 3.5, .0004));
  EXPECT_TRUE(PositionWithinBounds(-.0203, -.02, 3.5, .0004));
  EXPECT_FALSE(PositionWithinBounds(-.021, -.02, 3.5, .0004));
  EXPECT_FALSE(PositionWithinBounds(3.501, -.02, 3.5, .0004));
  EXPECT_FALSE(PositionWithinBounds(std::numeric_limits<double>::quiet_NaN(), -.02, 3.5, .0004));
  EXPECT_FALSE(PositionWithinBounds(std::numeric_limits<double>::infinity(), -.02, 3.5, .0004));
}
