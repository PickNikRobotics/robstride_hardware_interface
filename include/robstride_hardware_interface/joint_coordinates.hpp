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

#ifndef ROBSTRIDE_HARDWARE_INTERFACE__JOINT_COORDINATES_HPP_
#define ROBSTRIDE_HARDWARE_INTERFACE__JOINT_COORDINATES_HPP_

#include <cmath>

namespace robstride_hardware_interface
{

// Joint-frame algebra deliberately independent of CAN encoding/quantization.
inline double WrapToWindow(double value, double center)
{
  constexpr double two_pi = 2 * M_PI;
  return value - two_pi * std::floor((value - center + M_PI) / two_pi);
}

inline double JointToMotorTarget(
  double target, double measured, double offset,
  double direction, bool continuous)
{
  double delta = target - measured;
  if (continuous) {delta = WrapToWindow(delta, 0);}
  return direction * (measured + offset + delta);
}

inline bool PositionWithinBounds(double value, double lower, double upper, double tolerance)
{
  return std::isfinite(value) && value >= lower - tolerance && value <= upper + tolerance;
}

}  // namespace robstride_hardware_interface

#endif  // ROBSTRIDE_HARDWARE_INTERFACE__JOINT_COORDINATES_HPP_
