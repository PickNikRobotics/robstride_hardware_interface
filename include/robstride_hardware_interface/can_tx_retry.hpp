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

#ifndef ROBSTRIDE_HARDWARE_INTERFACE__CAN_TX_RETRY_HPP_
#define ROBSTRIDE_HARDWARE_INTERFACE__CAN_TX_RETRY_HPP_

#include <cerrno>
#include <cstdint>

namespace robstride_hardware_interface
{
// RT-owned error budget: retry on later control cycles, never sleep/poll in
// read()/write(). Complete SDK sends clear last_send_errno; do not mistake one
// short queue-pressure failure for bus-off before that recovery can happen.
class CanTxRetry
{
public:
  static constexpr uint64_t kBudgetNs = 30'000'000ull;
  static bool Transient(int error) {return error == ENOBUFS || error == EAGAIN;}
  void Reset() {first_failure_ns_ = 0;}
  void Observe(uint64_t now_ns, bool complete)
  {
    if (complete) {Reset();} else if (first_failure_ns_ == 0) {first_failure_ns_ = now_ns;}
  }
  bool Failed(uint64_t now_ns, int error) const
  {
    if (error == 0) {return false;}
    if (!Transient(error)) {return true;}
    return first_failure_ns_ != 0 && now_ns >= first_failure_ns_ &&
           now_ns - first_failure_ns_ >= kBudgetNs;
  }

private:
  uint64_t first_failure_ns_ = 0;
};
}  // namespace robstride_hardware_interface

#endif  // ROBSTRIDE_HARDWARE_INTERFACE__CAN_TX_RETRY_HPP_
