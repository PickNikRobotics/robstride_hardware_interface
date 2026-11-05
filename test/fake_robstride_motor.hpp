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

#ifndef FAKE_ROBSTRIDE_MOTOR_HPP_
#define FAKE_ROBSTRIDE_MOTOR_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "robstride_sdk/robstride_protocol.hpp"

namespace robstride_test
{

inline const char * TestInterface()
{
  const char * name = std::getenv("ROBSTRIDE_TEST_CAN_IFACE");
  return name ? name : "vcan0";
}

// Name alone is not a safety boundary: confirm the kernel's link kind before
// opening ANY CAN socket. Never allow a physical interface renamed to vcan0.
inline bool IsVirtualCan(const std::string & name)
{
  if (name.rfind("vcan", 0) != 0 || name.size() >= IFNAMSIZ) {return false;}
  const unsigned int index = ::if_nametoindex(name.c_str());
  if (index == 0) {return false;}
  const int fd = ::socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
  if (fd < 0) {return false;}
  timeval timeout{};
  timeout.tv_sec = 2;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  struct Request {nlmsghdr header; ifinfomsg info;} request{};
  request.header.nlmsg_len = sizeof(request);
  request.header.nlmsg_type = RTM_GETLINK;
  request.header.nlmsg_flags = NLM_F_REQUEST;
  request.header.nlmsg_seq = 1;
  request.info.ifi_index = static_cast<int>(index);
  sockaddr_nl address{};
  address.nl_family = AF_NETLINK;
  const auto sent = ::sendto(fd, &request, sizeof(request), 0,
      reinterpret_cast<sockaddr *>(&address), sizeof(address));
  alignas(nlmsghdr) std::array<char, 8192> response{};
  const auto bytes = sent == sizeof(request) ?
    ::recv(fd, response.data(), response.size(), 0) : -1;
  ::close(fd);
  if (bytes < static_cast<ssize_t>(NLMSG_LENGTH(sizeof(ifinfomsg)))) {return false;}
  const auto * header = reinterpret_cast<const nlmsghdr *>(response.data());
  if (header->nlmsg_type != RTM_NEWLINK || header->nlmsg_len > static_cast<size_t>(bytes)) {
    return false;
  }
  const auto * info = reinterpret_cast<const ifinfomsg *>(NLMSG_DATA(header));
  int length = IFLA_PAYLOAD(header);
  for (auto * attribute = IFLA_RTA(info); RTA_OK(attribute, length);
    attribute = RTA_NEXT(attribute, length))
  {
    if ((attribute->rta_type & NLA_TYPE_MASK) != IFLA_LINKINFO) {continue;}
    int nested_length = RTA_PAYLOAD(attribute);
    for (auto * nested = reinterpret_cast<rtattr *>(RTA_DATA(attribute));
      RTA_OK(nested, nested_length); nested = RTA_NEXT(nested, nested_length))
    {
      if ((nested->rta_type & NLA_TYPE_MASK) == IFLA_INFO_KIND &&
        RTA_PAYLOAD(nested) == 5 && std::memcmp(RTA_DATA(nested), "vcan", 5) == 0)
      {return true;}
    }
  }
  return false;
}

// Used only to inject a transmit failure on an already verified virtual link.
inline bool SetVirtualCanUp(const std::string & name, bool up)
{
  if (!IsVirtualCan(name)) {return false;}
  const int fd = ::socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
  if (fd < 0) {return false;}
  timeval timeout{};
  timeout.tv_sec = 2;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  struct Request {nlmsghdr header; ifinfomsg info;} request{};
  request.header.nlmsg_len = sizeof(request);
  request.header.nlmsg_type = RTM_NEWLINK;
  request.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
  request.header.nlmsg_seq = 1;
  request.info.ifi_index = static_cast<int>(::if_nametoindex(name.c_str()));
  request.info.ifi_flags = up ? IFF_UP : 0;
  request.info.ifi_change = IFF_UP;
  sockaddr_nl address{};
  address.nl_family = AF_NETLINK;
  const auto sent = ::sendto(fd, &request, sizeof(request), 0,
      reinterpret_cast<sockaddr *>(&address), sizeof(address));
  alignas(nlmsghdr) std::array<char, 1024> response{};
  const auto bytes = sent == sizeof(request) ?
    ::recv(fd, response.data(), response.size(), 0) : -1;
  ::close(fd);
  if (bytes < static_cast<ssize_t>(NLMSG_LENGTH(sizeof(nlmsgerr)))) {return false;}
  const auto * header = reinterpret_cast<const nlmsghdr *>(response.data());
  return header->nlmsg_type == NLMSG_ERROR &&
         reinterpret_cast<const nlmsgerr *>(NLMSG_DATA(header))->error == 0;
}

inline uint64_t SteadyNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Sample
{
  uint64_t stamp_ns = 0;
  uint8_t type = 0;
  uint16_t parameter = 0;
  double position = 0;
  double before = 0;
  double target = 0;
  double kp = 0;
  double kd = 0;
  double effort = 0;
  bool enabled = false;
};

// Independent wire implementation: no SDK Encode/Decode or quantizer is used.
// This ideal follower is NOT a thermal, gravity, load or friction simulation.
class FakeRobstrideMotor
{
public:
  FakeRobstrideMotor(
    const std::string & interface, uint8_t motor_id, uint8_t master_id,
    robstride_sdk::ActuatorType type = robstride_sdk::ActuatorType::RS00)
  : motor_id_(motor_id), master_id_(master_id), limits_(robstride_sdk::GetActuatorLimits(type))
  {
    if (!IsVirtualCan(interface)) {return;}
    socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {return;}
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(::if_nametoindex(interface.c_str()));
    if (::bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      ::close(socket_);
      socket_ = -1;
      return;
    }
    timeval timeout{};
    timeout.tv_usec = 10000;
    ::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    running_ = true;
    thread_ = std::thread(&FakeRobstrideMotor::Run, this);
  }

  ~FakeRobstrideMotor()
  {
    running_ = false;
    if (thread_.joinable()) {thread_.join();}
    if (socket_ >= 0) {::close(socket_);}
  }

  bool ok() const {return socket_ >= 0;}
  void set_position(double value) {position_ = value;}
  void set_velocity(double value) {velocity_ = value;}
  void set_effort(double value) {effort_ = value;}
  void set_temperature(double value) {temperature_ = value;}
  void set_silent(bool value) {silent_ = value;}
  void set_follow(bool value) {follow_ = value;}
  void set_drop_motion(bool value) {drop_motion_ = value;}
  void set_drop_enable(bool value) {drop_enable_ = value;}
  void set_feedback_dlc(uint8_t value) {feedback_dlc_ = value;}
  void reboot() {enabled_ = false;}
  double position() const {return position_;}
  bool enabled() const {return enabled_;}
  uint32_t watchdog_raw() const {return watchdog_raw_;}
  uint8_t mode() const {return mode_;}
  uint64_t replies() const {return replies_;}
  double applied_kp() const {return applied_kp_;}
  double applied_kd() const {return applied_kd_;}

  std::vector<Sample> samples() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return samples_;
  }

  std::vector<uint8_t> received() const
  {
    std::vector<uint8_t> types;
    for (const auto & sample : samples()) {types.push_back(sample.type);}
    return types;
  }

  void set_fault(bool fault) {set_fault_word(fault ? 1u : 0u);}
  void set_fault_word(uint32_t word)
  {
    fault_word_ = word;
    if (word != 0 && !silent_) {
      can_frame frame{};
      frame.can_id = CAN_EFF_FLAG | (0x15u << 24) | (motor_id_ << 8) | master_id_;
      frame.can_dlc = 8;
      for (size_t i = 0; i < 4; ++i) {frame.data[i] = (word >> (8 * i)) & 0xFF;}
      [[maybe_unused]] const auto written = ::write(socket_, &frame, sizeof(frame));
    }
  }

private:
  static double Decode(uint16_t value, double minimum, double maximum)
  {return minimum + value * (maximum - minimum) / 65535.0;}
  static uint16_t Encode(double value, double minimum, double maximum)
  {
    return static_cast<uint16_t>(std::lround((std::clamp(value, minimum, maximum) - minimum) *
           65535.0 / (maximum - minimum)));
  }
  static uint16_t U16(const uint8_t * data)
  {return static_cast<uint16_t>((data[0] << 8) | data[1]);}
  static void Put(uint8_t * data, uint16_t value)
  {data[0] = value >> 8; data[1] = value & 0xFF;}

  void Run()
  {
    while (running_) {
      can_frame frame{};
      const auto bytes = ::read(socket_, &frame, sizeof(frame));
      if (watchdog_raw_ != 0 && enabled_ &&
        SteadyNs() - last_command_ns_ > watchdog_raw_ * 50000ull)
      {enabled_ = false;}
      if (bytes != sizeof(frame) || !(frame.can_id & CAN_EFF_FLAG) ||
        (frame.can_id & 0xFF) != motor_id_ || frame.can_dlc != 8)
      {continue;}
      const uint8_t type = (frame.can_id >> 24) & 0x1F;
      last_command_ns_ = SteadyNs();
      uint16_t parameter = 0;
      const double before = position_;
      double target = before;
      double kp = 0, kd = 0, effort = 0;
      if (type == 3 && !drop_enable_) {
        enabled_ = true;
      } else if (type == 4) {
        enabled_ = false;
        if (frame.data[0]) {fault_word_ = 0;}
      } else if (type == 1) {
        target = Decode(U16(frame.data), -limits_.position_max, limits_.position_max);
        kp = Decode(U16(frame.data + 4), 0, limits_.kp_max);
        kd = Decode(U16(frame.data + 6), 0, limits_.kd_max);
        effort = Decode((frame.can_id >> 8) & 0xFFFF, -limits_.torque_max, limits_.torque_max);
        if (enabled_ && !drop_motion_ && fault_word_ == 0) {
          applied_kp_ = kp;
          applied_kd_ = kd;
          if (follow_ && kp > 0) {position_ = target;}
        }
      } else if (type == 0x12) {
        parameter = frame.data[0] | (frame.data[1] << 8);
        if (parameter == 0x7005) {mode_ = frame.data[4];}
        if (parameter == 0x7028) {
          uint32_t raw;
          std::memcpy(&raw, frame.data + 4, sizeof(raw));
          watchdog_raw_ = raw;
        }
        if (parameter == 0x7016) {
          float value;
          std::memcpy(&value, frame.data + 4, sizeof(value));
          target = value;
          if (enabled_ && follow_ && !drop_motion_ && fault_word_ == 0) {position_ = target;}
        }
      }
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        samples_.push_back({SteadyNs(), type, parameter, position_, before, target, kp, kd,
            effort, enabled_});
      }
      if (!silent_) {
        // Firmware-like delay: the driver's probe timestamp is recorded AFTER
        // the Stop burst; an instantaneous vcan reply can precede it.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        SendFeedback();
      }
    }
  }

  void SendFeedback()
  {
    can_frame frame{};
    const uint16_t area = motor_id_ | (fault_word_ ? 0x100 : 0) | (enabled_ ? 0x8000 : 0);
    frame.can_id = CAN_EFF_FLAG | (2u << 24) | (area << 8) | master_id_;
    frame.can_dlc = feedback_dlc_;
    Put(frame.data, Encode(position_, -limits_.position_max, limits_.position_max));
    Put(frame.data + 2, Encode(velocity_, -limits_.velocity_max, limits_.velocity_max));
    Put(frame.data + 4, Encode(effort_, -limits_.torque_max, limits_.torque_max));
    Put(frame.data + 6, static_cast<uint16_t>(std::lround(temperature_ * 10)));
    if (::write(socket_, &frame, sizeof(frame)) == sizeof(frame)) {++replies_;}
  }

  const uint8_t motor_id_, master_id_;
  const robstride_sdk::ActuatorLimits limits_;
  int socket_ = -1;
  std::atomic<bool> running_{false}, enabled_{false}, silent_{false}, follow_{false};
  std::atomic<bool> drop_motion_{false}, drop_enable_{false};
  std::atomic<uint8_t> feedback_dlc_{8}, mode_{0};
  std::atomic<uint32_t> fault_word_{0}, watchdog_raw_{0};
  std::atomic<uint64_t> replies_{0};
  uint64_t last_command_ns_ = 0;
  std::atomic<double> position_{0}, temperature_{25}, velocity_{0}, effort_{0};
  std::atomic<double> applied_kp_{0}, applied_kd_{0};
  std::thread thread_;
  mutable std::mutex mutex_;
  std::vector<Sample> samples_;
};

class FakeRobstrideFollower
{
public:
  explicit FakeRobstrideFollower(const std::string & interface)
  {
    for (uint8_t id = 1; id <= 7; ++id) {
      motors_[id - 1] = std::make_unique<FakeRobstrideMotor>(interface, id, 253,
        id <= 3 ? robstride_sdk::ActuatorType::RS06 : robstride_sdk::ActuatorType::RS00);
      motors_[id - 1]->set_follow(true);
    }
  }
  bool ok() const
  {return std::all_of(motors_.begin(), motors_.end(), [](const auto & m) {return m->ok();});}
  FakeRobstrideMotor & motor(size_t index) {return *motors_.at(index);}

private:
  std::array<std::unique_ptr<FakeRobstrideMotor>, 7> motors_;
};

}  // namespace robstride_test

// Existing lifecycle tests keep their one-motor API.
using robstride_test::FakeRobstrideMotor;

#endif  // FAKE_ROBSTRIDE_MOTOR_HPP_
