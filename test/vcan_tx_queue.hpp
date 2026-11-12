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

#ifndef VCAN_TX_QUEUE_HPP_
#define VCAN_TX_QUEUE_HPP_

#include <linux/pkt_sched.h>

#include <array>
#include <fstream>
#include <string>
#include <vector>

#include "fake_robstride_motor.hpp"

namespace robstride_test
{

// vcan has no wire serialization: txqueuelen alone does not create pressure.
// A real kernel netem queue with limit=10 and 0.2 ms latency supplies the missing
// drain delay. Every mutation verifies kernel kind=vcan first, adds (never
// replaces) its own qdisc, and restores only its own test configuration.
class LimitedVirtualTxQueue
{
public:
  explicit LimitedVirtualTxQueue(const std::string & name, unsigned int delay_us = 200)
  : name_(name)
  {
    if (!IsVirtualCan(name_) || !ReadQueueLength(original_length_)) {return;}
    if (!SetQueueLength(10)) {return;}
    length_changed_ = true;
    uint32_t us_to_ticks = 0, ticks_to_us = 0, resolution = 0;
    std::ifstream clock("/proc/net/psched");
    clock >> std::hex >> us_to_ticks >> ticks_to_us >> resolution;
    if (!clock || ticks_to_us == 0) {return;}
    const double factor = static_cast<double>(us_to_ticks) / ticks_to_us * resolution / 1e6;
    tc_netem_qopt options{};
    options.latency = static_cast<uint32_t>(delay_us * factor);
    options.limit = 10;
    tcmsg traffic{};
    traffic.tcm_ifindex = static_cast<int>(::if_nametoindex(name_.c_str()));
    traffic.tcm_handle = 0x10000;
    traffic.tcm_parent = TC_H_ROOT;
    std::vector<uint8_t> payload(sizeof(traffic));
    std::memcpy(payload.data(), &traffic, sizeof(traffic));
    const char kind[] = "netem";
    Attribute(payload, TCA_KIND, kind, sizeof(kind));
    Attribute(payload, TCA_OPTIONS, &options, sizeof(options));
    qdisc_owned_ = Request(RTM_NEWQDISC, NLM_F_CREATE | NLM_F_EXCL, payload);
    unsigned int length = 0;
    ok_ = qdisc_owned_ && ReadQueueLength(length) && length == 10;
  }
  ~LimitedVirtualTxQueue()
  {
    if (qdisc_owned_) {
      tcmsg traffic{};
      traffic.tcm_ifindex = static_cast<int>(::if_nametoindex(name_.c_str()));
      traffic.tcm_handle = 0x10000;
      traffic.tcm_parent = TC_H_ROOT;
      std::vector<uint8_t> payload(sizeof(traffic));
      std::memcpy(payload.data(), &traffic, sizeof(traffic));
      Request(RTM_DELQDISC, 0, payload);
    }
    if (length_changed_) {SetQueueLength(original_length_);}
  }
  bool ok() const {return ok_;}

private:
  static void Attribute(std::vector<uint8_t> & bytes, uint16_t type, const void * data, size_t size)
  {
    const size_t offset = bytes.size();
    bytes.resize(offset + RTA_SPACE(size), 0);
    auto * attribute = reinterpret_cast<rtattr *>(bytes.data() + offset);
    attribute->rta_type = type;
    attribute->rta_len = RTA_LENGTH(size);
    std::memcpy(RTA_DATA(attribute), data, size);
  }
  bool ReadQueueLength(unsigned int & length) const
  {
    std::ifstream file("/sys/class/net/" + name_ + "/tx_queue_len");
    return static_cast<bool>(file >> length);
  }
  bool SetQueueLength(unsigned int length)
  {
    ifinfomsg link{};
    link.ifi_index = static_cast<int>(::if_nametoindex(name_.c_str()));
    std::vector<uint8_t> payload(sizeof(link));
    std::memcpy(payload.data(), &link, sizeof(link));
    Attribute(payload, IFLA_TXQLEN, &length, sizeof(length));
    return Request(RTM_NEWLINK, 0, payload);
  }
  bool Request(uint16_t type, uint16_t flags, const std::vector<uint8_t> & payload)
  {
    if (!IsVirtualCan(name_)) {return false;}
    const int fd = ::socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (fd < 0) {return false;}
    timeval timeout{};
    timeout.tv_sec = 2;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    std::vector<uint8_t> bytes(NLMSG_LENGTH(payload.size()), 0);
    auto * header = reinterpret_cast<nlmsghdr *>(bytes.data());
    header->nlmsg_len = bytes.size();
    header->nlmsg_type = type;
    header->nlmsg_flags = flags | NLM_F_REQUEST | NLM_F_ACK;
    header->nlmsg_seq = 1;
    std::memcpy(NLMSG_DATA(header), payload.data(), payload.size());
    sockaddr_nl address{};
    address.nl_family = AF_NETLINK;
    const auto sent = ::sendto(fd, bytes.data(), bytes.size(), 0,
        reinterpret_cast<sockaddr *>(&address), sizeof(address));
    alignas(nlmsghdr) std::array<char, 1024> response{};
    const auto size = sent == static_cast<ssize_t>(bytes.size()) ?
      ::recv(fd, response.data(), response.size(), 0) : -1;
    ::close(fd);
    if (size < static_cast<ssize_t>(NLMSG_LENGTH(sizeof(nlmsgerr)))) {return false;}
    const auto * result = reinterpret_cast<const nlmsghdr *>(response.data());
    return result->nlmsg_type == NLMSG_ERROR &&
           reinterpret_cast<const nlmsgerr *>(NLMSG_DATA(result))->error == 0;
  }
  std::string name_;
  unsigned int original_length_ = 0;
  bool length_changed_ = false;
  bool qdisc_owned_ = false;
  bool ok_ = false;
};
}  // namespace robstride_test

#endif  // VCAN_TX_QUEUE_HPP_
