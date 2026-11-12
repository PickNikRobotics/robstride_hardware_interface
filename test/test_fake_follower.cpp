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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robstride_hardware_interface/robstride_hardware_interface.hpp"
#include "fake_robstride_motor.hpp"

namespace
{
using Hardware = robstride_hardware_interface::RobstrideHardware;
using Callback = hardware_interface::CallbackReturn;
using Return = hardware_interface::return_type;
using SetData = robstride_interfaces::srv::SetDataToRobstride;
using GetData = robstride_interfaces::srv::GetDataFromRobstride;
using Values = std::array<double, 7>;
using namespace std::chrono_literals;

hardware_interface::HardwareInfo Info(const std::string & mode = "motion")
{
  hardware_interface::HardwareInfo info;
  info.name = "fake_follower";
  info.type = "system";
  info.hardware_plugin_name = "robstride_hardware_interface/RobstrideHardware";
  info.hardware_parameters = {{"error_timeout_ms", "350"}, {"master_id", "253"},
    {"motor_can_timeout_ms", "0"}, {"hold_torque_on_deactivate", "true"},
    {"thermal_cutoff_c", "100"}, {"thermal_clear_c", "75"}};
  for (uint8_t id = 1; id <= 7; ++id) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(id);
    joint.type = "joint";
    joint.parameters["id"] = std::to_string(id);
    hardware_interface::InterfaceInfo position;
    position.name = "position";
    joint.command_interfaces.push_back(position);
    for (const char * name : {"position", "velocity", "effort"}) {
      hardware_interface::InterfaceInfo state;
      state.name = name;
      joint.state_interfaces.push_back(state);
    }
    info.joints.push_back(joint);
    auto & limits = info.limits[joint.name];
    limits.has_position_limits = true;
    limits.min_position = -1;
    limits.max_position = 2;
    hardware_interface::ComponentInfo gpio;
    gpio.name = joint.name + "_motor";
    gpio.type = "gpio";
    gpio.parameters = {{"ID", std::to_string(id)}, {"actuator_type", id <= 3 ? "RS06" : "RS00"},
      {"can_interface", robstride_test::TestInterface()}, {"control_mode", mode},
      {"kp", "50"}, {"kd", "3"}};
    for (const char * name : {"temperature", "run_state", "fault_bits"}) {
      hardware_interface::InterfaceInfo state;
      state.name = name;
      gpio.state_interfaces.push_back(state);
    }
    info.gpios.push_back(gpio);
  }
  return info;
}

class FollowerTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!robstride_test::IsVirtualCan(robstride_test::TestInterface())) {
      if (std::getenv("ROBSTRIDE_REQUIRE_VCAN")) {
        FAIL() << "qualification requires a kernel vcan link";
      }
      GTEST_SKIP() << "run with test/with_vcan.py in an isolated network namespace";
    }
    follower =
      std::make_unique<robstride_test::FakeRobstrideFollower>(robstride_test::TestInterface());
    ASSERT_TRUE(follower->ok());
    observer = std::make_shared<rclcpp::Node>("follower_observer");
    state_sub = observer->create_subscription<robstride_interfaces::msg::RobstrideState>(
      "/robstride_hardware_interface/robstride_state", rclcpp::SystemDefaultsQoS(),
      [this](robstride_interfaces::msg::RobstrideState::ConstSharedPtr state) {
        latest = *state;
        ++state_messages;
      });
    thermal_sub = observer->create_subscription<std_msgs::msg::Bool>(
      "/robstride_hardware_interface/thermal_tripped", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr value) {tripped = value->data;});
  }
  void TearDown() override
  {
    hardware.reset();  // Hardware owner closes before fake sockets.
    follower.reset();
    observer.reset();
  }
  void Initialize(hardware_interface::HardwareInfo info = Info())
  {
    hardware = std::make_unique<Hardware>();
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = std::move(info);
    ASSERT_EQ(hardware->on_init(params), Callback::SUCCESS);
    command = hardware->export_command_interfaces();
    state = hardware->export_state_interfaces();
  }
  double Position(size_t index) const {return state.at(index * 6).get_optional().value();}
  void Command(size_t index, double value) {ASSERT_TRUE(command.at(index).set_value(value));}
  void Pose(const Values & pose)
  {for (size_t i = 0; i < 7; ++i) {follower->motor(i).set_position(pose[i]);}}
  void Cycle(int milliseconds = 100)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    do {
      EXPECT_EQ(hardware->read(hardware->now(), period), Return::OK);
      EXPECT_EQ(hardware->write(hardware->now(), period), Return::OK);
      rclcpp::spin_some(observer);
      std::this_thread::sleep_for(10ms);
    } while (std::chrono::steady_clock::now() < end);
  }
  template<class Service>
  typename Service::Response::SharedPtr Call(
    const std::string & endpoint,
    const typename Service::Request::SharedPtr & request)
  {
    auto client = observer->create_client<Service>("/robstride_hardware_interface/" + endpoint);
    if (!client->wait_for_service(2s)) {ADD_FAILURE() << "missing " << endpoint; return nullptr;}
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(observer, future,
        2s) != rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << "timeout " << endpoint; return nullptr;
    }
    return future.get();
  }
  bool Torque(bool enabled)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    const auto response = Call<std_srvs::srv::SetBool>("set_torque", request);
    return response && response->success;
  }
  size_t Count(size_t i, uint8_t type)
  {
    const auto frames = follower->motor(i).received();
    return std::count(frames.begin(), frames.end(), type);
  }
  std::unique_ptr<robstride_test::FakeRobstrideFollower> follower;
  std::unique_ptr<Hardware> hardware;
  rclcpp::Node::SharedPtr observer;
  rclcpp::Subscription<robstride_interfaces::msg::RobstrideState>::SharedPtr state_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr thermal_sub;
  robstride_interfaces::msg::RobstrideState latest;
  size_t state_messages = 0;
  bool tripped = false;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(.01);
  const rclcpp_lifecycle::State lifecycle;
  std::vector<hardware_interface::CommandInterface> command;
  std::vector<hardware_interface::StateInterface> state;
};

TEST_F(FollowerTest, ActivationSeedsAllSevenMeasuredPositionsWithoutZeroFrames)
{
  const Values pose{.2, .7, .9, -.2, .3, -.4, .5};
  Pose(pose);
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(Position(i), pose[i], .001);
    EXPECT_NEAR(command[i].get_optional().value(), pose[i], .001);
    EXPECT_NEAR(follower->motor(i).position(), pose[i], .001);
    EXPECT_EQ(Count(i, 6), 0u);
    EXPECT_EQ(follower->motor(i).watchdog_raw(), 0u);
  }
}

TEST_F(FollowerTest, OutOfBoundsActivationRefusesBeforeAnyEnable)
{
  Pose({.2, .7, .9, -.2, .3, -.4, 2.3});
  Initialize();
  EXPECT_EQ(hardware->on_activate(lifecycle), Callback::ERROR);
  std::this_thread::sleep_for(40ms);
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_EQ(Count(i, 3), 0u);
                                                             }
}

TEST_F(FollowerTest, TorqueOffActivationOutOfBoundsStaysLimpAndGatesSetTorque)
{
  Pose({.2, .7, .9, -.2, .3, -.4, 2.3});
  auto info = Info();
  info.hardware_parameters["torque_enable"] = "false";
  Initialize(info);
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  const auto refused = Call<std_srvs::srv::SetBool>("set_torque", request);
  ASSERT_TRUE(refused);
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("joint7"), std::string::npos) << refused->message;
  Cycle();
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_EQ(Count(i, 3), 0u);
    EXPECT_FALSE(follower->motor(i).enabled());
  }
  Pose({.2, .7, .9, -.2, .3, -.4, .5});
  Cycle();
  ASSERT_TRUE(Torque(true));
  Cycle(300);
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_GT(Count(i, 3), 0u);
    EXPECT_TRUE(follower->motor(i).enabled());
  }
}

TEST_F(FollowerTest, CommStateReportsStaleFeedbackAfterDeactivate)
{
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  EXPECT_EQ(latest.comm_state, 0);
  ASSERT_EQ(hardware->on_deactivate(lifecycle), Callback::SUCCESS);
  const size_t messages = state_messages;
  const auto end = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < end) {
    EXPECT_EQ(hardware->read(hardware->now(), period), Return::OK);
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_GT(state_messages, messages);
  EXPECT_EQ(latest.comm_state, -1) << "closed buses must not report a healthy link";
}

TEST_F(FollowerTest, FaultAndSilentLinkHaveSeparateTelemetry)
{
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  follower->motor(1).set_fault_word(1u << 14);
  Cycle(200);
  ASSERT_EQ(latest.fault_bits.size(), 7u);
  EXPECT_EQ(latest.fault_bits[1], 1u << 14);
  EXPECT_EQ(latest.comm_state, 0) << "healthy CAN must not be labelled link failure";
  EXPECT_GE(follower->motor(1).replies(), 10u);
  follower->motor(1).set_fault_word(0);
  follower->motor(5).set_silent(true);
  Cycle(500);
  EXPECT_EQ(latest.comm_state, -1) << "stale feedback is a communication failure";
  EXPECT_EQ(latest.fault_bits[5], 0u);
}

TEST_F(FollowerTest, LostJointFreezesPeersAndReactivationStopsEveryMotor)
{
  Pose({.2, .7, .9, -.2, .3, -.4, .5});
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  follower->motor(3).set_silent(true);
  Cycle(500);
  Values held{};
  for (size_t i = 0; i < 7; ++i) {
    held[i] = follower->motor(i).position(); Command(i, 1.5);
                                                                                           }
  Cycle(100);
  for (size_t i = 0; i < 7; ++i) {
    if (i != 3) {EXPECT_NEAR(follower->motor(i).position(), held[i], .001);}
  }
  follower->motor(3).set_silent(false);
  Cycle(150);
  EXPECT_FALSE(follower->motor(3).enabled());
  const size_t enables = Count(3, 3);
  Cycle(250);
  EXPECT_EQ(Count(3, 3), enables) << "new feedback must not clear the freeze";
  ASSERT_EQ(hardware->on_deactivate(lifecycle), Callback::SUCCESS);
  // Software does not enforce HOME. Test an explicitly supplied HOME pose; the
  // no-brakes support requirement belongs to the robot's recovery runbook.
  Pose({0, 0, 0, 0, 0, 0, 0});
  std::array<size_t, 7> stops{};
  for (size_t i = 0; i < 7; ++i) {
    stops[i] = Count(i, 4);
                                                         }
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_GT(Count(i, 4), stops[i]) << "CURRENT recovery releases torque on all motors";
    EXPECT_NEAR(follower->motor(i).position(), 0, .001);
    EXPECT_TRUE(follower->motor(i).enabled());
  }
}

TEST_F(FollowerTest, FailedRecoveryWithSilentJointDisablesAll)
{
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  follower->motor(6).set_silent(true);
  Cycle(500);
  ASSERT_EQ(hardware->on_deactivate(lifecycle), Callback::SUCCESS);
  EXPECT_EQ(hardware->on_activate(lifecycle), Callback::ERROR);
  std::this_thread::sleep_for(40ms);
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_FALSE(follower->motor(i).enabled());
                                                                             }
}

TEST_F(FollowerTest, FaultBit14StaticHoldIsNotTheTemperatureLatch)
{
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  follower->motor(2).set_effort(10);
  follower->motor(2).set_fault_word(1u << 14);  // Injected overload, not thermodynamics.
  Cycle(300);
  EXPECT_EQ(latest.fault_bits[2], 1u << 14);
  EXPECT_FALSE(tripped);
  EXPECT_FALSE(follower->motor(2).enabled());
}

TEST_F(FollowerTest, Temperature100LatchesWhile99Point9DoesNot)
{
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  follower->motor(6).set_temperature(99.9);
  Cycle(250);
  EXPECT_FALSE(tripped);
  follower->motor(6).set_temperature(100);
  Cycle(250);
  EXPECT_TRUE(tripped);
  EXPECT_TRUE(follower->motor(6).enabled());
  EXPECT_FALSE(Torque(true));
  follower->motor(6).set_temperature(25);
  Cycle(250);
  EXPECT_TRUE(tripped);
}

TEST_F(FollowerTest, GainAckIsQueuedNotMotorReadbackEvenWhenMotionFramesDrop)
{
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  for (const auto & gain : {std::string("kp"), std::string("kd")}) {
    auto write = std::make_shared<SetData::Request>();
    write->id = 1; write->item_name = gain; write->item_data = gain == "kp" ? 23.f : 1.4f;
    const auto ack = Call<SetData>("set_data_to_robstride", write);
    ASSERT_TRUE(ack && ack->result);
    Cycle();
    const auto samples = follower->motor(0).samples();
    ASSERT_FALSE(samples.empty());
    EXPECT_NEAR(gain == "kp" ? samples.back().kp : samples.back().kd, write->item_data, .08);
    auto read = std::make_shared<GetData::Request>();
    read->id = 1; read->item_name = gain;
    const auto response = Call<GetData>("get_data_from_robstride", read);
    ASSERT_TRUE(response);
    EXPECT_FALSE(response->result) <<
      "gain is explicitly unavailable/unconfirmed by public readback";
  }
  const double applied = follower->motor(0).applied_kp();
  follower->motor(0).set_drop_motion(true);
  auto write = std::make_shared<SetData::Request>();
  write->id = 1; write->item_name = "kp"; write->item_data = 40;
  const auto ack = Call<SetData>("set_data_to_robstride", write);
  ASSERT_TRUE(ack && ack->result) << "CURRENT ack cannot detect discarded Type-1 commands";
  Cycle();
  EXPECT_DOUBLE_EQ(follower->motor(0).applied_kp(), applied);
  const auto samples = follower->motor(0).samples();
  EXPECT_NEAR(samples.back().kp, 40, .08) << "wire send observed, but follower did not apply it";
}

TEST_F(FollowerTest, TorqueOffStaysReadableAndShutdownSendsNoEnable)
{
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  ASSERT_TRUE(Torque(false));
  Cycle();
  const size_t messages = state_messages;
  Pose({.3, .4, .5, .1, .2, -.1, .6});
  Cycle();
  EXPECT_GT(state_messages, messages);
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_FALSE(follower->motor(i).enabled());
                                                                             }
  const size_t enables = Count(0, 3);
  hardware.reset();
  std::this_thread::sleep_for(40ms);
  EXPECT_EQ(Count(0, 3), enables);
}

TEST_F(FollowerTest, PositionModePreloadsMeasuredTargetBeforeEnable)
{
  for (const std::string mode : {"position_pp", "position_csp"}) {
    SCOPED_TRACE(mode);
    Pose({.2, .7, .9, -.2, .3, -.4, .5});
    Initialize(Info(mode));
    ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
    Cycle();
    for (size_t i = 0; i < 7; ++i) {
      const auto samples = follower->motor(i).samples();
      const auto enable = std::find_if(samples.rbegin(), samples.rend(),
          [](const auto & sample) {return sample.type == 3;});
      ASSERT_NE(enable, samples.rend());
      const auto preload = std::find_if(enable, samples.rend(),
          [](const auto & sample) {return sample.type == 0x12 && sample.parameter == 0x7016;});
      ASSERT_NE(preload, samples.rend());
      EXPECT_NEAR(preload->target, preload->position, .001);
      EXPECT_EQ(follower->motor(i).mode(), mode == "position_pp" ? 1 : 5);
      Command(i, .8);
    }
    Cycle();
    for (size_t i = 0; i < 7; ++i) {
      EXPECT_NEAR(follower->motor(i).position(), .8, .001);
    }
    ASSERT_EQ(hardware->on_deactivate(lifecycle), Callback::SUCCESS);
    hardware.reset();
  }
}

TEST_F(FollowerTest, WatchdogReleasesAfterHeldShutdownWhenArmed)
{
  auto info = Info();
  info.hardware_parameters["motor_can_timeout_ms"] = "500";
  Initialize(info);
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  EXPECT_EQ(follower->motor(0).watchdog_raw(), 10000u);
  for (size_t i = 0; i < 7; ++i) {
    ASSERT_TRUE(follower->motor(i).enabled());
                                                                            }
  ASSERT_EQ(hardware->on_deactivate(lifecycle), Callback::SUCCESS);
  std::this_thread::sleep_for(600ms);
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_FALSE(follower->motor(i).enabled());
                                                                             }
}

TEST_F(FollowerTest, RebootedMotorStaysOffUntilFreshActivation)
{
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  const size_t enables = Count(2, 3);
  follower->motor(2).reboot();
  Cycle(500);
  EXPECT_EQ(Count(2, 3), enables);
  EXPECT_FALSE(follower->motor(2).enabled());
  EXPECT_EQ(latest.comm_state, 0) << "reboot with fresh frames is not link loss";
}

TEST_F(FollowerTest, TruncatedFeedbackCannotActivateWithoutMeasuredPosition)
{
  follower->motor(6).set_feedback_dlc(4);
  Initialize();
  EXPECT_EQ(hardware->on_activate(lifecycle), Callback::ERROR);
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_EQ(Count(i, 3), 0u);
                                                             }
}

TEST_F(FollowerTest, DriverTransmitFailureLogsAndBecomesLinkLossWithoutTearingDownPeers)
{
  Initialize();
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  const std::string interface = robstride_test::TestInterface();
  ASSERT_TRUE(robstride_test::SetVirtualCanUp(interface, false));
  struct Restore {std::string name;
    ~Restore()
    {
      robstride_test::SetVirtualCanUp(name, true);
    }
     } restore{interface};
  testing::internal::CaptureStderr();
  const auto start = std::chrono::steady_clock::now();
  const auto result = hardware->write(hardware->now(), period);
  const auto diagnostic = testing::internal::GetCapturedStderr();
  EXPECT_EQ(result, Return::OK) << "CURRENT write logs but does not return ERROR";
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
  EXPECT_NE(diagnostic.find("CAN transmit incomplete"), std::string::npos);
  Cycle(500);
  EXPECT_EQ(latest.comm_state, -1);
  ASSERT_TRUE(robstride_test::SetVirtualCanUp(interface, true));
  Cycle(100);
}

TEST_F(FollowerTest, TransmitFailureIsCapturedBySdkWithoutBlocking)
{
  robstride_sdk::CanBus bus(robstride_test::TestInterface());
  ASSERT_TRUE(bus.Open());
  robstride_sdk::RobstrideMotor motor(1, 253, robstride_sdk::ActuatorType::RS06);
  EXPECT_TRUE(bus.SendFrame(motor.EncodeEnable()));
  bus.Close();
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(bus.SendFrame(motor.EncodeEnable()));
  EXPECT_EQ(bus.last_send_errno(), EBADF);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
}

// Jaw limits measured on the bench: closed contact to the open stop (5.84),
// centred so the one-turn window is [-0.277, 6.007).
hardware_interface::HardwareInfo JawInfo(
  double lower = -0.15, double upper = 5.88, bool soft = false, double soft_upper = 5.41)
{
  auto info = Info();
  info.limits["joint7"].min_position = lower;
  info.limits["joint7"].max_position = upper;
  if (soft) {
    info.joints[6].parameters["soft_lower_limit"] = std::to_string(lower);
    info.joints[6].parameters["soft_upper_limit"] = std::to_string(soft_upper);
    info.joints[6].parameters["soft_k_position"] = "10";
  }
  return info;
}

void Step(Hardware & hardware, const rclcpp::Duration & period, int cycles = 1)
{
  for (int k = 0; k < cycles; ++k) {
    EXPECT_EQ(hardware.read(hardware.now(), period), Return::OK);
    EXPECT_EQ(hardware.write(hardware.now(), period), Return::OK);
    std::this_thread::sleep_for(10ms);
  }
}

TEST_F(FollowerTest, FoldOnceJawCrossesTheWindowEdgeWithoutAOneTurnJump)
{
  Pose({.2, .7, .9, -.2, .3, -.4, 5.0});
  Initialize(JawInfo());
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  ASSERT_NEAR(Position(6), 5.0, .001);
  // Drive the jaw past the window edge (6.007) by hand, as the 15 rad/s
  // overshoot did past the old 5.772 edge; the reading must keep counting.
  follower->motor(6).set_follow(false);
  for (double p : {5.6, 5.9, 6.0, 6.05, 6.1}) {
    follower->motor(6).set_position(p);
    Step(*hardware, period, 3);
    EXPECT_NEAR(Position(6), p, .002) << "reading folded at " << p;
  }
  // A command is sent in the same turn, not one turn further open.
  follower->motor(6).set_follow(true);
  Command(6, .5);
  Cycle();
  EXPECT_NEAR(follower->motor(6).position(), .5, .002);
  EXPECT_NEAR(Position(6), .5, .002);
}

TEST_F(FollowerTest, PerReadFoldIsGoneEvenWithTodaysNarrowWindow)
{
  // Today's limits -0.15..5.41: window edge 5.772, the one crossed on the bench.
  Pose({.2, .7, .9, -.2, .3, -.4, 5.3});
  Initialize(JawInfo(-0.15, 5.41));
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  follower->motor(6).set_follow(false);
  for (double p : {5.6, 5.724, 5.78, 5.84}) {
    follower->motor(6).set_position(p);
    Step(*hardware, period, 3);
    EXPECT_NEAR(Position(6), p, .002) << "reading folded at " << p;
  }
  follower->motor(6).set_follow(true);
  Command(6, .021);
  Cycle();
  EXPECT_NEAR(follower->motor(6).position(), .021, .002) << "target landed a turn away";
}

TEST_F(FollowerTest, ActivationSeedsTheJawClosedPartlyOpenAndAtTheStop)
{
  constexpr double kTwoPi = 2 * M_PI;
  // {raw motor angle at power-up, expected joint reading}: zero_sta 0 reports
  // 0..2pi, zero_sta 1 reports -pi..pi; both must land on the physical angle.
  const std::vector<std::pair<double, double>> cases{
    {.157, .157}, {1.0, 1.0}, {3.0, 3.0}, {3.0 - kTwoPi, 3.0}, {5.84, 5.84},
    {5.84 - kTwoPi, 5.84}, {kTwoPi - .0105, -.0105}, {kTwoPi - .12, -.12}};
  for (const auto & [raw, expected] : cases) {
    Pose({.2, .7, .9, -.2, .3, -.4, raw});
    Initialize(JawInfo());
    ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS) << raw;
    Cycle(50);
    EXPECT_NEAR(Position(6), expected, .002) << "raw " << raw;
    EXPECT_NEAR(command[6].get_optional().value(), expected, .002) << "raw " << raw;
    // Holding the seeded command must not move the motor.
    EXPECT_NEAR(follower->motor(6).position(), raw, .002) << "raw " << raw;
    ASSERT_EQ(hardware->on_deactivate(lifecycle), Callback::SUCCESS);
    hardware.reset();
  }
}

TEST_F(FollowerTest, TodaysNarrowWindowRefusesTheJawAtTheOpenStop)
{
  // Documents why the jaw limits move: 5.84 folds to -0.443 in -0.15..5.41.
  Pose({.2, .7, .9, -.2, .3, -.4, 5.84});
  Initialize(JawInfo(-0.15, 5.41));
  EXPECT_EQ(hardware->on_activate(lifecycle), Callback::ERROR);
}

TEST_F(FollowerTest, ReframedFeedbackFoldsAgainInsteadOfJumping)
{
  Pose({.2, .7, .9, -.2, .3, -.4, -.0105});
  Initialize(JawInfo());
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle();
  // A power-cycled motor (zero_sta 0) reports the same closed jaw as 2*pi - 0.0105.
  follower->motor(6).set_follow(false);
  follower->motor(6).set_position(2 * M_PI - .0105);
  Step(*hardware, period, 3);
  EXPECT_NEAR(Position(6), -.0105, .002);
}

TEST_F(FollowerTest, SoftLimitSlowsAndStopsAPastLimitCommand)
{
  Pose({.2, .7, .9, -.2, .3, -.4, 1.0});
  Initialize(JawInfo(-0.15, 5.88, true, 5.41));
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle(50);
  Command(6, 5.8);  // past the 5.41 soft limit, inside the 5.88 hard limit
  std::vector<double> sent;
  for (int k = 0; k < 150; ++k) {
    Step(*hardware, period);
    sent.push_back(follower->motor(6).position());
  }
  double peak = 0, last_step = 1e9;
  for (size_t k = 1; k < sent.size(); ++k) {
    EXPECT_GE(sent[k], sent[k - 1] - 1e-3) << k;
    peak = std::max(peak, sent[k]);
  }
  EXPECT_LE(peak, 5.41 + .001);
  EXPECT_NEAR(sent.back(), 5.41, .01);
  // It slowed: the late steps are far smaller than the first.
  last_step = sent.back() - sent[sent.size() - 2];
  EXPECT_GT(sent[1] - sent[0], 10 * std::max(last_step, 1e-6));
  // Coming back in is not held back.
  Command(6, 1.0);
  Step(*hardware, period, 30);
  EXPECT_NEAR(follower->motor(6).position(), 1.0, .01);
}

TEST_F(FollowerTest, WithoutSoftLimitsAPastLimitCommandPassesStraightThrough)
{
  // Documents today's behaviour: the driver itself never clamps a command.
  Pose({.2, .7, .9, -.2, .3, -.4, 1.0});
  Initialize(JawInfo());
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  Cycle(50);
  Command(6, 5.8);
  Cycle(50);
  EXPECT_NEAR(follower->motor(6).position(), 5.8, .002);
}

TEST_F(FollowerTest, MalformedSoftLimitsAreRejectedAtInit)
{
  auto info = JawInfo(-0.15, 5.88, true, 5.41);
  info.joints[6].parameters["soft_k_position"] = "0";
  hardware = std::make_unique<Hardware>();
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = std::move(info);
  EXPECT_EQ(hardware->on_init(params), Callback::ERROR);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
