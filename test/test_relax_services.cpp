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
#include "vcan_tx_queue.hpp"

namespace
{
using Hardware = robstride_hardware_interface::RobstrideHardware;
using Callback = hardware_interface::CallbackReturn;
using Return = hardware_interface::return_type;
using Trigger = std_srvs::srv::Trigger;
using Values = std::array<double, 7>;
using namespace std::chrono_literals;

class RelaxServices : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!robstride_test::IsVirtualCan(robstride_test::TestInterface())) {
      if (std::getenv("ROBSTRIDE_REQUIRE_VCAN")) {FAIL() << "requires kernel vcan";}
      GTEST_SKIP() << "run test/with_vcan.py in an isolated network namespace";
    }
    char name[] = "/tmp/robstride-relax-service-XXXXXX";
    const int fd = ::mkstemp(name);
    ASSERT_GE(fd, 0);
    ::close(fd);
    yaml_path = name;
    std::ofstream yaml(yaml_path);
    yaml << "version: 1\nrelax_safe_poses:\n  - name: TEST_REST\n    joints:\n";
    for (int joint = 1; joint <= 7; ++joint) {
      yaml << "      joint" << joint << ": {position_rad: 0.0, tolerance_rad: " <<
        (joint == 7 ? "0.04" : "0.08") << "}\n";
    }
    yaml << "    stillness: {window_s: 0.15, max_span_rad: 0.005, max_abs_velocity_rad_s: 0.01}\n";
    yaml.close();
    follower = std::make_unique<robstride_test::FakeRobstrideFollower>(
      robstride_test::TestInterface());
    ASSERT_TRUE(follower->ok());
    observer = std::make_shared<rclcpp::Node>("relax_observer");
    relaxed_sub = observer->create_subscription<std_msgs::msg::Bool>(
      "/robstride_hardware_interface/relaxed", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {relaxed = message->data;});
  }
  void TearDown() override
  {
    hardware.reset();
    follower.reset();
    observer.reset();
    if (!yaml_path.empty()) {::unlink(yaml_path.c_str());}
  }
  hardware_interface::HardwareInfo Info(const std::string & mode = "motion")
  {
    hardware_interface::HardwareInfo info;
    info.name = "relax_fake";
    info.type = "system";
    info.hardware_plugin_name = "robstride_hardware_interface/RobstrideHardware";
    info.hardware_parameters = {{"error_timeout_ms", "350"}, {"master_id", "253"},
      {"motor_can_timeout_ms", "0"}, {"hold_torque_on_deactivate", "true"},
      {"thermal_cutoff_c", "100"}, {"thermal_clear_c", "75"},
      {"relax_safe_poses_file", yaml_path}};
    for (uint8_t id = 1; id <= 7; ++id) {
      hardware_interface::ComponentInfo joint;
      joint.name = "joint" + std::to_string(id);
      joint.type = "joint";
      joint.parameters["id"] = std::to_string(id);
      for (const char * name : {"position", "velocity", "effort"}) {
        hardware_interface::InterfaceInfo interface;
        interface.name = name;
        joint.command_interfaces.push_back(interface);
        joint.state_interfaces.push_back(interface);
      }
      joint.parameters["soft_lower_limit"] = "-1";
      joint.parameters["soft_upper_limit"] = "1";
      joint.parameters["soft_k_position"] = "10";
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
      info.gpios.push_back(gpio);
    }
    return info;
  }
  void Initialize(const std::string & mode = "motion", bool torque = true, bool configured = true)
  {
    auto info = Info(mode);
    if (!torque) {info.hardware_parameters["torque_enable"] = "false";}
    if (!configured) {info.hardware_parameters.erase("relax_safe_poses_file");}
    hardware = std::make_unique<Hardware>();
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = std::move(info);
    ASSERT_EQ(hardware->on_init(params), Callback::SUCCESS);
    command = hardware->export_command_interfaces();
    state = hardware->export_state_interfaces();
    ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
    Cycle(220);
  }
  void Pose(const Values & pose)
  {
    for (size_t i = 0; i < 7; ++i) {follower->motor(i).set_position(pose[i]);}
  }
  void Commands(const Values & pose)
  {
    for (size_t i = 0; i < 7; ++i) {ASSERT_TRUE(command.at(i * 3).set_value(pose[i]));}
  }
  void Tick()
  {
    ASSERT_EQ(hardware->read(hardware->now(), period), Return::OK);
    if (repeat_controller) {Commands(controller_pose);}
    ASSERT_EQ(hardware->write(hardware->now(), period), Return::OK);
    rclcpp::spin_some(observer);
    if (velocity_noise) {
      // Bench idle raw Type-2 velocity at rest, position static.
      static constexpr double kRaw[] = {.127, -.045, -.080, .017, -.007, -.026, -.016, .149};
      for (size_t i = 0; i < 7; ++i) {follower->motor(i).set_velocity(kRaw[(ticks + i) % 8]);}
      ++ticks;
    }
    if (silent_after_enable) {
      bool enabled = false;
      for (size_t i = 0; i < 7; ++i) {enabled |= follower->motor(i).enabled();}
      if (enabled) {follower->motor(6).set_silent(true);}
    }
    std::this_thread::sleep_for(10ms);
  }
  void Cycle(int ms = 100)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    do {Tick();} while (std::chrono::steady_clock::now() < end);
  }
  Trigger::Response::SharedPtr Call(const std::string & endpoint, bool updates = true)
  {
    auto client = observer->create_client<Trigger>("/robstride_hardware_interface/" + endpoint);
    if (!client->wait_for_service(2s)) {ADD_FAILURE() << "missing " << endpoint; return nullptr;}
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (future.wait_for(0s) != std::future_status::ready &&
      std::chrono::steady_clock::now() < end)
    {
      if (updates) {Tick();} else {rclcpp::spin_some(observer); std::this_thread::sleep_for(1ms);}
    }
    if (future.wait_for(0s) != std::future_status::ready) {
      ADD_FAILURE() << "timeout " << endpoint; return nullptr;
    }
    return future.get();
  }
  void Off() {for (size_t i = 0; i < 7; ++i) {EXPECT_FALSE(follower->motor(i).enabled()) << i;}}
  void On() {for (size_t i = 0; i < 7; ++i) {EXPECT_TRUE(follower->motor(i).enabled()) << i;}}
  size_t Enables(size_t i)
  {
    const auto frames = follower->motor(i).received();
    return std::count(frames.begin(), frames.end(), 3);
  }
  std::string yaml_path;
  std::unique_ptr<robstride_test::FakeRobstrideFollower> follower;
  std::unique_ptr<Hardware> hardware;
  rclcpp::Node::SharedPtr observer;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relaxed_sub;
  std::vector<hardware_interface::CommandInterface> command;
  std::vector<hardware_interface::StateInterface> state;
  bool relaxed = false;
  bool repeat_controller = false;
  bool silent_after_enable = false;
  bool velocity_noise = false;
  size_t ticks = 0;
  Values controller_pose{};
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(.01);
  const rclcpp_lifecycle::State lifecycle;
};

TEST_F(RelaxServices, ConfirmsSevenOffThenEngagesSettledMeasuredHoldWithoutStaleStep)
{
  Initialize();
  repeat_controller = true;
  Cycle(200);  // the controller's hold must itself be stable for a full window
  const auto released = Call("relax");
  ASSERT_TRUE(released && released->success) << (released ? released->message : "no reply");
  EXPECT_NE(released->message.find("TEST_REST"), std::string::npos);
  Off();
  Cycle(150);
  EXPECT_TRUE(relaxed);
  const Values settled{.003, .007, .004, .022, .001, .034, .006};
  Pose(settled);
  Cycle(200);
  const uint64_t start = robstride_test::SteadyNs();
  const auto engaged = Call("engage");
  ASSERT_TRUE(engaged && engaged->success) << (engaged ? engaged->message : "no reply");
  EXPECT_NE(engaged->message.find("elapsed_ms="), std::string::npos);
  const auto step_field = engaged->message.find("max_step_rad=");
  ASSERT_NE(step_field, std::string::npos);
  EXPECT_LT(std::stod(engaged->message.substr(step_field + 13)), .001);
  const auto latency_field = engaged->message.find("elapsed_ms=");
  ASSERT_NE(latency_field, std::string::npos);
  EXPECT_LT(std::stod(engaged->message.substr(latency_field + 11)), 500.0);
  On();
  Cycle(200);  // active producer continues rewriting the PRE-relax all-zero hold
  EXPECT_FALSE(relaxed);
  double step = 0;
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(follower->motor(i).position(), settled[i], .001) << i;
    EXPECT_NEAR(command[i * 3].get_optional().value(), 0, .001) << "controller deliberately stale";
    const auto samples = follower->motor(i).samples();
    auto preload = samples.end();
    for (auto sample = samples.begin(); sample != samples.end(); ++sample) {
      if (sample->stamp_ns < start) {continue;}
      if (sample->type == 1) {
        step = std::max(step, std::abs(sample->target - sample->before));
        EXPECT_NEAR(sample->effort, 0, .001);
        if (preload == samples.end()) {preload = sample;}
      }
      if (sample->type == 3) {
        ASSERT_NE(preload, samples.end()) << "measured MIT preload precedes Enable";
        EXPECT_LT(preload->stamp_ns, sample->stamp_ns);
        EXPECT_NEAR(preload->target, preload->before, .001);
      }
    }
  }
  EXPECT_LT(step, .001);
  std::cout << "vcan measured engage: " << engaged->message <<
    "; max_wire_step_rad=" << step << '\n';
  controller_pose[6] = .2;  // jaw-only motion must not release stale arm targets
  Cycle(200);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(follower->motor(i).position(), settled[i], .001);
  }
  EXPECT_NEAR(follower->motor(6).position(), .2, .002);
  controller_pose.fill(.4);  // NEW commands release only the corresponding joint holds
  Cycle(250);
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(follower->motor(i).position(), .4, .002);
  }
}

TEST_F(RelaxServices, TenFrameTxQueueReproducesBurstEnobufsThenRelaxEngageSucceeds)
{
  Initialize();
  // Linux vcan normally drains immediately even at qlen=10. Supply a bounded
  // kernel qdisc, not a mocked SendFrames result, to reproduce adapter pressure.
  robstride_test::LimitedVirtualTxQueue queue(robstride_test::TestInterface());
  ASSERT_TRUE(queue.ok()) << "requires CAP_NET_ADMIN and kernel netem";
  {
    robstride_sdk::CanBus probe(robstride_test::TestInterface());
    ASSERT_TRUE(probe.Open());
    robstride_sdk::RobstrideMotor motor(1, 253, robstride_sdk::ActuatorType::RS06);
    // Safe no-target parameter writes exercise the EXACT SDK sendmmsg path.
    std::vector<can_frame> burst(35, motor.EncodeParamWriteU8(
        robstride_sdk::ParamIndex::RUN_MODE, 0));
    const int queued = probe.SendFrames(burst);
    EXPECT_LT(queued, 35);
    EXPECT_EQ(probe.last_send_errno(), ENOBUFS);
  }
  std::this_thread::sleep_for(40ms);  // deliberate probe traffic must drain before driver cycles
  Cycle(250);
  const auto relaxed_response = Call("relax");
  ASSERT_TRUE(relaxed_response && relaxed_response->success) <<
    (relaxed_response ? relaxed_response->message : "no reply");
  Off();
  Pose({.003, .007, .004, .022, .001, .034, .006});
  Cycle(100);
  const auto engaged = Call("engage");
  ASSERT_TRUE(engaged && engaged->success) << (engaged ? engaged->message : "no reply");
  On();
  const Values settled{.003, .007, .004, .022, .001, .034, .006};
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(follower->motor(i).position(), settled[i], .001);
  }
  Cycle(200);  // reseeded command needs its own full stillness window
  const auto again = Call("relax");
  ASSERT_TRUE(again && again->success) << (again ? again->message : "no reply");
  Off();
  EXPECT_TRUE(Call("engage")->success);
  On();
  std::cout << "qlen=10 engage: " << engaged->message << '\n';
}

TEST_F(RelaxServices, NoisyRawVelocityAtRestStillRelaxesBeforeAndAfterEngage)
{
  Initialize();
  velocity_noise = true;  // real arm at HOME: positions static, raw velocity is not
  Cycle(250);
  const auto first = Call("relax");
  ASSERT_TRUE(first && first->success) << (first ? first->message : "no reply");
  Off();
  const Values settled{.003, -.002, .002, -.001, .024, .029, .009};
  Pose(settled);
  Cycle(200);
  const auto engaged = Call("engage");
  ASSERT_TRUE(engaged && engaged->success) << (engaged ? engaged->message : "no reply");
  On();
  Cycle(300);  // hold at HOME under the measured-pose hold, then relax directly
  const auto again = Call("relax");
  ASSERT_TRUE(again && again->success) << (again ? again->message : "no reply");
  Off();
}

TEST_F(RelaxServices, RelaxedCommandChangesWarnButNeverEnableOrMove)
{
  Initialize();
  ASSERT_TRUE(Call("relax")->success);
  Cycle(120);
  const auto enables = Enables(0);
  testing::internal::CaptureStderr();
  controller_pose.fill(.5);
  repeat_controller = true;
  Cycle(200);
  const auto warning = testing::internal::GetCapturedStderr();
  EXPECT_NE(warning.find("changed while relaxed"), std::string::npos) << warning;
  Off();
  EXPECT_EQ(Enables(0), enables);
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(follower->motor(i).position(), 0, .001);
  }
}

TEST_F(RelaxServices, RefusesOffPoseAndSeventhJointToleranceWithoutDisabling)
{
  for (size_t joint : {size_t{0}, size_t{6}}) {
    Values pose{};
    pose[joint] = joint == 0 ? .081 : .041;
    Pose(pose);
    Initialize();
    const auto response = Call("relax");
    ASSERT_TRUE(response);
    EXPECT_FALSE(response->success);
    EXPECT_NE(response->message.find("relax-safe pose"), std::string::npos) << response->message;
    On();
    ASSERT_EQ(hardware->on_deactivate(lifecycle), Callback::SUCCESS);
    hardware.reset();
  }
}

TEST_F(RelaxServices, RefusesMovingMeasuredPoseAndNonzeroVelocityCommand)
{
  Initialize();
  // Slow measured drift (0.02 rad/s): inside the 0.005 rad span over the
  // 0.15 s window, but over the position-slope bound. Raw velocity reads 0.
  follower->motor(6).set_follow(false);
  const auto start = std::chrono::steady_clock::now();
  const auto drift = [&] {
      const double seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      follower->motor(6).set_position(.02 * seconds);
    };
  for (int i = 0; i < 20; ++i) {
    drift();
    Tick();
  }
  auto client = observer->create_client<Trigger>("/robstride_hardware_interface/relax");
  ASSERT_TRUE(client->wait_for_service(2s));
  auto pending = client->async_send_request(std::make_shared<Trigger::Request>());
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (pending.wait_for(0s) != std::future_status::ready &&
    std::chrono::steady_clock::now() < deadline)
  {
    drift();
    Tick();
  }
  ASSERT_EQ(pending.wait_for(0s), std::future_status::ready);
  const auto moving = pending.get();
  ASSERT_TRUE(moving);
  EXPECT_FALSE(moving->success);
  EXPECT_NE(moving->message.find("not stationary"), std::string::npos) << moving->message;
  On();
  follower->motor(6).set_follow(true);
  ASSERT_TRUE(command[6 * 3 + 1].set_value(.01));
  Cycle(200);
  const auto commanded = Call("relax");
  ASSERT_TRUE(commanded);
  EXPECT_FALSE(commanded->success);
  EXPECT_NE(commanded->message.find("commands"), std::string::npos) << commanded->message;
  On();
}

TEST_F(RelaxServices, RecentlyChangingPositionCommandsRefuseWithStationaryFakeMotor)
{
  Initialize();
  follower->motor(6).set_follow(false);
  ASSERT_TRUE(command[6 * 3].set_value(.01));
  Cycle(50);
  const auto response = Call("relax");
  ASSERT_TRUE(response);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("commands"), std::string::npos) << response->message;
  On();
  Cycle(200);
  EXPECT_TRUE(Call("relax")->success);  // unchanged hold is the documented driver-level gate
}

TEST_F(RelaxServices, StaleCompleteFeedbackFaultAndThermalTripRefuse)
{
  Initialize();
  follower->motor(6).set_feedback_dlc(4);
  Cycle(150);  // last_update advances; only last_feedback must authorize relax
  const auto stale = Call("relax");
  ASSERT_TRUE(stale);
  EXPECT_FALSE(stale->success);
  EXPECT_NE(stale->message.find("unhealthy"), std::string::npos) << stale->message;
  follower->motor(6).set_feedback_dlc(8);
  Cycle(220);
  follower->motor(6).set_fault(true);
  Cycle(30);
  const auto fault = Call("relax");
  ASSERT_TRUE(fault);
  EXPECT_FALSE(fault->success);
  follower->motor(6).set_fault(false);
  follower->motor(5).set_temperature(100);
  Cycle(50);
  const auto thermal = Call("relax");
  ASSERT_TRUE(thermal);
  EXPECT_FALSE(thermal->success);
  EXPECT_NE(thermal->message.find("Thermal"), std::string::npos) << thermal->message;
}

TEST_F(RelaxServices, EngageOutOfBoundsRefusesBeforeAnyEnable)
{
  Initialize();
  ASSERT_TRUE(Call("relax")->success);
  Pose({0, 0, 0, 0, 0, 0, 2.3});
  Cycle(120);
  const auto enables = Enables(6);
  const auto response = Call("engage");
  ASSERT_TRUE(response);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("joint7"), std::string::npos) << response->message;
  Cycle(100);
  Off();
  EXPECT_EQ(Enables(6), enables);
}

TEST_F(RelaxServices, PartialEnableTimeoutDisablesAllWithoutAutomaticRetry)
{
  Initialize();
  ASSERT_TRUE(Call("relax")->success);
  follower->motor(6).set_drop_enable(true);
  const auto response = Call("engage");
  ASSERT_TRUE(response);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("timed out"), std::string::npos) << response->message;
  Cycle(80);
  Off();
  const auto enables = Enables(6);
  follower->motor(6).set_drop_enable(false);
  Cycle(250);
  Off();
  EXPECT_EQ(Enables(6), enables);
  EXPECT_TRUE(Call("engage")->success);
  On();
}

TEST_F(RelaxServices, ExpiredRequestCannotEnableWhenControlLoopResumes)
{
  Initialize();
  ASSERT_TRUE(Call("relax")->success);
  const auto enables = Enables(0);
  const auto response = Call("engage", false);
  ASSERT_TRUE(response);
  EXPECT_FALSE(response->success);
  Cycle(200);
  Off();
  EXPECT_EQ(Enables(0), enables);
}

TEST_F(RelaxServices, FreshHealthIsRequiredDuringEnableAndFailureRequestsAllOff)
{
  Initialize();
  ASSERT_TRUE(Call("relax")->success);
  follower->motor(6).set_drop_enable(true);  // one joint prevents fast confirmation
  silent_after_enable = true;
  const auto response = Call("engage");
  ASSERT_TRUE(response);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("unhealthy"), std::string::npos) << response->message;
  silent_after_enable = false;
  Cycle(80);
  Off();
  const auto enables = Enables(0);
  follower->motor(6).set_silent(false);
  follower->motor(6).set_drop_enable(false);
  Cycle(200);
  Off();
  EXPECT_EQ(Enables(0), enables);
  EXPECT_FALSE(relaxed) << "failed/unconfirmed transition is not confirmed safe rest";
}

TEST_F(RelaxServices, InactiveAndNewWindowRefuseWithoutChangingTorque)
{
  Initialize();
  ASSERT_EQ(hardware->on_deactivate(lifecycle), Callback::SUCCESS);
  const auto inactive = Call("relax");
  ASSERT_TRUE(inactive);
  EXPECT_FALSE(inactive->success);
  EXPECT_NE(inactive->message.find("inactive"), std::string::npos);
  ASSERT_EQ(hardware->on_activate(lifecycle), Callback::SUCCESS);
  const auto window = Call("relax");
  ASSERT_TRUE(window);
  EXPECT_FALSE(window->success);
  EXPECT_NE(window->message.find("window"), std::string::npos) << window->message;
  On();
}

TEST_F(RelaxServices, PoseCentresOutsideBoundsAndNonPositionModesRejectConfiguration)
{
  for (bool bad_mode : {false, true}) {
    auto info = Info(bad_mode ? "velocity" : "motion");
    if (!bad_mode) {info.limits["joint7"].min_position = .1;}
    hardware = std::make_unique<Hardware>();
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = std::move(info);
    EXPECT_EQ(hardware->on_init(params), Callback::ERROR);
    hardware.reset();
  }
}

TEST_F(RelaxServices, PositionModesPreloadMeasuredTargetBeforeEngageEnable)
{
  for (const std::string mode : {"position_pp", "position_csp"}) {
    Initialize(mode);
    ASSERT_TRUE(Call("relax")->success);
    Pose({.01, .02, .03, .02, .01, .03, .01});
    Cycle(100);
    const auto start = robstride_test::SteadyNs();
    ASSERT_TRUE(Call("engage")->success);
    for (size_t i = 0; i < 7; ++i) {
      const auto samples = follower->motor(i).samples();
      auto preload = samples.end();
      for (auto sample = samples.begin(); sample != samples.end(); ++sample) {
        if (sample->stamp_ns < start) {continue;}
        if (sample->type == 0x12 && sample->parameter == 0x7016) {preload = sample;}
        if (sample->type == 3) {
          ASSERT_NE(preload, samples.end());
          EXPECT_NEAR(preload->target, preload->before, .001);
          EXPECT_LT(preload->stamp_ns, sample->stamp_ns);
        }
      }
    }
    ASSERT_EQ(hardware->on_deactivate(lifecycle), Callback::SUCCESS);
    hardware.reset();
  }
}

TEST_F(RelaxServices, EngageIsIdempotentAndAlsoAnchorsReadableTorqueOffStartup)
{
  Initialize();
  EXPECT_TRUE(Call("engage")->success);
  On();
  ASSERT_EQ(hardware->on_deactivate(lifecycle), Callback::SUCCESS);
  hardware.reset();
  Initialize("motion", false);
  Off();
  Pose({.01, .02, .03, .02, .01, .03, .01});
  Cycle(100);
  ASSERT_TRUE(Call("engage")->success);
  On();
}

TEST_F(RelaxServices, MissingConfigRefusesRelaxAndFreshWindowRequired)
{
  Initialize("motion", true, false);
  const auto response = Call("relax");
  ASSERT_TRUE(response);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("No relax_safe_poses_file"), std::string::npos);
  On();
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
