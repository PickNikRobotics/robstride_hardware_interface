// Copyright 2026 Minjo Kim
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
// Modified by PickNik Inc., 2026: torque-retention lifecycle, thermal handoff,
// bounded motion-mode effort feedforward, activation bounds and link-only
// comm_state. See NOTICE for source history.

#include "robstride_hardware_interface/robstride_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <iomanip>
#include <sstream>
#include <set>
#include <stdexcept>
#include <thread>

#include "robstride_hardware_interface/joint_coordinates.hpp"

namespace robstride_hardware_interface
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

RobstrideHardware::RobstrideHardware()
: rclcpp::Node("robstride_hardware_interface"),
  logger_(rclcpp::get_logger("robstride_hardware_interface"))
{
}

RobstrideHardware::~RobstrideHardware()
{
  if (service_executor_thread_.joinable()) {
    // A cancel() that lands before spin() starts is lost, so repeat it until
    // spin() returns. Destroying the component right after on_init() hung here.
    while (!service_executor_stopped_.load()) {
      service_executor_->cancel();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    service_executor_thread_.join();
  }
  for (auto & kv : buses_) {
    kv.second->Stop();
    kv.second->Close();
  }
}

namespace
{
std::string GetJointParam(
  const hardware_interface::ComponentInfo & joint, const std::string & key,
  const std::string & default_value = "")
{
  const auto it = joint.parameters.find(key);
  return it == joint.parameters.end() ? default_value : it->second;
}

bool HasInterface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & info) {return info.name == name;});
}

// The two timestamps come from different threads, so last_ns can land
// slightly after now_ns. Clamp to 0 instead of wrapping around.
double ElapsedMsSinceUpdate(uint64_t now_ns, uint64_t last_ns)
{
  if (last_ns == 0) {return -1.0;}
  if (last_ns >= now_ns) {return 0.0;}
  return static_cast<double>(now_ns - last_ns) / 1.0e6;
}

// How often write() re-sends the re-enable burst to an untorqued joint.
constexpr uint64_t kRecoveryResendNs = 200'000'000ull;

// CAN_TIMEOUT (0x7028) in its native 1/20000 s units, clamped to range.
// Zero (or negative) disables the motor's watchdog entirely.
uint32_t CanTimeoutRaw(double timeout_ms)
{
  if (timeout_ms <= 0.0) {
    return 0u;
  }
  return std::max(1u, static_cast<uint32_t>(std::min(timeout_ms * 20.0, 100000.0)));
}

constexpr double kTwoPi = 2.0 * M_PI;

bool IsEnabled(const robstride_sdk::MotorState & st)
{
  return st.run_state.load(std::memory_order_relaxed) ==
         static_cast<uint8_t>(robstride_sdk::RunState::MOTOR);
}
}  // namespace

CallbackReturn RobstrideHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    const auto cutoff = info_.hardware_parameters.find("thermal_cutoff_c");
    const auto clear = info_.hardware_parameters.find("thermal_clear_c");
    const double cutoff_c =
      cutoff == info_.hardware_parameters.end() ? 0.0 : std::stod(cutoff->second);
    const double clear_c =
      clear == info_.hardware_parameters.end() ? 75.0 : std::stod(clear->second);
    if (std::isfinite(cutoff_c) && std::isfinite(clear_c) &&
      cutoff_c > 0.0 && clear_c >= cutoff_c)
    {
      RCLCPP_ERROR_STREAM(
        logger_,
        "thermal_clear_c (75 by default) must be below thermal_cutoff_c (" << cutoff_c <<
          "); set thermal_clear_c in the hardware parameters to a lower value.");
      return CallbackReturn::ERROR;
    }
    thermal_latch_.Configure(cutoff_c, clear_c);
    if (info_.hardware_parameters.count("master_id")) {
      master_id_ = static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("master_id")));
    }
    if (info_.hardware_parameters.count("error_timeout_ms")) {
      error_timeout_ms_ = std::stod(info_.hardware_parameters.at("error_timeout_ms"));
    }
    // Defaults to error_timeout_ms; 0 lets a motor keep holding its last
    // target when the bus goes silent instead of releasing torque.
    motor_can_timeout_ms_ = error_timeout_ms_;
    if (info_.hardware_parameters.count("motor_can_timeout_ms")) {
      motor_can_timeout_ms_ = std::stod(info_.hardware_parameters.at("motor_can_timeout_ms"));
    }
    if (info_.hardware_parameters.count("freeze_on_joint_loss")) {
      const std::string & v = info_.hardware_parameters.at("freeze_on_joint_loss");
      freeze_on_joint_loss_ = v == "true" || v == "True" || v == "1";
    }
    if (info_.hardware_parameters.count("torque_enable")) {
      const std::string & v = info_.hardware_parameters.at("torque_enable");
      torque_enabled_desired_.store(v == "true" || v == "True" || v == "1");
    }
    if (info_.hardware_parameters.count("hold_torque_on_deactivate")) {
      const std::string & v = info_.hardware_parameters.at("hold_torque_on_deactivate");
      hold_torque_on_deactivate_ = v == "true" || v == "True" || v == "1";
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Malformed hardware parameter "
      "(master_id/error_timeout_ms/motor_can_timeout_ms/thermal_cutoff_c/thermal_clear_c): " <<
        e.what());
    return CallbackReturn::ERROR;
  }
  if (motor_can_timeout_ms_ <= 0.0) {
    RCLCPP_WARN(
      logger_,
      "Motor CAN watchdog disabled (motor_can_timeout_ms <= 0): if the bus dies, motors keep "
      "holding their last target until a clean shutdown or a power cut.");
  }
  // The two settings are coupled: deactivation closes the bus, so a motor
  // whose own watchdog is still armed releases torque a short time after the
  // frames stop regardless of what on_deactivate() chose not to send.
  if (hold_torque_on_deactivate_ && motor_can_timeout_ms_ > 0.0) {
    RCLCPP_WARN_STREAM(
      logger_,
      "hold_torque_on_deactivate is set but the motor CAN watchdog is armed at " <<
        motor_can_timeout_ms_ << " ms, so each motor will release torque that long after "
        "deactivation and the arm will drop. Set motor_can_timeout_ms to 0 to hold the pose "
        "after a stop.");
  }

  const size_t n = info_.joints.size();

  // Optional cross-check only: catches a description that lost or gained
  // a joint during editing.
  if (info_.hardware_parameters.count("number_of_joints")) {
    try {
      const size_t declared =
        static_cast<size_t>(std::stoul(info_.hardware_parameters.at("number_of_joints")));
      if (declared != n) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "Hardware declares number_of_joints=" << declared << " but the description has " <<
            n << " <joint> block(s)");
        return CallbackReturn::ERROR;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Malformed hardware parameter number_of_joints: " << e.what());
      return CallbackReturn::ERROR;
    }
  }

  joints_.resize(n);
  hw_state_position_.assign(n, 0.0);
  hw_state_velocity_.assign(n, 0.0);
  hw_state_effort_.assign(n, 0.0);
  hw_cmd_position_.assign(n, 0.0);
  hw_cmd_velocity_.assign(n, 0.0);
  freeze_position_.assign(n, 0.0);
  wrap_offset_.assign(n, 0.0);
  wrap_seeded_.assign(n, false);
  last_motor_position_.assign(n, 0.0);
  soft_prev_cmd_.assign(n, 0.0);
  seen_enabled_.assign(n, false);
  last_recovery_ns_.assign(n, 0);
  recovery_step_.assign(n, RecoveryStep::Waiting);
  measured_in_bounds_ = std::vector<std::atomic<bool>>(n);
  for (auto & bounds : measured_in_bounds_) {
    bounds.store(false, std::memory_order_relaxed);
  }
  hw_cmd_effort_.assign(n, 0.0);
  hw_cmd_kp_.assign(n, 0.0);
  hw_cmd_kd_.assign(n, 0.0);
  configured_kp_.assign(n, 0.0);
  configured_kd_.assign(n, 0.0);
  hw_state_temperature_.assign(n, 0.0);
  hw_state_run_state_.assign(n, 0.0);
  hw_state_fault_bits_.assign(n, 0.0);
  pending_kp_ = std::vector<std::atomic<double>>(n);
  pending_kd_ = std::vector<std::atomic<double>>(n);
  has_pending_kp_ = std::vector<std::atomic<bool>>(n);
  has_pending_kd_ = std::vector<std::atomic<bool>>(n);
  for (size_t i = 0; i < n; ++i) {
    has_pending_kp_[i].store(false);
    has_pending_kd_[i].store(false);
  }

  motors_.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];

    const std::string id_str = GetJointParam(joint, "id");
    if (id_str.empty()) {
      RCLCPP_ERROR_STREAM(logger_, "Joint '" << joint.name << "' is missing its 'id' param");
      return CallbackReturn::ERROR;
    }

    JointHandle & jh = joints_[i];
    jh.joint_name = joint.name;
    try {
      jh.id = static_cast<uint8_t>(std::stoi(id_str));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Joint '" << joint.name << "': " << e.what());
      return CallbackReturn::ERROR;
    }

    // Device configuration lives in the <gpio> block, matched by id.
    const hardware_interface::ComponentInfo * gpio = nullptr;
    for (const auto & candidate : info_.gpios) {
      const std::string gpio_id = GetJointParam(candidate, "ID");
      if (!gpio_id.empty() && gpio_id == id_str) {
        gpio = &candidate;
        break;
      }
    }
    if (gpio == nullptr) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Joint '" << joint.name << "' (id=" << id_str <<
          ") has no matching <gpio> block. Every joint needs one declaring "
          "ID / actuator_type / can_interface / control_mode.");
      return CallbackReturn::ERROR;
    }
    jh.gpio_name = gpio->name;

    const std::string type_str = GetJointParam(*gpio, "actuator_type");
    const std::string can_if = GetJointParam(*gpio, "can_interface");
    if (type_str.empty() || can_if.empty()) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "gpio '" << gpio->name << "' is missing a required param "
          "(actuator_type / can_interface)");
      return CallbackReturn::ERROR;
    }

    try {
      jh.actuator_type = robstride_sdk::ActuatorTypeFromString(type_str);
      jh.control_mode = robstride_sdk::RunModeFromString(GetJointParam(*gpio, "control_mode", ""));
      hw_cmd_kp_[i] = std::stod(GetJointParam(*gpio, "kp", "0.0"));
      hw_cmd_kd_[i] = std::stod(GetJointParam(*gpio, "kd", "0.0"));
      configured_kp_[i] = hw_cmd_kp_[i];
      configured_kd_[i] = hw_cmd_kd_[i];
      jh.direction = std::stod(GetJointParam(*gpio, "direction", "1"));
      if (jh.direction != 1.0 && jh.direction != -1.0) {
        throw std::invalid_argument("direction must be 1 or -1");
      }
      jh.max_feedforward_effort = std::stod(
        GetJointParam(
          *gpio, "max_feedforward_effort",
          std::to_string(DefaultMaxFeedforwardEffort(jh.actuator_type))));
      if (!(jh.max_feedforward_effort >= 0.0)) {
        throw std::invalid_argument("max_feedforward_effort must be >= 0");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "gpio '" << gpio->name << "': " << e.what());
      return CallbackReturn::ERROR;
    }
    jh.can_interface = can_if;

    // Optional soft limits live on the <joint> (not the gpio), like "id".
    const std::string soft_lo = GetJointParam(joint, "soft_lower_limit", "");
    const std::string soft_hi = GetJointParam(joint, "soft_upper_limit", "");
    const std::string soft_k = GetJointParam(joint, "soft_k_position", "");
    if (!soft_lo.empty() || !soft_hi.empty() || !soft_k.empty()) {
      try {
        jh.soft_lower = std::stod(soft_lo);
        jh.soft_upper = std::stod(soft_hi);
        jh.soft_k_position = std::stod(soft_k);
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(logger_, "Joint '" << joint.name << "': soft_lower_limit, "
          "soft_upper_limit and soft_k_position must all be numbers: " << e.what());
        return CallbackReturn::ERROR;
      }
      if (!(std::isfinite(jh.soft_lower) && std::isfinite(jh.soft_upper) &&
        jh.soft_lower < jh.soft_upper && jh.soft_k_position > 0.0 &&
        std::isfinite(jh.soft_k_position)))
      {
        RCLCPP_ERROR_STREAM(logger_, "Joint '" << joint.name << "': soft limits need "
          "soft_lower_limit < soft_upper_limit and a finite soft_k_position > 0");
        return CallbackReturn::ERROR;
      }
      jh.has_soft_limits = true;
    }

    // A single-turn reading can only be placed unambiguously inside a
    // window narrower than one turn, so center it on the joint's range.
    const auto lim = info_.limits.find(joint.name);
    if (lim != info_.limits.end() && lim->second.has_position_limits) {
      const double lower = lim->second.min_position;
      const double upper = lim->second.max_position;
      jh.wrap_center = 0.5 * (lower + upper);
      if (upper - lower >= kTwoPi) {
        RCLCPP_WARN_STREAM(
          logger_,
          "Joint '" << joint.name << "' spans " << (upper - lower) << " rad, a full turn or "
            "more: its position is reported modulo 2*pi around " << jh.wrap_center <<
            ", so the ends of its range alias onto each other.");
      }
    } else {
      jh.continuous = true;
    }

    // The declared command interface must be the one this mode writes, or
    // a controller would claim it and the joint would never move.
    const char * required_command = nullptr;
    switch (jh.control_mode) {
      case robstride_sdk::RunMode::RUN_MODE_VELOCITY:
        required_command = hardware_interface::HW_IF_VELOCITY;
        break;
      case robstride_sdk::RunMode::RUN_MODE_CURRENT:
        required_command = hardware_interface::HW_IF_EFFORT;
        break;
      default:
        required_command = hardware_interface::HW_IF_POSITION;
        break;
    }
    if (!HasInterface(joint.command_interfaces, required_command)) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Joint '" << joint.name << "' runs in control_mode '" <<
          GetJointParam(*gpio, "control_mode", "motion") << "', which is driven through the '" <<
          required_command << "' command interface, but the joint does not declare it.");
      return CallbackReturn::ERROR;
    }

    motors_.push_back(
      std::make_unique<robstride_sdk::RobstrideMotor>(jh.id, master_id_, jh.actuator_type));
    jh.motor = motors_.back().get();

    if (buses_.find(can_if) == buses_.end()) {
      buses_[can_if] = std::make_unique<robstride_sdk::CanBus>(can_if);
      can_tx_retry_.emplace(can_if, CanTxRetry{});
    }
    buses_[can_if]->RegisterMotor(jh.motor);
  }

  try {
    const auto file = info_.hardware_parameters.find("relax_safe_poses_file");
    if (file != info_.hardware_parameters.end()) {
      std::vector<std::string> names;
      for (const auto & joint : joints_) {
        names.push_back(joint.joint_name);
      }
      relax_safe_poses_ = LoadRelaxSafePoses(file->second, names);
      // A named pose's centre must be physically admissible. Its tolerance
      // may cross a URDF limit (HOME can sit on one); runtime bounds always
      // intersect the pose check, using the existing feedback-quantum margin.
      for (const auto & pose : relax_safe_poses_) {
        for (size_t i = 0; i < n; ++i) {
          const auto limit = info_.limits.find(joints_[i].joint_name);
          if (limit != info_.limits.end() && limit->second.has_position_limits &&
            !PositionWithinBounds(pose.position[i], limit->second.min_position,
              limit->second.max_position, 0.0))
          {
            throw std::invalid_argument("pose '" + pose.name + "' joint '" +
                    joints_[i].joint_name + "' is outside URDF bounds");
          }
          if (joints_[i].control_mode == robstride_sdk::RunMode::RUN_MODE_VELOCITY ||
            joints_[i].control_mode == robstride_sdk::RunMode::RUN_MODE_CURRENT)
          {
            throw std::invalid_argument("relax-safe poses require position-controlled joints");
          }
        }
      }
      stillness_history_ = std::make_unique<StillnessHistory>(n);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, "Invalid relax_safe_poses_file: " << e.what());
    return CallbackReturn::ERROR;
  }
  measured_hold_ = std::make_unique<MeasuredHold>(n);
  engage_start_position_.resize(n);
  torque_feedback_after_ns_.resize(n);
  torque_hold_sent_.resize(n);
  measured_feedback_ns_.resize(n);
  measured_fault_present_.resize(n);

  relaxed_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "~/relaxed", rclcpp::QoS(1).reliable().transient_local());
  relaxation_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        std_msgs::msg::Bool message;
        message.data = relaxed_state_.load(std::memory_order_relaxed);
        relaxed_pub_->publish(message);
        if (ignored_relaxed_command_.exchange(false, std::memory_order_relaxed)) {
          RCLCPP_WARN_THROTTLE(logger_, *this->rclcpp::Node::get_clock(), 5000,
            "Position/velocity command changed while relaxed; ignored with torque OFF. "
            "Motion entry points must call ~/engage before sending new commands.");
        }
    });

  state_pub_ = this->create_publisher<RobstrideStateMsg>(
    "~/robstride_state", rclcpp::SystemDefaultsQoS());
  state_pub_uni_ptr_ = std::make_unique<StatePublisher>(state_pub_);
  state_pub_uni_ptr_->msg_.id.resize(n);
  state_pub_uni_ptr_->msg_.enabled.resize(n);
  state_pub_uni_ptr_->msg_.run_state.resize(n);
  state_pub_uni_ptr_->msg_.fault_bits.resize(n);

  // Publish from the non-RT executor; read() only updates the atomic latch.
  // Transient local also delivers a trip to a runtime that connects later.
  thermal_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "~/thermal_tripped", rclcpp::QoS(1).reliable().transient_local());
  thermal_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        std_msgs::msg::Bool message;
        message.data = thermal_latch_.tripped();
        thermal_pub_->publish(message);
    });
  thermal_samples_.resize(n);
  thermal_clear_sub_ = this->create_subscription<std_msgs::msg::UInt64>(
    "~/thermal_rest_verified", rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::UInt64::ConstSharedPtr message) {
      // This is a rest-pose attestation by the recovery owner, not a torque
      // command. The RT loop owns the latch and revalidates every motor.
      // The payload is the HOME measurement's same-host steady timestamp,
      // not receipt time: stale or delayed attestations cannot authorize clear.
      thermal_clear_request_ns_.store(message->data, std::memory_order_relaxed);
    });

  get_data_srv_ = this->create_service<robstride_interfaces::srv::GetDataFromRobstride>(
    "~/get_data_from_robstride",
    std::bind(
      &RobstrideHardware::GetDataCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_data_srv_ = this->create_service<robstride_interfaces::srv::SetDataToRobstride>(
    "~/set_data_to_robstride",
    std::bind(
      &RobstrideHardware::SetDataCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_zero_srv_ = this->create_service<robstride_interfaces::srv::SetZeroRobstride>(
    "~/set_zero_robstride",
    std::bind(
      &RobstrideHardware::SetZeroCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_torque_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_torque",
    std::bind(
      &RobstrideHardware::SetTorqueCallback, this, std::placeholders::_1, std::placeholders::_2));
  relax_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/relax",
    std::bind(&RobstrideHardware::RelaxCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  engage_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/engage",
    std::bind(&RobstrideHardware::EngageCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  reboot_srv_ = this->create_service<robstride_interfaces::srv::RebootRobstride>(
    "~/reboot_robstride",
    std::bind(
      &RobstrideHardware::RebootCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  service_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  service_executor_->add_node(this->get_node_base_interface());
  service_executor_thread_ = std::thread(
    [this]() {
      service_executor_->spin();
      service_executor_stopped_ = true;
    });

  RCLCPP_INFO_STREAM(
    logger_,
    "Initialized " << n << " RobStride joint(s) across " << buses_.size() << " CAN bus(es)");
  return CallbackReturn::SUCCESS;
}

// Both exports follow the URDF declaration: an undeclared interface must
// not be claimable.
std::vector<hardware_interface::StateInterface> RobstrideHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & declared = info_.joints[i].state_interfaces;
    const std::string & name = joints_[i].joint_name;
    if (HasInterface(declared, hardware_interface::HW_IF_POSITION)) {
      state_interfaces.emplace_back(
        name, hardware_interface::HW_IF_POSITION, &hw_state_position_[i]);
    }
    if (HasInterface(declared, hardware_interface::HW_IF_VELOCITY)) {
      state_interfaces.emplace_back(
        name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_[i]);
    }
    if (HasInterface(declared, hardware_interface::HW_IF_EFFORT)) {
      state_interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_state_effort_[i]);
    }

    const auto * gpio = FindGpio(joints_[i].gpio_name);
    if (gpio == nullptr) {
      continue;
    }
    if (HasInterface(gpio->state_interfaces, "temperature")) {
      state_interfaces.emplace_back(gpio->name, "temperature", &hw_state_temperature_[i]);
    }
    if (HasInterface(gpio->state_interfaces, "run_state")) {
      state_interfaces.emplace_back(gpio->name, "run_state", &hw_state_run_state_[i]);
    }
    if (HasInterface(gpio->state_interfaces, "fault_bits")) {
      state_interfaces.emplace_back(gpio->name, "fault_bits", &hw_state_fault_bits_[i]);
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobstrideHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & declared = info_.joints[i].command_interfaces;
    const std::string & name = joints_[i].joint_name;
    if (HasInterface(declared, hardware_interface::HW_IF_POSITION)) {
      command_interfaces.emplace_back(
        name, hardware_interface::HW_IF_POSITION, &hw_cmd_position_[i]);
    }
    if (HasInterface(declared, hardware_interface::HW_IF_VELOCITY)) {
      command_interfaces.emplace_back(
        name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocity_[i]);
    }
    if (HasInterface(declared, hardware_interface::HW_IF_EFFORT)) {
      command_interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_cmd_effort_[i]);
    }

    const auto * gpio = FindGpio(joints_[i].gpio_name);
    if (gpio == nullptr) {
      continue;
    }
    if (HasInterface(gpio->command_interfaces, "kp")) {
      command_interfaces.emplace_back(gpio->name, "kp", &hw_cmd_kp_[i]);
    }
    if (HasInterface(gpio->command_interfaces, "kd")) {
      command_interfaces.emplace_back(gpio->name, "kd", &hw_cmd_kd_[i]);
    }
  }
  return command_interfaces;
}

CallbackReturn RobstrideHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (thermal_latch_.tripped()) {
    RCLCPP_ERROR(logger_,
      "Thermal trip latched; refusing activation. Recover at rest before restarting the driver.");
    return CallbackReturn::ERROR;
  }

  // Reactivation is the only way out of a freeze; see read().
  frozen_ = false;
  measured_hold_->Reset();
  relaxed_state_.store(false, std::memory_order_relaxed);
  if (stillness_history_) {stillness_history_->Reset();}
  std::fill(seen_enabled_.begin(), seen_enabled_.end(), false);
  std::fill(recovery_step_.begin(), recovery_step_.end(), RecoveryStep::Waiting);
  for (auto & retry : can_tx_retry_) {
    retry.second.Reset();
  }

  for (auto & kv : buses_) {
    if (!kv.second->Open()) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to open CAN interface '" << kv.first << "'");
      return CallbackReturn::ERROR;
    }
    kv.second->Start();
  }

  // Arm each motor's own CAN watchdog, a second line of defence beside the
  // host-side staleness check. RAM-resident, so lost on a power cycle.
  for (auto & jh : joints_) {
    buses_[jh.can_interface]->SendFrame(
      jh.motor->EncodeParamWriteU32(
        robstride_sdk::ParamIndex::CAN_TIMEOUT, CanTimeoutRaw(motor_can_timeout_ms_)));
  }

  // Select each joint's run_mode (0x7005) before enabling.
  for (auto & jh : joints_) {
    buses_[jh.can_interface]->SendFrame(
      jh.motor->EncodeParamWriteU8(
        robstride_sdk::ParamIndex::RUN_MODE, static_cast<uint8_t>(jh.control_mode)));
  }

  // Activation must never produce motion. The closed-loop modes keep their
  // target inside the motor, so enabling with a stale loc_ref would servo
  // back to wherever it was last aimed. Preload every mode's target from
  // the measured position first. Stop frames are answered with the same
  // Type-2 feedback, so send one as a position probe: harmless on an
  // already-disabled motor, and it also clears a crashed run's leftovers.
  for (auto & jh : joints_) {
    buses_[jh.can_interface]->SendFrame(jh.motor->EncodeDisable(false));
  }

  auto abort_activation = [this]() {
      for (auto & jh : joints_) {
        buses_[jh.can_interface]->SendFrame(jh.motor->EncodeDisable(false));
      }
      for (auto & kv : buses_) {
        kv.second->Stop();
        kv.second->Close();
      }
      return CallbackReturn::ERROR;
    };

  const uint64_t probe_start_ns = robstride_sdk::NowNs();
  const auto probe_deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(error_timeout_ms_));
  auto probed = [probe_start_ns](const JointHandle & jh) {
      // A fault word or truncated frame does not contain a measured position.
      return jh.motor->state().last_feedback_ns.load(std::memory_order_acquire) > probe_start_ns;
    };
  while (std::chrono::steady_clock::now() < probe_deadline &&
    !std::all_of(joints_.begin(), joints_.end(), probed))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    JointHandle & jh = joints_[i];
    if (!probed(jh)) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Joint '" << jh.joint_name << "' (id=" << static_cast<int>(jh.id) <<
          ") reported no position within " << error_timeout_ms_ <<
          " ms; refusing to enable torque against an unknown target");
      return abort_activation();
    }

    // Same source write() uses one cycle later, not MECH_POS: a target in a
    // different frame would just move the jump one cycle later.
    const float measured = jh.motor->state().position.load(std::memory_order_relaxed);
    wrap_seeded_[i] = false;  // a fresh activation chooses the turn again
    const double joint_position = JointPosition(i, measured);
    const bool in_bounds = MeasuredWithinBounds(i, measured);
    measured_in_bounds_[i].store(in_bounds, std::memory_order_relaxed);
    if (!in_bounds) {
      if (torque_enabled_desired_.load(std::memory_order_relaxed)) {
        RCLCPP_ERROR_STREAM(logger_, "Joint '" << jh.joint_name << "' measured position " <<
          joint_position << " is outside its declared bounds; refusing activation");
        return abort_activation();
      }
      RCLCPP_ERROR_STREAM(logger_, "Joint '" << jh.joint_name << "' measured position " <<
        joint_position << " is outside its declared bounds; activating with torque off, and "
        "~/set_torque(true) is refused until it is back within bounds");
    }
    switch (jh.control_mode) {
      case robstride_sdk::RunMode::RUN_MODE_POSITION_PP:
      case robstride_sdk::RunMode::RUN_MODE_POSITION_CSP:
        buses_[jh.can_interface]->SendFrame(
          jh.motor->EncodeParamWriteFloat(robstride_sdk::ParamIndex::LOC_REF, measured));
        break;
      case robstride_sdk::RunMode::RUN_MODE_VELOCITY:
        buses_[jh.can_interface]->SendFrame(
          jh.motor->EncodeParamWriteFloat(robstride_sdk::ParamIndex::SPD_REF, 0.0f));
        break;
      case robstride_sdk::RunMode::RUN_MODE_CURRENT:
        buses_[jh.can_interface]->SendFrame(
          jh.motor->EncodeParamWriteFloat(robstride_sdk::ParamIndex::IQ_REF, 0.0f));
        break;
      default:
        break;
    }

    // Seed the command interfaces from the same reading.
    hw_state_position_[i] = joint_position;
    hw_cmd_position_[i] = hw_state_position_[i];
    soft_prev_cmd_[i] = hw_state_position_[i];
    hw_cmd_velocity_[i] = 0.0;
    hw_cmd_effort_[i] = 0.0;
  }

  UpdateThermalLatch();
  if (thermal_latch_.tripped()) {
    // All motors are still OFF after the probe. Never enable a hot arm.
    torque_enabled_desired_.store(false, std::memory_order_relaxed);
  }

  // Activating without torque is a supported end state: the motors keep
  // answering the Stop frames write() sends, so state still reads while
  // every joint can be moved by hand. Nothing to confirm here.
  if (!torque_enabled_desired_.load(std::memory_order_relaxed)) {
    is_active_ = true;
    RCLCPP_INFO(
      logger_,
      "RobStride hardware activated with torque disabled (torque_enable=false). Joints report "
      "state but are free to move; call ~/set_torque to engage.");
    return CallbackReturn::SUCCESS;
  }

  for (auto & jh : joints_) {
    buses_[jh.can_interface]->SendFrame(jh.motor->EncodeEnable());
  }

  // Wait for each motor to confirm it is enabled. Motors are reused across
  // reactivations, so require a timestamp newer than this activation's own
  // start rather than merely non-zero.
  const uint64_t activate_start_ns = robstride_sdk::NowNs();
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(error_timeout_ms_));
  std::vector<bool> confirmed(joints_.size(), false);
  size_t num_confirmed = 0;
  auto next_resend = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (num_confirmed < joints_.size() && std::chrono::steady_clock::now() < deadline) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (confirmed[i]) {continue;}
      const robstride_sdk::MotorState & st = joints_[i].motor->state();
      if (st.last_feedback_ns.load(std::memory_order_acquire) > activate_start_ns &&
        st.run_state.load(std::memory_order_relaxed) ==
        static_cast<uint8_t>(robstride_sdk::RunState::MOTOR))
      {
        confirmed[i] = true;
        ++num_confirmed;
      }
    }
    if (num_confirmed < joints_.size()) {
      // Enable frames are fire-and-forget; resend in case one was dropped.
      if (std::chrono::steady_clock::now() >= next_resend) {
        for (size_t i = 0; i < joints_.size(); ++i) {
          if (!confirmed[i]) {
            buses_[joints_[i].can_interface]->SendFrame(joints_[i].motor->EncodeEnable());
          }
        }
        next_resend += std::chrono::milliseconds(200);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  if (num_confirmed < joints_.size()) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (!confirmed[i]) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "Joint '" << joints_[i].joint_name << "' (id=" <<
            static_cast<int>(joints_[i].id) << ") did not confirm enable within " <<
            error_timeout_ms_ << " ms");
      }
    }
    // A silent motor and a bus that never transmitted look identical here.
    for (auto & kv : buses_) {
      const int err = kv.second->last_send_errno();
      if (err != 0) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "CAN interface '" << kv.first << "' failed to transmit (" << std::strerror(err) <<
            "), so the enable frames above may never have reached the bus. This usually "
            "means the interface is bus-off; on an adapter without bus-off restart support "
            "it must be re-plugged and re-configured with scripts/setup_can.sh.");
      }
    }
    return abort_activation();
  }

  // Seed the command interfaces from measured state so write()'s first
  // cycle holds position instead of snapping to 0.
  for (size_t i = 0; i < joints_.size(); ++i) {
    hw_state_position_[i] =
      JointPosition(i, joints_[i].motor->state().position.load(std::memory_order_relaxed));
    hw_cmd_position_[i] = hw_state_position_[i];
    soft_prev_cmd_[i] = hw_state_position_[i];
    hw_cmd_velocity_[i] = 0.0;
    hw_cmd_effort_[i] = 0.0;
  }

  is_active_ = true;
  RCLCPP_INFO(logger_, "RobStride hardware activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobstrideHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  // Disabling torque is the right default for a rig that can rest unpowered,
  // but it drops a gravity-loaded arm wherever it happens to be standing. With
  // hold_torque_on_deactivate the motors keep their last target, so a stop
  // holds the pose; nothing new is commanded either way.
  if (hold_torque_on_deactivate_) {
    RCLCPP_WARN(
      logger_,
      "Deactivating with torque engaged (hold_torque_on_deactivate=true): the joints stay "
      "rigid at their current pose. Cut motor power to make the arm limp.");
  } else {
    for (auto & jh : joints_) {
      const auto it = buses_.find(jh.can_interface);
      if (it != buses_.end()) {
        it->second->SendFrame(jh.motor->EncodeDisable(false));
      }
    }
  }
  for (auto & kv : buses_) {
    kv.second->Stop();
    kv.second->Close();
  }

  RCLCPP_INFO(logger_, "RobStride hardware deactivated");
  return CallbackReturn::SUCCESS;
}

return_type RobstrideHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  UpdateThermalLatch();
  bool any_stale = false;
  bool any_lost = false;
  std::string lost_joints;
  const uint64_t now_ns = robstride_sdk::NowNs();
  const bool torque_desired = torque_enabled_desired_.load(std::memory_order_relaxed);
  for (size_t i = 0; i < joints_.size(); ++i) {
    const robstride_sdk::MotorState & st = joints_[i].motor->state();
    // Acquire the complete-feedback stamp before copying its measured fields.
    // Safe services consume this read() snapshot, never a newer stamp over
    // an old RT position buffer.
    measured_feedback_ns_[i] = st.last_feedback_ns.load(std::memory_order_acquire);
    measured_fault_present_[i] = st.fault_present.load(std::memory_order_relaxed);
    const double dir = joints_[i].direction;
    const double motor_position = st.position.load(std::memory_order_relaxed);
    hw_state_position_[i] = JointPosition(i, motor_position);
    measured_in_bounds_[i].store(MeasuredWithinBounds(i, motor_position),
      std::memory_order_relaxed);
    hw_state_velocity_[i] = dir * st.velocity.load(std::memory_order_relaxed);
    hw_state_effort_[i] = dir * st.torque.load(std::memory_order_relaxed);
    hw_state_temperature_[i] = st.temperature.load(std::memory_order_relaxed);
    hw_state_run_state_[i] =
      static_cast<double>(st.run_state.load(std::memory_order_relaxed));
    hw_state_fault_bits_[i] =
      static_cast<double>(st.fault_bits.load(std::memory_order_relaxed));

    // While untorqued, track the command to the measured position so the
    // motor has a fresh target the instant it re-engages.
    const bool enabled = IsEnabled(st);
    if (!enabled) {
      measured_hold_->TrackDisabled(i, hw_state_position_[i]);
      // Keep producer buffers intact in the safe relaxed/hold path. write()
      // must see a controller's actual command, not a value read() overwrote.
      if (!measured_hold_->relaxed() && !measured_hold_->holding()) {
        hw_cmd_position_[i] = hw_state_position_[i];
        hw_cmd_velocity_[i] = 0.0;
        hw_cmd_effort_[i] = 0.0;
      }
      soft_prev_cmd_[i] = hw_state_position_[i];
      // Same for the frozen hold target: a joint that dropped out may have
      // moved while limp, so it must resume holding where it actually is.
      freeze_position_[i] = hw_state_position_[i];
    }

    const uint64_t last_ns = st.last_update_ns.load(std::memory_order_relaxed);
    const double elapsed_ms = ElapsedMsSinceUpdate(now_ns, last_ns);
    const bool stale = elapsed_ms<0.0 || elapsed_ms> error_timeout_ms_;
    any_stale |= stale;

    if (!is_active_) {
      continue;
    }

    // A joint that was enabled and no longer is has rebooted (a power blip
    // clears its RAM) or faulted. Its position may now be a whole turn off
    // and a controller may still hold a setpoint from before, so this is a
    // loss even when it answers again well inside error_timeout_ms.
    bool dropped_out = false;
    if (!torque_desired) {
      seen_enabled_[i] = false;
    } else if (enabled) {
      seen_enabled_[i] = true;
    } else {
      dropped_out = seen_enabled_[i];
    }

    if (stale || dropped_out) {
      any_lost = true;
      // Collected, not logged here: _THROTTLE suppresses per call site, so
      // logging in the loop would hide all but one joint.
      if (!lost_joints.empty()) {
        lost_joints += ", ";
      }
      lost_joints += joints_[i].joint_name + (stale ? " (stale)" : " (left enabled state)");
    }
  }

  if (state_pub_uni_ptr_ && state_pub_uni_ptr_->trylock()) {
    auto & msg = state_pub_uni_ptr_->msg_;
    msg.header.stamp = time;
    // Link freshness is independent of a motor's fault/status word. Keep the
    // existing 0/-1 convention; per-motor fault_bits and enabled remain separate.
    msg.comm_state = any_stale ? -1 : 0;
    for (size_t i = 0; i < joints_.size(); ++i) {
      const robstride_sdk::MotorState & st = joints_[i].motor->state();
      msg.id[i] = joints_[i].id;
      msg.enabled[i] =
        st.run_state.load(std::memory_order_relaxed) ==
        static_cast<uint8_t>(robstride_sdk::RunState::MOTOR);
      msg.run_state[i] = st.run_state.load(std::memory_order_relaxed);
      msg.fault_bits[i] = st.fault_bits.load(std::memory_order_relaxed);
    }
    state_pub_uni_ptr_->unlockAndPublish();
  }

  // Services are spun on their own thread; no spin_some() on this cycle.
  last_read_ns_ = now_ns;

  if (!any_lost && !frozen_) {
    return return_type::OK;
  }

  if (!any_lost) {
    // Frozen but reporting again. Not resumed automatically: a link that
    // dropped once can drop again, and the controllers' setpoints moved on.
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *this->rclcpp::Node::get_clock(), 5000,
      "All joints reporting again but still frozen, holding position and ignoring "
        "controller commands. Reactivate the hardware component to resume control.");
    return return_type::OK;
  }

  if (!freeze_on_joint_loss_) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, *this->rclcpp::Node::get_clock(), 1000,
      "Lost [" << lost_joints << "]; deactivating hardware");
    return return_type::ERROR;
  }

  // ERROR here would deactivate the whole component and cut torque on every
  // joint. Freeze instead: hold the measured positions and ignore commands.
  if (!frozen_) {
    frozen_ = true;
    for (size_t i = 0; i < joints_.size(); ++i) {
      freeze_position_[i] = hw_state_position_[i];
    }
    RCLCPP_ERROR_STREAM(
      logger_,
      "Lost [" << lost_joints << "]; holding all reachable joints at their current "
        "position and ignoring controller commands. A lost joint is not re-enabled. "
        "Reactivate the hardware component to resume control once the cause has been "
        "dealt with.");
  } else {
    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, *this->rclcpp::Node::get_clock(), 5000,
      "Still frozen; lost [" << lost_joints << "]");
  }
  return return_type::OK;
}

return_type RobstrideHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds() > 0.0 ? period.seconds() : 0.01;
  const uint64_t now_ns = robstride_sdk::NowNs();
  if (stillness_history_ && is_active_) {
    stillness_history_->Observe(now_ns, hw_state_position_, hw_state_velocity_,
      hw_cmd_position_, hw_cmd_velocity_, HealthyForTorque(now_ns));
  }
  if (measured_hold_->ObserveCommand(hw_cmd_position_, hw_cmd_velocity_)) {
    ignored_relaxed_command_.store(true, std::memory_order_relaxed);
  }
  ApplyTorqueRequest(now_ns);
  CheckTorqueTransition(now_ns);
  const bool torque_desired = torque_enabled_desired_.load(std::memory_order_relaxed);

  std::unordered_map<std::string, std::vector<can_frame>> frames_by_bus;
  std::unordered_map<std::string, std::vector<size_t>> frame_joints_by_bus;
  std::vector<bool> recovery_queued(joints_.size(), false);
  std::vector<bool> hold_queued(joints_.size(), false);
  for (size_t i = 0; i < joints_.size(); ++i) {
    const JointHandle & jh = joints_[i];
    const robstride_sdk::MotorState & st = jh.motor->state();
    frame_joints_by_bus[jh.can_interface].push_back(i);

    // Apply any gain change parked by SetDataCallback, keeping the service
    // thread away from the buffers this cycle reads.
    if (has_pending_kp_[i].exchange(false, std::memory_order_relaxed)) {
      hw_cmd_kp_[i] = pending_kp_[i].load(std::memory_order_relaxed);
    }
    if (has_pending_kd_[i].exchange(false, std::memory_order_relaxed)) {
      hw_cmd_kd_[i] = pending_kd_[i].load(std::memory_order_relaxed);
    }

    if (!torque_desired) {
      // Sent from this RT cycle, not the service thread, to avoid racing
      // with the frames this loop already sends.
      frames_by_bus[jh.can_interface].push_back(jh.motor->EncodeDisable(false));
      continue;
    }

    const bool currently_enabled = IsEnabled(st);
    if (!currently_enabled && (frozen_ || thermal_latch_.tripped())) {
      // A joint that dropped out stays limp until the hardware is
      // reactivated; re-enabling it here is what let a rebooted motor chase
      // a stale setpoint. The Stop frame keeps its feedback coming.
      frames_by_bus[jh.can_interface].push_back(jh.motor->EncodeDisable(false));
      continue;
    }
    if (!currently_enabled && recovery_step_[i] == RecoveryStep::Waiting &&
      now_ns - last_recovery_ns_[i] >= kRecoveryResendNs)
    {
      recovery_step_[i] = RecoveryStep::Mode;
    }
    if (recovery_step_[i] != RecoveryStep::Waiting) {
      // Progress run_mode -> watchdog -> measured preload -> Enable -> hold
      // across cycles. Retries repeat only the UNSENT step, not the burst.
      frames_by_bus[jh.can_interface].push_back(RecoveryFrame(i));
      recovery_queued[i] = true;
      continue;
    }
    if (!currently_enabled) {
      // While waiting to retry Enable, keep complete feedback fresh with a
      // measured target, even if firmware's enabled state races this cycle.
      frames_by_bus[jh.can_interface].push_back(RecoveryFrame(i));
      continue;
    }

    const uint64_t last_ns = st.last_update_ns.load(std::memory_order_relaxed);
    const double elapsed_ms = ElapsedMsSinceUpdate(now_ns, last_ns);
    const bool stale = elapsed_ms<0.0 || elapsed_ms> error_timeout_ms_;
    // fault_present is refreshed by every feedback frame and self-heals;
    // fault_bits stays latched with nothing to reset it, so it is not used.
    const bool faulted = st.fault_present.load(std::memory_order_relaxed);
    if (stale || faulted) {
      // Don't drive a motor we can't hear toward a possibly stale target.
      frames_by_bus[jh.can_interface].push_back(jh.motor->EncodeDisable(false));
      continue;
    }
    // Motion mode uses the combined Type-1 frame; the other modes are
    // driven by writing their own target parameter. While frozen the
    // controllers are not in charge, so hold the captured position.
    // Controllers speak the joint frame; MotorTargetPosition() maps back.
    const bool seeded_hold = measured_hold_->holding(i);
    const double command_position = seeded_hold ? measured_hold_->position(i) : hw_cmd_position_[i];
    const double target_position = MotorTargetPosition(
      i, frozen_ ? freeze_position_[i] : SoftLimitedTarget(i, command_position, dt));
    const double target_velocity =
      jh.direction * (frozen_ || seeded_hold ? 0.0 : hw_cmd_velocity_[i]);
    const double target_effort =
      jh.direction * (frozen_ || seeded_hold ? 0.0 : hw_cmd_effort_[i]);

    can_frame frame{};
    switch (jh.control_mode) {
      case robstride_sdk::RunMode::RUN_MODE_VELOCITY:
        frame = jh.motor->EncodeParamWriteFloat(
          robstride_sdk::ParamIndex::SPD_REF, static_cast<float>(target_velocity));
        break;
      case robstride_sdk::RunMode::RUN_MODE_POSITION_PP:
      case robstride_sdk::RunMode::RUN_MODE_POSITION_CSP:
        frame = jh.motor->EncodeParamWriteFloat(
          robstride_sdk::ParamIndex::LOC_REF, static_cast<float>(target_position));
        break;
      case robstride_sdk::RunMode::RUN_MODE_CURRENT:
        frame = jh.motor->EncodeParamWriteFloat(
          robstride_sdk::ParamIndex::IQ_REF, static_cast<float>(target_effort));
        break;
      case robstride_sdk::RunMode::RUN_MODE_MOTION:
      case robstride_sdk::RunMode::RUN_MODE_SET_ZERO:
      default:
        frame = jh.motor->EncodeMotionCommand(
          static_cast<float>(target_position),
          static_cast<float>(target_velocity),
          static_cast<float>(hw_cmd_kp_[i]),
          static_cast<float>(hw_cmd_kd_[i]),
          static_cast<float>(frozen_ || seeded_hold ? 0.0 :
          MotorFeedforward(jh, hw_cmd_effort_[i])));
        break;
    }
    frames_by_bus[jh.can_interface].push_back(frame);
    hold_queued[i] = true;
  }

  for (auto & kv : frames_by_bus) {
    const int sent = std::max(0, buses_[kv.first]->SendFrames(kv.second));
    auto & retry = can_tx_retry_.at(kv.first);
    const bool complete = sent == static_cast<int>(kv.second.size());
    retry.Observe(now_ns, complete);
    // Only successfully queued frames may advance the state machine or
    // authorize post-frame feedback. sendmmsg can accept a prefix then fail.
    for (int frame = 0; frame < sent; ++frame) {
      const size_t i = frame_joints_by_bus[kv.first][frame];
      if (recovery_queued[i]) {RecoveryFrameQueued(i, now_ns);}
      const bool first_hold = torque_transition_ == TorqueOperation::Engage && hold_queued[i] &&
        !torque_hold_sent_[i];
      if (torque_transition_ == TorqueOperation::Relax && torque_feedback_after_ns_[i] == 0) {
        torque_feedback_after_ns_[i] = now_ns;
      } else if (first_hold) {
        torque_hold_sent_[i] = true;
        torque_feedback_after_ns_[i] = now_ns;
      }
    }
    if (complete) {continue;}
    const int err = buses_[kv.first]->last_send_errno();
    if (retry.Failed(now_ns, err)) {
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *this->rclcpp::Node::get_clock(), 1000,
        "CAN transmit incomplete on '" << kv.first << "': queued " << sent << " of " <<
          kv.second.size() << " frames (" << (err ? std::strerror(err) : "queue full") <<
          "). Retry budget exhausted or non-transient write failure -- typically bus-off. "
          "Adapters without bus-off restart need a re-plug and scripts/setup_can.sh.");
    } else {
      RCLCPP_WARN_STREAM_THROTTLE(logger_, *this->rclcpp::Node::get_clock(), 1000,
        "Transient CAN TX queue pressure on '" << kv.first << "': queued " << sent << " of " <<
          kv.second.size() << " frames; retrying unsent recovery steps on subsequent cycles " <<
          "for at most 30 ms (no RT wait)");
    }
  }
  return return_type::OK;
}

can_frame RobstrideHardware::RecoveryFrame(size_t i) const
{
  const auto & joint = joints_[i];
  switch (recovery_step_[i]) {
    case RecoveryStep::Mode:
      return joint.motor->EncodeParamWriteU8(robstride_sdk::ParamIndex::RUN_MODE,
          static_cast<uint8_t>(joint.control_mode));
    case RecoveryStep::Watchdog:
      return joint.motor->EncodeParamWriteU32(robstride_sdk::ParamIndex::CAN_TIMEOUT,
          CanTimeoutRaw(motor_can_timeout_ms_));
    case RecoveryStep::Enable:
      return joint.motor->EncodeEnable();
    default:
      break;
  }
  const float position = static_cast<float>(MotorTargetPosition(i, hw_state_position_[i]));
  switch (joint.control_mode) {
    case robstride_sdk::RunMode::RUN_MODE_POSITION_PP:
    case robstride_sdk::RunMode::RUN_MODE_POSITION_CSP:
      return joint.motor->EncodeParamWriteFloat(robstride_sdk::ParamIndex::LOC_REF, position);
    case robstride_sdk::RunMode::RUN_MODE_VELOCITY:
      return joint.motor->EncodeParamWriteFloat(robstride_sdk::ParamIndex::SPD_REF, 0.0f);
    case robstride_sdk::RunMode::RUN_MODE_CURRENT:
      return joint.motor->EncodeParamWriteFloat(robstride_sdk::ParamIndex::IQ_REF, 0.0f);
    default:
      return joint.motor->EncodeMotionCommand(position, 0.0f,
          static_cast<float>(hw_cmd_kp_[i]), static_cast<float>(hw_cmd_kd_[i]), 0.0f);
  }
}

void RobstrideHardware::RecoveryFrameQueued(size_t i, uint64_t now_ns)
{
  switch (recovery_step_[i]) {
    case RecoveryStep::Mode: recovery_step_[i] = RecoveryStep::Watchdog; break;
    case RecoveryStep::Watchdog: recovery_step_[i] = RecoveryStep::Preload; break;
    case RecoveryStep::Preload: recovery_step_[i] = RecoveryStep::Enable; break;
    case RecoveryStep::Enable:
      last_recovery_ns_[i] = now_ns;
      recovery_step_[i] = RecoveryStep::Hold;
      break;
    case RecoveryStep::Hold:
      recovery_step_[i] = RecoveryStep::Waiting;
      if (torque_transition_ == TorqueOperation::Engage) {
        torque_hold_sent_[i] = true;
        torque_feedback_after_ns_[i] = now_ns;
      }
      break;
    case RecoveryStep::Waiting: break;
  }
}

double RobstrideHardware::WrapMotorPosition(size_t i, double motor_position)
{
  const JointHandle & jh = joints_[i];
  const double unwrapped = jh.direction * motor_position;
  const double wrapped = WrapToWindow(unwrapped, jh.wrap_center);
  wrap_offset_[i] = unwrapped - wrapped;
  return wrapped;
}

double RobstrideHardware::JointPosition(size_t i, double motor_position)
{
  const JointHandle & jh = joints_[i];
  if (jh.continuous) {
    return WrapMotorPosition(i, motor_position);
  }
  // The SDK reports a continuous motor angle, but starts a fresh baseline
  // after a feedback gap (a motor power cycle may reframe it). No real motion
  // covers pi in one cycle, so treat such a step as a reframe and fold again.
  if (!wrap_seeded_[i] || std::abs(motor_position - last_motor_position_[i]) > M_PI) {
    const double wrapped = WrapMotorPosition(i, motor_position);
    wrap_seeded_[i] = true;
    last_motor_position_[i] = motor_position;
    return wrapped;
  }
  last_motor_position_[i] = motor_position;
  return jh.direction * motor_position - wrap_offset_[i];
}

double RobstrideHardware::SoftLimitedTarget(size_t i, double target, double dt)
{
  const JointHandle & jh = joints_[i];
  if (!jh.has_soft_limits || !std::isfinite(target)) {
    soft_prev_cmd_[i] = target;
    return target;
  }
  // JointSoftLimiter-style position rule on the previous target: moving toward
  // a soft limit, a step is at most k * (distance left) * dt (k*dt capped at 1),
  // so the target approaches the limit asymptotically and never passes it.
  // A target already beyond a soft limit cannot move further out, and may
  // always move back in. Motion away from the limits is not restricted.
  const double prev = soft_prev_cmd_[i];
  const double gain = std::min(1.0, jh.soft_k_position * dt);
  double limited = target;
  if (target > prev) {
    limited = std::min(target, prev + std::max(0.0, jh.soft_upper - prev) * gain);
  } else if (target < prev) {
    limited = std::max(target, prev - std::max(0.0, prev - jh.soft_lower) * gain);
  }
  soft_prev_cmd_[i] = limited;
  return limited;
}

bool RobstrideHardware::MeasuredWithinBounds(size_t i, double motor_position) const
{
  const JointHandle & jh = joints_[i];
  // After activation a bounded joint reports its continuous value; before it,
  // (or when continuous) the reading is folded into the window.
  const double joint_position = (!jh.continuous && wrap_seeded_[i]) ?
    jh.direction * motor_position - wrap_offset_[i] :
    WrapToWindow(jh.direction * motor_position, jh.wrap_center);
  const auto limit = info_.limits.find(jh.joint_name);
  if (limit == info_.limits.end() || !limit->second.has_position_limits) {
    return std::isfinite(joint_position);
  }
  // One full Type-2 position quantum permits HOME exactly on a limit. This
  // is not a commissioning allowance or a command-limit relaxation.
  const double feedback_quantum = 2.0 * jh.motor->limits().position_max / 65535.0;
  return PositionWithinBounds(joint_position, limit->second.min_position,
           limit->second.max_position, feedback_quantum);
}

double RobstrideHardware::MotorTargetPosition(size_t i, double joint_position) const
{
  const JointHandle & jh = joints_[i];
  return JointToMotorTarget(joint_position, hw_state_position_[i], wrap_offset_[i],
    jh.direction, jh.continuous);
}

JointHandle * RobstrideHardware::FindJointById(uint8_t id)
{
  for (auto & jh : joints_) {
    if (jh.id == id) {return &jh;}
  }
  return nullptr;
}

const hardware_interface::ComponentInfo * RobstrideHardware::FindGpio(
  const std::string & name) const
{
  for (const auto & gpio : info_.gpios) {
    if (gpio.name == name) {return &gpio;}
  }
  return nullptr;
}

void RobstrideHardware::GetDataCallback(
  const std::shared_ptr<robstride_interfaces::srv::GetDataFromRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::GetDataFromRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }
  const robstride_sdk::MotorState & st = jh->motor->state();
  if (request->item_name == "position") {
    response->item_data = st.position.load();
  } else if (request->item_name == "velocity") {
    response->item_data = st.velocity.load();
  } else if (request->item_name == "torque" || request->item_name == "effort") {
    response->item_data = st.torque.load();
  } else if (request->item_name == "temperature") {
    response->item_data = st.temperature.load();
  } else if (request->item_name == "run_state") {
    response->item_data = static_cast<float>(st.run_state.load());
  } else if (request->item_name == "fault_bits") {
    response->item_data = static_cast<float>(st.fault_bits.load());
  } else {
    response->result = false;
    return;
  }
  response->result = true;
}

void RobstrideHardware::SetDataCallback(
  const std::shared_ptr<robstride_interfaces::srv::SetDataToRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::SetDataToRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }
  const size_t idx = static_cast<size_t>(jh - joints_.data());

  // kp/kd are also gpio command interfaces; both paths land in the same
  // slot, which write() owns.
  if (request->item_name == "kp") {
    pending_kp_[idx].store(request->item_data, std::memory_order_relaxed);
    has_pending_kp_[idx].store(true, std::memory_order_relaxed);
    response->result = true;
    return;
  }
  if (request->item_name == "kd") {
    pending_kd_[idx].store(request->item_data, std::memory_order_relaxed);
    has_pending_kd_[idx].store(true, std::memory_order_relaxed);
    response->result = true;
    return;
  }

  // Otherwise treat item_name as a RobStride parameter index and write it.
  // Raw passthrough in the motor frame: gpio direction is not applied.
  try {
    const uint16_t index = static_cast<uint16_t>(std::stoul(request->item_name, nullptr, 0));
    const auto it = buses_.find(jh->can_interface);
    if (it == buses_.end()) {
      response->result = false;
      return;
    }
    response->result =
      it->second->SendFrame(jh->motor->EncodeParamWriteFloat(index, request->item_data));
  } catch (const std::exception &) {
    response->result = false;
  }
}

void RobstrideHardware::SetZeroCallback(
  const std::shared_ptr<robstride_interfaces::srv::SetZeroRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::SetZeroRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }
  const auto it = buses_.find(jh->can_interface);
  response->result = it != buses_.end() && it->second->SendFrame(jh->motor->EncodeSetZero());
  // The next feedback frame reports a relabeled raw position, so discard
  // the wraparound offset.
  jh->motor->ResetPositionTracking();
}

void RobstrideHardware::UpdateThermalLatch()
{
  const bool was_tripped = thermal_latch_.tripped();
  for (const auto & motor : motors_) {
    const auto & state = motor->state();
    if (state.last_update_ns.load(std::memory_order_relaxed) != 0) {
      thermal_latch_.Observe(state.temperature.load(std::memory_order_relaxed));
    }
  }
  if (!was_tripped && thermal_latch_.tripped()) {
    RCLCPP_ERROR(logger_,
      "Thermal cutoff reached: trip latched, torque retained for runtime rest-pose recovery; "
      "automatic re-enable blocked");
  }
  const auto request_ns = thermal_clear_request_ns_.exchange(0, std::memory_order_relaxed);
  const auto now_ns = robstride_sdk::NowNs();
  if (request_ns == 0 || request_ns > now_ns || now_ns - request_ns >= 100'000'000ull ||
    torque_enabled_desired_.load(std::memory_order_relaxed))
  {
    return;
  }
  for (size_t i = 0; i < motors_.size(); ++i) {
    const auto & state = motors_[i]->state();
    const auto stamp = state.last_feedback_ns.load(std::memory_order_acquire);
    thermal_samples_[i] = {
      state.temperature.load(std::memory_order_relaxed), stamp,
      state.run_state.load(std::memory_order_relaxed) ==
      static_cast<uint8_t>(robstride_sdk::RunState::RESET),
      // Type-2 refreshes current health; detailed fault_bits retain history.
      !state.fault_present.load(std::memory_order_relaxed)};
  }
  if (thermal_latch_.TryClear(thermal_samples_, now_ns)) {
    RCLCPP_INFO(logger_,
      "Thermal latch cleared after runtime rest verification and fresh cool OFF feedback; "
      "torque remains OFF, explicit operator re-engagement required");
  }
}

void RobstrideHardware::SetTorqueCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  // Compatibility maintenance path: queue to the writer. The bounds snapshot
  // is RT-published; this callback never reads mutable coordinate buffers.
  // This is intentionally NOT the named-pose safe relax API.
  if (request->data &&
    torque_request_state_.load(std::memory_order_acquire) != TorqueRequestState::Idle)
  {
    response->success = false;
    response->message = "Previous torque transition requires control-loop cleanup";
    return;
  }
  if (request->data && thermal_latch_.tripped()) {
    response->success = false;
    response->message =
      "Thermal trip latched; no re-enable until successful rest/OFF recovery and cooling";
    return;
  }
  for (size_t i = 0; request->data && i < joints_.size(); ++i) {
    if (!measured_in_bounds_[i].load(std::memory_order_relaxed)) {
      response->success = false;
      response->message = "Joint '" + joints_[i].joint_name +
        "' measured position is outside its declared bounds; refusing to enable torque";
      return;
    }
  }
  maintenance_torque_request_.store(request->data ? 1 : 0, std::memory_order_release);
  response->success = true;
}

bool RobstrideHardware::HealthyForTorque(uint64_t now_ns) const
{
  if (!is_active_ || frozen_ || thermal_latch_.tripped() || joints_.empty() ||
    last_read_ns_ == 0 || ElapsedMsSinceUpdate(now_ns, last_read_ns_) > 100.0)
  {
    return false;
  }
  for (const auto & bus : buses_) {
    const int error = bus.second->last_send_errno();
    if (can_tx_retry_.at(bus.first).Failed(now_ns, error)) {return false;}
  }
  const double freshness_ms = std::min(error_timeout_ms_, 100.0);
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & state = joints_[i].motor->state();
    const double age = ElapsedMsSinceUpdate(now_ns, measured_feedback_ns_[i]);
    const auto run_state = static_cast<uint8_t>(hw_state_run_state_[i]);
    if (age < 0.0 || age > freshness_ms || measured_fault_present_[i] ||
      state.fault_present.load(std::memory_order_relaxed) ||
      (run_state != static_cast<uint8_t>(robstride_sdk::RunState::RESET) &&
      run_state != static_cast<uint8_t>(robstride_sdk::RunState::MOTOR)) ||
      !std::isfinite(hw_state_position_[i]) || !std::isfinite(hw_state_velocity_[i]) ||
      !std::isfinite(hw_state_temperature_[i]) || hw_state_temperature_[i] < 0.0)
    {
      return false;
    }
  }
  return true;
}

bool RobstrideHardware::AllMeasuredWithinBounds(size_t & outside) const
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    // Bounds and the seeded target refer to the same read() snapshot.
    if (!measured_in_bounds_[i].load(std::memory_order_relaxed)) {outside = i; return false;}
  }
  return true;
}

void RobstrideHardware::SeedMeasuredCommands()
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    hw_cmd_position_[i] = hw_state_position_[i];
    soft_prev_cmd_[i] = hw_state_position_[i];
    freeze_position_[i] = hw_state_position_[i];
    hw_cmd_velocity_[i] = 0.0;
    hw_cmd_effort_[i] = 0.0;
  }
}

void RobstrideHardware::RestoreConfiguredGains()
{
  // Engage from OFF always starts from the URDF gains. Runtime kp/kd writes
  // (e.g. a float mode) otherwise outlive relax and re-engage until restart.
  bool changed = false;
  for (size_t i = 0; i < joints_.size(); ++i) {
    has_pending_kp_[i].store(false, std::memory_order_relaxed);
    has_pending_kd_[i].store(false, std::memory_order_relaxed);
    changed |= hw_cmd_kp_[i] != configured_kp_[i] || hw_cmd_kd_[i] != configured_kd_[i];
    hw_cmd_kp_[i] = configured_kp_[i];
    hw_cmd_kd_[i] = configured_kd_[i];
  }
  gains_restored_.store(changed, std::memory_order_relaxed);
}

void RobstrideHardware::FinishTorqueRequest(TorqueResult result, size_t index, uint64_t now_ns)
{
  const auto transition = torque_transition_;
  torque_result_.store(result, std::memory_order_relaxed);
  torque_result_index_.store(index, std::memory_order_relaxed);
  torque_result_ms_.store(
    static_cast<double>(now_ns - torque_transition_start_ns_) / 1.0e6,
    std::memory_order_relaxed);
  torque_result_max_step_.store(engage_max_step_, std::memory_order_relaxed);
  torque_transition_ = TorqueOperation::None;
  // Publish only if the waiter has not timed out. A cancelled waiter leaves
  // the mailbox unavailable until RT has performed its OFF cleanup.
  auto state = TorqueRequestState::Active;
  if (!torque_request_state_.compare_exchange_strong(state, TorqueRequestState::Complete,
      std::memory_order_release))
  {
    state = TorqueRequestState::Applying;
    if (!torque_request_state_.compare_exchange_strong(state, TorqueRequestState::Complete,
        std::memory_order_release))
    {
      if (transition != TorqueOperation::None) {
        torque_enabled_desired_.store(false, std::memory_order_relaxed);
        relaxed_state_.store(false, std::memory_order_relaxed);
        std::fill(recovery_step_.begin(), recovery_step_.end(), RecoveryStep::Waiting);
        if (transition == TorqueOperation::Engage) {
          measured_hold_->AbortEngage(engage_from_relaxed_);
        }
      }
      torque_request_state_.store(TorqueRequestState::Idle, std::memory_order_release);
    }
  }
}

void RobstrideHardware::ApplyTorqueRequest(uint64_t now_ns)
{
  const int maintenance = maintenance_torque_request_.exchange(-1, std::memory_order_acquire);
  if (maintenance >= 0) {
    size_t outside = 0;
    if (maintenance == 0 || (!thermal_latch_.tripped() && AllMeasuredWithinBounds(outside))) {
      torque_enabled_desired_.store(maintenance == 1, std::memory_order_relaxed);
      measured_hold_->Reset();
      relaxed_state_.store(false, std::memory_order_relaxed);
      if (stillness_history_) {stillness_history_->Reset();}
      std::fill(recovery_step_.begin(), recovery_step_.end(), RecoveryStep::Waiting);
      if (torque_transition_ != TorqueOperation::None) {
        FinishTorqueRequest(TorqueResult::TimedOut, 0, now_ns);
      }
    }
  }
  auto queued = TorqueRequestState::Queued;
  if (!torque_request_state_.compare_exchange_strong(queued, TorqueRequestState::Applying,
      std::memory_order_acquire))
  {
    if (queued == TorqueRequestState::Cancelled && torque_transition_ == TorqueOperation::None) {
      torque_request_state_.store(TorqueRequestState::Idle, std::memory_order_release);
    }
    return;
  }
  const auto request = torque_request_.load(std::memory_order_relaxed);
  torque_transition_start_ns_ = torque_request_start_ns_.load(std::memory_order_relaxed);
  torque_transition_deadline_ns_ = torque_request_deadline_ns_.load(std::memory_order_acquire);
  engage_max_step_ = 0.0;
  if (now_ns >= torque_transition_deadline_ns_) {
    FinishTorqueRequest(TorqueResult::TimedOut, 0, now_ns); return;
  }
  if (!is_active_) {FinishTorqueRequest(TorqueResult::Inactive, 0, now_ns); return;}
  if (thermal_latch_.tripped()) {FinishTorqueRequest(TorqueResult::ThermalTrip, 0, now_ns); return;}
  if (frozen_) {FinishTorqueRequest(TorqueResult::Frozen, 0, now_ns); return;}
  if (!HealthyForTorque(now_ns)) {FinishTorqueRequest(TorqueResult::Unhealthy, 0, now_ns); return;}
  size_t outside = 0;
  if (!AllMeasuredWithinBounds(outside)) {
    FinishTorqueRequest(TorqueResult::OutOfBounds, outside, now_ns); return;
  }

  if (request == TorqueOperation::Relax) {
    if (!stillness_history_) {
      FinishTorqueRequest(TorqueResult::NotConfigured, 0, now_ns); return;
    }
    // Repeated safe relax is permitted, but an unknown/partial torque state
    // is not a supported-pose permission to disable an otherwise live arm.
    for (const auto & joint : joints_) {
      if (!IsEnabled(joint.motor->state()) && !measured_hold_->relaxed()) {
        FinishTorqueRequest(TorqueResult::Unhealthy, 0, now_ns); return;
      }
    }
    const auto verdict = stillness_history_->Check(relax_safe_poses_, hw_state_position_);
    TorqueResult result = TorqueResult::Confirmed;
    switch (verdict.check) {
      case RelaxCheck::Safe: break;
      case RelaxCheck::NotConfigured: result = TorqueResult::NotConfigured; break;
      case RelaxCheck::PoseMismatch: result = TorqueResult::PoseMismatch; break;
      case RelaxCheck::HistoryMissing: result = TorqueResult::HistoryMissing; break;
      case RelaxCheck::MeasuredMoving: result = TorqueResult::MeasuredMoving; break;
      case RelaxCheck::CommandMoving: result = TorqueResult::CommandMoving; break;
      case RelaxCheck::Unhealthy: result = TorqueResult::Unhealthy; break;
    }
    if (result != TorqueResult::Confirmed) {
      FinishTorqueRequest(result, verdict.pose, now_ns); return;
    }
    relaxed_pose_ = verdict.pose;
    measured_hold_->Relax(hw_cmd_position_, hw_cmd_velocity_);
    torque_enabled_desired_.store(false, std::memory_order_relaxed);
    relaxed_state_.store(false, std::memory_order_relaxed);
    std::fill(recovery_step_.begin(), recovery_step_.end(), RecoveryStep::Waiting);
  } else {
    for (const auto & joint : joints_) {
      if (joint.control_mode == robstride_sdk::RunMode::RUN_MODE_VELOCITY ||
        joint.control_mode == robstride_sdk::RunMode::RUN_MODE_CURRENT)
      {
        FinishTorqueRequest(TorqueResult::Unhealthy, 0, now_ns); return;
      }
    }
    const bool already_enabled = std::all_of(joints_.begin(), joints_.end(),
        [](const auto & joint) {return IsEnabled(joint.motor->state());});
    if (torque_enabled_desired_.load(std::memory_order_relaxed) && already_enabled) {
      // Idempotent entry gate: no torque transition or reseed is performed.
      FinishTorqueRequest(TorqueResult::AlreadyEngaged, 0, now_ns); return;
    }
    for (const auto & joint : joints_) {
      if (IsEnabled(joint.motor->state())) {
        FinishTorqueRequest(TorqueResult::Unhealthy, 0, now_ns); return;
      }
    }
    engage_from_relaxed_ = measured_hold_->relaxed();
    std::copy(hw_state_position_.begin(), hw_state_position_.end(), engage_start_position_.begin());
    measured_hold_->Engage(hw_state_position_, hw_cmd_position_, hw_cmd_velocity_);
    relaxed_state_.store(false, std::memory_order_relaxed);
    SeedMeasuredCommands();
    RestoreConfiguredGains();
    torque_enabled_desired_.store(true, std::memory_order_relaxed);
    std::fill(seen_enabled_.begin(), seen_enabled_.end(), false);
    std::fill(last_recovery_ns_.begin(), last_recovery_ns_.end(), 0);
    std::fill(recovery_step_.begin(), recovery_step_.end(), RecoveryStep::Mode);
  }
  torque_transition_ = request;
  std::fill(torque_feedback_after_ns_.begin(), torque_feedback_after_ns_.end(), 0);
  std::fill(torque_hold_sent_.begin(), torque_hold_sent_.end(), false);
  auto applying = TorqueRequestState::Applying;
  torque_request_state_.compare_exchange_strong(applying, TorqueRequestState::Active,
    std::memory_order_release);
}

void RobstrideHardware::CheckTorqueTransition(uint64_t now_ns)
{
  if (torque_transition_ == TorqueOperation::None) {return;}
  const bool engaging = torque_transition_ == TorqueOperation::Engage;
  TorqueResult failure = TorqueResult::Confirmed;
  size_t outside = 0;
  if (now_ns >= torque_transition_deadline_ns_ ||
    torque_request_state_.load(std::memory_order_acquire) == TorqueRequestState::Cancelled)
  {
    failure = TorqueResult::TimedOut;
  } else if (!is_active_) {
    failure = TorqueResult::Inactive;
  } else if (!HealthyForTorque(now_ns)) {
    failure = TorqueResult::Unhealthy;
  } else if (engaging && !AllMeasuredWithinBounds(outside)) {
    failure = TorqueResult::OutOfBounds;
  }
  if (engaging) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      engage_max_step_ = std::max(engage_max_step_,
          std::abs(hw_state_position_[i] - engage_start_position_[i]));
    }
  }
  if (failure != TorqueResult::Confirmed) {
    // Partial enable or uncertain feedback never retries after failure. OFF
    // frames are scheduled by this same writer, and only an explicit call
    // may try again. Relax failure also leaves the requested OFF state.
    torque_enabled_desired_.store(false, std::memory_order_relaxed);
    std::fill(seen_enabled_.begin(), seen_enabled_.end(), false);
    std::fill(recovery_step_.begin(), recovery_step_.end(), RecoveryStep::Waiting);
    if (engaging) {
      measured_hold_->AbortEngage(engage_from_relaxed_);
    }
    // An unconfirmed transition is not an attestation of supported/all-OFF
    // rest, even though the writer continues requesting OFF for every motor.
    relaxed_state_.store(false, std::memory_order_relaxed);
    FinishTorqueRequest(failure, outside, now_ns); return;
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    const bool enabled = hw_state_run_state_[i] ==
      static_cast<uint8_t>(robstride_sdk::RunState::MOTOR);
    if (torque_feedback_after_ns_[i] == 0 ||
      measured_feedback_ns_[i] <= torque_feedback_after_ns_[i] ||
      (engaging && !torque_hold_sent_[i]) || enabled != engaging)
    {
      return;
    }
  }
  if (engaging) {measured_hold_->ConfirmEngaged();}
  relaxed_state_.store(!engaging, std::memory_order_relaxed);
  FinishTorqueRequest(TorqueResult::Confirmed, relaxed_pose_, now_ns);
}

bool RobstrideHardware::RequestTorque(TorqueOperation operation, std::string & message)
{
  if (torque_request_state_.load(std::memory_order_acquire) != TorqueRequestState::Idle) {
    message = "Previous torque transition is still pending; control-loop confirmation required";
    return false;
  }
  const double timeout_s = std::clamp(error_timeout_ms_ / 1000.0, 0.5, 2.0);
  const auto start_ns = robstride_sdk::NowNs();
  const auto deadline = start_ns + static_cast<uint64_t>(timeout_s * 1.0e9);
  torque_request_start_ns_.store(start_ns, std::memory_order_relaxed);
  torque_request_deadline_ns_.store(deadline, std::memory_order_relaxed);
  torque_request_.store(operation, std::memory_order_relaxed);
  torque_request_state_.store(TorqueRequestState::Queued, std::memory_order_release);
  while (torque_request_state_.load(std::memory_order_acquire) != TorqueRequestState::Complete &&
    rclcpp::ok() && robstride_sdk::NowNs() < deadline + 50'000'000ull)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto state = torque_request_state_.load(std::memory_order_acquire);
  while (state != TorqueRequestState::Complete) {
    if (state == TorqueRequestState::Queued) {
      // Claim queued cancellation atomically; RT cannot consume it now.
      if (torque_request_state_.compare_exchange_weak(state, TorqueRequestState::Idle)) {break;}
    } else if (torque_request_state_.compare_exchange_weak(state, TorqueRequestState::Cancelled)) {
      break;
    }
  }
  if (state != TorqueRequestState::Complete) {
    message = "Torque transition timed out without control-loop confirmation; "
      "state is unconfirmed and any partial engage will be disabled when updates resume";
    return false;
  }
  const auto result = torque_result_.load(std::memory_order_relaxed);
  torque_request_state_.store(TorqueRequestState::Idle, std::memory_order_release);
  switch (result) {
    case TorqueResult::Confirmed:
    case TorqueResult::AlreadyEngaged: {
        std::ostringstream text;
        text << std::fixed << std::setprecision(3);
        if (result == TorqueResult::AlreadyEngaged) {
          text << "Already engaged; no torque transition; ";
        } else if (operation == TorqueOperation::Relax) {
          text << "Relaxed at pose '" << relax_safe_poses_[torque_result_index_.load()].name <<
            "'; ";
        } else {
          text << "Engaged measured-pose hold; ";
          if (gains_restored_.exchange(false, std::memory_order_relaxed)) {
            text << "restored configured kp/kd; ";
          }
        }
        text << "all " << joints_.size() << " joints confirmed; elapsed_ms=" <<
          torque_result_ms_.load() << "; max_step_rad=" << std::setprecision(6) <<
          torque_result_max_step_.load();
        message = text.str();
        return true;
      }
    case TorqueResult::Inactive: message = "Hardware is inactive"; break;
    case TorqueResult::NotConfigured: message = "No relax_safe_poses_file configured"; break;
    case TorqueResult::PoseMismatch: message = "Measured pose matches no relax-safe pose"; break;
    case TorqueResult::HistoryMissing: message = "Complete stillness window not available"; break;
    case TorqueResult::MeasuredMoving: message = "Measured joints are not stationary"; break;
    case TorqueResult::CommandMoving:
      message = "Position/velocity commands are not stationary";
      break;
    case TorqueResult::Unhealthy:
      message = "Hardware unhealthy: fresh complete fault-free feedback required";
      break;
    case TorqueResult::OutOfBounds:
      message = "Joint '" + joints_[torque_result_index_.load()].joint_name +
        "' measured position is outside its declared bounds"; break;
    case TorqueResult::ThermalTrip: message = "Thermal trip latched"; break;
    case TorqueResult::Frozen: message = "Hardware is frozen; fresh activation required"; break;
    case TorqueResult::TimedOut:
      message = "Torque transition timed out; all joints requested OFF";
      break;
  }
  return false;
}

void RobstrideHardware::RelaxCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = RequestTorque(TorqueOperation::Relax, response->message);
}

void RobstrideHardware::EngageCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = RequestTorque(TorqueOperation::Engage, response->message);
}

void RobstrideHardware::RebootCallback(
  const std::shared_ptr<robstride_interfaces::srv::RebootRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::RebootRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }

  // Unconditional by design: whether a given fault is safe to clear is a
  // policy call for the caller, so this never gates on fault type.
  const auto it = buses_.find(jh->can_interface);
  // Not a literal MCU reboot: this is a Stop frame with the clear-fault
  // flag set. write() re-enables the motor on its own afterwards.
  response->result = it != buses_.end() && it->second->SendFrame(jh->motor->EncodeDisable(true));
}

}  // namespace robstride_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robstride_hardware_interface::RobstrideHardware, hardware_interface::SystemInterface)
