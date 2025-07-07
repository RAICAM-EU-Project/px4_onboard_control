/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * 2025 Changda Tian 
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/direct_actuators.hpp>
#include <px4_ros2/control/peripheral_actuators.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "My Manual Mode";

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
    : ModeBase(node, kName), _node(node)
  {
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);
    _actuator_setpoint    = std::make_shared<px4_ros2::DirectActuatorsSetpointType>(*this);
    _peripheral_actuator_controls = std::make_shared<px4_ros2::PeripheralActuatorControls>(*this);

    // Subscribe to velocity command topic
    _cmd_vel_sub = node.create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&FlightModeTest::cmdVelCallback, this, std::placeholders::_1));
  }

  void onActivate() override {}
  void onDeactivate() override {}

  void updateSetpoint(float /*dt_s*/) override
  {
    // 1) Remap /cmd_vel linear [-1,1] to physical speed and then to throttle [0,1].
    float throttle;
    if (_linear_velocity >= 0.0f) {
      // forward: 0…1 → 0…max_forward
      float target_speed = _linear_velocity * _max_forward_speed;
      // map [0, max_forward] → [0.5, 1.0]
      throttle = (target_speed / _max_forward_speed) * 0.5f + 0.5f;
    } else {
      // backward: –1…0 → –max_backward…0
      float target_speed = _linear_velocity * _max_backward_speed;
      // map [–max_backward, 0] → [0.0, 0.5]
      throttle = (target_speed / _max_backward_speed) * 0.5f + 0.5f;
    }
    throttle = std::clamp(throttle, 0.0f, 1.0f);

    // 2) Remap /cmd_vel angular [-1,1] → steering actuator in [-0.9,0.9]
    float steering = std::clamp(_angular_velocity, -1.0f, 1.0f) * 0.9f;
    std::cout<<"steering: "<<steering<<"\n";

    // 3) Optional RC manual override if sticks are moved farther than the above
    float rc_throttle = (_manual_control_input->throttle() + 1.0f) * 0.5f;  // [–1,1] → [0,1]
    float rc_steering = _manual_control_input->roll() * 0.9f;              // [–1,1] → [–0.9,0.9]

    if (std::abs(rc_throttle - 0.5f) > std::abs(throttle - 0.5f) ||
        std::abs(rc_steering)  > std::abs(steering))
    {
      throttle = rc_throttle;
      steering = rc_steering;
    }

    // 4) Publish to the actuators
    Eigen::Matrix<float, 12, 1> motor_commands = Eigen::Matrix<float, 12, 1>::Zero();
    motor_commands[0] = throttle;
    _actuator_setpoint->updateMotors(motor_commands);

    Eigen::Matrix<float, 8, 1> servo_commands = Eigen::Matrix<float, 8, 1>::Zero();
    servo_commands[0] = steering;
    _actuator_setpoint->updateServos(servo_commands);

    // 5) Log for debugging
    RCLCPP_INFO(_node.get_logger(),
                "CmdVel remap → Throttle: %.3f | Steering: %.3f",
                throttle, steering);

    // 6) Pass through any extra RC aux channel if desired
    _peripheral_actuator_controls->set(_manual_control_input->aux1());
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Expecting /cmd_vel: linear.x ∈ [–1,1], angular.z ∈ [–1,1]
    _linear_velocity  = static_cast<float>(msg->linear.x);
    _angular_velocity = static_cast<float>(msg->angular.z);
  }

  // Raw inputs
  float _linear_velocity{0.0f};
  float _angular_velocity{0.0f};

  // Asymmetrical limits
  static constexpr float _max_forward_speed  = 0.6f;  
  static constexpr float _max_backward_speed = 0.6f;  

  // RC override & actuator interfaces
  std::shared_ptr<px4_ros2::ManualControlInput>    _manual_control_input;
  std::shared_ptr<px4_ros2::DirectActuatorsSetpointType> _actuator_setpoint;
  std::shared_ptr<px4_ros2::PeripheralActuatorControls> _peripheral_actuator_controls;

  rclcpp::Node & _node;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
};
