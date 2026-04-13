/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */
#ifndef OFFBOARD_CONTROL_H
#define OFFBOARD_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stdint.h>
//messages
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/actuator_outputs.hpp>
#include <px4_msgs/msg/esc_report.hpp>

//actions
#include <my_px4_interfaces/action/go_to_position.hpp>


#include <chrono>
#include <iostream>
// #include "action_server.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using GoToPosition = my_px4_interfaces::action::GoToPosition;
using GoalHandleGoToPosition = rclcpp_action::ServerGoalHandle<GoToPosition>;

class ActionServer; // forward declaration

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		// 1. QoS para Comandos (Reliable - o que você já tinha)
		auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10));

		// 2. QoS para Sensores/Telemetria (Best Effort - para o EscReport)
		rmw_qos_profile_t qos_profile_sensor = rmw_qos_profile_sensor_data;
		auto qos_best_effort = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor.history, 5), qos_profile_sensor);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		actuator_outputs_sub_ = this->create_subscription<px4_msgs::msg::ActuatorOutputs>(
		"fmu/out/actuator_outputs",qos_best_effort,
		std::bind(&OffboardControl::actuator_outputs_callback, this, std::placeholders::_1)
		);
	
		esc_report_sub_ = this->create_subscription<px4_msgs::msg::EscReport>(
		"/fmu/out/esc_report", qos_best_effort,
		std::bind(&OffboardControl::esc_report_callback, this, std::placeholders::_1)
		);
		vehicle_local_position_subscriber_ =
		this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
			"/fmu/out/vehicle_local_position_v1",
			qos_best_effort,
			std::bind(&OffboardControl::position_callback, this, std::placeholders::_1)
		);
		// action_server_ = std::make_unique<ActionServer>(
		// 	std::shared_ptr<OffboardControl>(this, [](OffboardControl*){}),
		// 	this
		// );

		timer_ = this->create_wall_timer(
			100ms,
			std::bind(&OffboardControl::timer_callback2, this)
		);
		offboard_setpoint_counter_ = 0;
	}

	void arm();
	void disarm();
	float x_ = 0.0;
	float y_ = 0.0;
	float z_ = 0.0;
	bool reached_first_point_ = false; //indica que atingiu as posições do primeiro ponto, para depois ir para o segundo ponto
	bool reached_second_point_ = false; //indica que atingiu as posições do primeiro ponto, para depois ir para o segundo ponto
	int current_wp_ = 0;
	bool mission_finished_ = false;
	float my_yaw = 0.0;
	

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::ActuatorOutputs>::SharedPtr actuator_outputs_sub_;
	rclcpp::Subscription<px4_msgs::msg::EscReport>::SharedPtr esc_report_sub_;


	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	// rclcpp_action::Server<GoToPosition>::SharedPtr action_server_;


	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
	void publish_trajectory_setpoint2();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void timer_callback();
	void timer_callback2();
	void actuator_outputs_callback(const px4_msgs::msg::ActuatorOutputs::SharedPtr msg);
	void esc_report_callback(const px4_msgs::msg::EscReport::SharedPtr msg);
	void timer_callback3();

};

#endif
