/**
 * @brief 
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */
#ifndef MISSION_CYCLE_H
#define MISSION_CYCLE_H

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


//lifecycle
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <chrono>
#include <iostream>
// #include "action_server.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using LifecycleCallbackReturn =
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ActionServer; // forward declaration

class OffboardControl : public rclcpp_lifecycle::LifecycleNode
{
public:
	OffboardControl() : LifecycleNode("offboard_control")
	{

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

	 /// Timer responsável pela publicação periódica
        rclcpp::TimerBase::SharedPtr number_timer_;

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
	LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
	LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
	LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);
    	LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);
	LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);

   

	// No OffboardControl.h (dentro de public:)

	void timer_callback3();

};

#endif
