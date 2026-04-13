/****************************************************************************
 *
 * Offboard control com múltiplos waypoints (3 posições) - CORRIGIDO
 *
 ****************************************************************************/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		// Publishers
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// 🔥 QoS CORRETO (ESSENCIAL)
		rclcpp::QoS qos_profile(10);
		qos_profile.best_effort();
		qos_profile.durability_volatile();

		// Subscriber posição
		vehicle_local_position_sub_ =
			this->create_subscription<VehicleLocalPosition>(
				"/fmu/out/vehicle_local_position",
				qos_profile,
				[this](const VehicleLocalPosition::SharedPtr msg) {

					current_x_ = msg->x;
					current_y_ = msg->y;
					current_z_ = msg->z;

					double t = msg->timestamp / 1e6;

					RCLCPP_INFO(this->get_logger(),
						"[POS] t: %.2f | x: %.2f y: %.2f z: %.2f",
						t, current_x_, current_y_, current_z_);
				});

		offboard_setpoint_counter_ = 0;

		// Timer principal
		timer_ = this->create_wall_timer(100ms, [this]() {

			RCLCPP_INFO(this->get_logger(), "[TIMER] rodando...");

			if (offboard_setpoint_counter_ == 10) {
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
			}

			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		});
	}

	void arm()
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
		RCLCPP_INFO(this->get_logger(), "Arm command sent");
	}

	void disarm()
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
		RCLCPP_INFO(this->get_logger(), "Disarm command sent");
	}

private:
	// ROS
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

	// Controle
	uint64_t offboard_setpoint_counter_;

	// Waypoints
	std::vector<std::array<float, 3>> waypoints_ = {
		{-1.5, -1.5, -5.0},
		{ 1.5, -1.5, -5.0},
		{ 0.0,  1.5, -5.0}
	};

	size_t current_setpoint_ = 0;  // 🔥 corrigido
	float tolerance_ = 0.3;

	// Posição atual
	float current_x_ = 0.0;
	float current_y_ = 0.0;
	float current_z_ = 0.0;

	// =========================

	bool reached_setpoint()
	{
		auto target = waypoints_[current_setpoint_];

		float dx = current_x_ - target[0];
		float dy = current_y_ - target[1];
		float dz = current_z_ - target[2];

		float dist = sqrt(dx*dx + dy*dy + dz*dz);

		return dist < tolerance_;
	}

	void publish_offboard_control_mode()
	{
		OffboardControlMode msg{};
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

		offboard_control_mode_publisher_->publish(msg);
	}

	void publish_trajectory_setpoint()
	{
		// Se chegou no waypoint → próximo
		if (reached_setpoint()) {
			if (current_setpoint_ + 1 < waypoints_.size()) {
				current_setpoint_++;
				RCLCPP_INFO(this->get_logger(),
					"➡ Indo para waypoint %ld", current_setpoint_);
			}
		}

		auto target = waypoints_[current_setpoint_];

		TrajectorySetpoint msg{};
		msg.position = {target[0], target[1], target[2]};
		msg.yaw = -3.14;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

		trajectory_setpoint_publisher_->publish(msg);
	}

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
	{
		VehicleCommand msg{};
		msg.param1 = param1;
		msg.param2 = param2;
		msg.command = command;
		msg.target_system = 1;
		msg.target_component = 1;
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

		vehicle_command_publisher_->publish(msg);
	}
};

// =========================

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();

	return 0;
}
