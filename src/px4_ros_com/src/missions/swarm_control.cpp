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


class SwarmControl : public rclcpp::Node
{
public:
    SwarmControl() : Node("swarm_control")
    {
        drones_.emplace_back(this, "px4_1", 1);
        drones_.emplace_back(this, "px4_2", 2);
        drones_.emplace_back(this, "px4_3", 3);

        timer_ = this->create_wall_timer(100ms,
            std::bind(&SwarmControl::loop, this));
    }

private:
    std::vector<Drone> drones_;
    rclcpp::TimerBase::SharedPtr timer_;

    void loop()
    {
        // exemplo: formação em linha
        drones_[0].set_target(0, 0, -5);
        drones_[1].set_target(2, 0, -5);
        drones_[2].set_target(4, 0, -5);

        for (auto &d : drones_) {
            d.update();
        }
    }
};
