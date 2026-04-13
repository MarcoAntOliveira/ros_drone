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


class Drone
{
public:
    Drone(rclcpp::Node* node, std::string ns, int system_id)
        : node_(node), ns_(ns), system_id_(system_id)
    {
        auto qos = rclcpp::QoS(10).best_effort();

        offboard_pub_ = node_->create_publisher<OffboardControlMode>(
            "/" + ns_ + "/fmu/in/offboard_control_mode", 10);

        traj_pub_ = node_->create_publisher<TrajectorySetpoint>(
            "/" + ns_ + "/fmu/in/trajectory_setpoint", 10);

        cmd_pub_ = node_->create_publisher<VehicleCommand>(
            "/" + ns_ + "/fmu/in/vehicle_command", 10);

        pos_sub_ = node_->create_subscription<VehicleLocalPosition>(
            "/" + ns_ + "/fmu/out/vehicle_local_position",
            qos,
            [this](VehicleLocalPosition::SharedPtr msg) {
                x_ = msg->x;
                y_ = msg->y;
                z_ = msg->z;
            });
    }

    void update()
    {
        publish_offboard_control_mode();
        publish_setpoint();

        if (counter_++ > 10) {
            set_mode_offboard();
            arm();
        }
    }

    void set_target(float x, float y, float z)
    {
        target_x_ = x;
        target_y_ = y;
        target_z_ = z;
    }

private:
    rclcpp::Node* node_;
    std::string ns_;
    int system_id_;

    float x_, y_, z_;
    float target_x_, target_y_, target_z_;

    int counter_ = 0;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pos_sub_;

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = now();
        offboard_pub_->publish(msg);
    }

    void publish_setpoint()
    {
        TrajectorySetpoint msg{};
        msg.position = {target_x_, target_y_, target_z_};
        msg.timestamp = now();
        traj_pub_->publish(msg);
    }

    void set_mode_offboard()
    {
        publish_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

    void arm()
    {
        publish_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
    }

    void publish_command(uint16_t cmd, float p1, float p2=0)
    {
        VehicleCommand msg{};
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.target_system = system_id_;
        msg.from_external = true;
        msg.timestamp = now();
        cmd_pub_->publish(msg);
    }

    uint64_t now()
    {
        return node_->get_clock()->now().nanoseconds() / 1000;
    }
};
