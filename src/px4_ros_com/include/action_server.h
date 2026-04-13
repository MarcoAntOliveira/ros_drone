#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "my_px4_interfaces/action/go_to_position.hpp"

// Forward declaration para evitar inclusão circular
class OffboardControl; 

class ActionServer {
public:
    using GoToPosition = my_px4_interfaces::action::GoToPosition;
    using GoalHandleGoToPosition = rclcpp_action::ServerGoalHandle<GoToPosition>;

    // O construtor recebe o nó (para criar o server) e o controlador (para comandar o drone)
    ActionServer(rclcpp::Node::SharedPtr node, OffboardControl* controller);

    // Mova os métodos de callback para public para o create_server acessar ou
    // crie o server dentro do construtor da ActionServer (Melhor).
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPosition::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGoToPosition> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleGoToPosition> goal_handle);

private:
    void execute(const std::shared_ptr<GoalHandleGoToPosition> goal_handle);

    rclcpp::Node::SharedPtr node_;
    OffboardControl* controller_;
    rclcpp_action::Server<GoToPosition>::SharedPtr action_server_;
};
