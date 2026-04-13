#include "action_server.h"
#include "offboard_control.h" // Precisa do header completo aqui

ActionServer::ActionServer(rclcpp::Node::SharedPtr node, OffboardControl* controller)
    : node_(node), controller_(controller) {
    
    this->action_server_ = rclcpp_action::create_server<GoToPosition>(
        node_,
        "go_to_position",
        std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1)
    );
}

rclcpp_action::GoalResponse ActionServer::handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const GoToPosition::Goal> goal) {
    RCLCPP_INFO(node_->get_logger(), "Recebi novo objetivo: x=%.2f, y=%.2f", goal->x, goal->y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ActionServer::handle_accepted(const std::shared_ptr<GoalHandleGoToPosition> goal_handle) {
    // Executa em uma thread separada para não travar o nó
    std::thread{std::bind(&ActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void ActionServer::execute(const std::shared_ptr<GoalHandleGoToPosition> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPosition::Feedback>();
    auto result = std::make_shared<GoToPosition::Result>();

    // Atualiza as variáveis na classe de controle
    controller_->x_ = goal->x;
    controller_->y_ = goal->y;
    controller_->z_ = goal->z;
    controller_->mission_finished_ = false;

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok() && !controller_->mission_finished_) {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }
        // Aqui você pode atualizar o feedback baseado na posição atual do controller_
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    result->success = true;
    goal_handle->succeed(result);
}
