#include "offboard.h"
#include "action_server.h"


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.5);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
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

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 5.0, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint2()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.5, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}
void OffboardControl::position_callback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
	x_ = msg->x;
	y_ = msg->y;
	z_ = msg->z;
	RCLCPP_INFO(this->get_logger(),
        "CALLBACK OK → x: %.2f y: %.2f z: %.2f",
        x_, y_, z_);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
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



void OffboardControl::actuator_outputs_callback(const px4_msgs::msg::ActuatorOutputs::SharedPtr msg)
{
    // O array msg->output contém os valores (geralmente PWM entre 1000-2000 ou 0-1)
    // Exemplo: imprimir o valor do Motor 1
    RCLCPP_INFO(this->get_logger(), "Motor 1 Output: %f", msg->output[0]);
    RCLCPP_INFO(this->get_logger(), "Motor 2 Output: %f", msg->output[1]);
    RCLCPP_INFO(this->get_logger(), "Motor 3 Output: %f", msg->output[2]);
    RCLCPP_INFO(this->get_logger(), "Motor 4 Output: %f", msg->output[3]);
    
}
// A função de callback
void OffboardControl::esc_report_callback(const px4_msgs::msg::EscReport::SharedPtr msg) {
    // EscReport NÃO tem array. O dado já está na raiz da mensagem.
    float corrente = msg->esc_current;
    int32_t rpm = msg->esc_rpm;
    uint8_t id = msg->esc_address; // Identificador do motor

    float Kt = 0.019f;
    float torque_estimado = Kt * corrente;

    RCLCPP_INFO(this->get_logger(), "Motor ID %u: RPM: %d | Torque: %.4f Nm", id, rpm, torque_estimado);
}

void OffboardControl::timer_callback3()
{
    // 🔁 Entrar em OFFBOARD e armar
    if (offboard_setpoint_counter_ == 10) {
        this->publish_vehicle_command(
            VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        this->arm();
    }

    // 📡 Sempre precisa publicar isso
    publish_offboard_control_mode();

    float tol = 0.3;

    // 📍 Waypoints (pode virar variável da classe depois)
    std::vector<std::array<float, 3>> waypoints = {
        {0.0, 0.0, -5.0},  // TAKEOFF
        {0.0, 5.0, -5.0},
        {5.0, 5.0, -5.0},
        {5.0, 0.0, -5.0},
        {0.0, 0.0, -5.0}   // volta
    };

    // 🧠 Estado atual (membro da classe!)
    // int current_wp_;
    // bool mission_finished_;

    if (!mission_finished_) {

        auto target = waypoints[current_wp_];

        // 📤 Publica setpoint
        TrajectorySetpoint msg{};
        msg.position = {target[0], target[1], target[2]};
        msg.yaw = -3.14;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        trajectory_setpoint_publisher_->publish(msg);

        // 📊 Debug
        RCLCPP_INFO(this->get_logger(),
            "📍 Indo para WP %d → x: %.2f y: %.2f z: %.2f",
            current_wp_, target[0], target[1], target[2]);

        // ✅ Checagem de chegada
        if (fabs(x_ - target[0]) < tol &&
            fabs(y_ - target[1]) < tol &&
            fabs(z_ - target[2]) < tol)
        {
            RCLCPP_INFO(this->get_logger(),
                "✅ Chegou no waypoint %d!", current_wp_);

            current_wp_++;

            // 🚀 Terminou missão
            if (current_wp_ >= (int)waypoints.size()) {
                mission_finished_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "🏁 Missão finalizada!");
            }
        }

    } else {

        // 🛬 LAND (desce suavemente)
        TrajectorySetpoint msg{};
        msg.position = {x_, y_, -1.0};  // aproxima do chão
        msg.yaw = -3.14;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        trajectory_setpoint_publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
            "🛬 Pousando...");
    }

    // Posição atual
    RCLCPP_INFO(this->get_logger(),
        "📡 Atual → x: %.2f y: %.2f z: %.2f",
        x_, y_, z_);

    //  contador OFFBOARD

    if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
    }
}

void OffboardControl::timer_callback2()
{
    // 🔁 Entrar em OFFBOARD e armar
    if (offboard_setpoint_counter_ == 10) {
        this->publish_vehicle_command(
            VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        this->arm();
    }

    // 📡 Necessário sempre publicar
    publish_offboard_control_mode();

    float tol = 0.3;

    // 👉 Ainda indo para o primeiro ponto
    if (!reached_first_point_) {

        // 📍 Primeiro ponto
        TrajectorySetpoint msg{};
        msg.position = {0.0, 5.0, -5.0};
        msg.yaw = -3.14;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);

        // ✅ Verifica chegada CORRETA
        if (fabs(x_ - 0.0) < tol &&
            fabs(y_ - 5.0) < tol &&
            fabs(z_ + 5.0) < tol)
        {
            RCLCPP_INFO(this->get_logger(),
                "✅ Chegou no ponto 1! Indo pro ponto 2...");
            reached_first_point_ = true;
        }

    } else {

        // 📍 Segundo ponto
        TrajectorySetpoint msg{};
        msg.position = {0.0, 0.0, 0.0};
        msg.yaw = -3.14;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    // 📊 Debug
    RCLCPP_INFO(this->get_logger(),
        "📍 x: %.2f y: %.2f z: %.2f",
        x_, y_, z_);

    // 🔢 contador offboard
    if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
    }
}
// No topo da classe ou no arquivo .h, defina:
// float my_yaw = 0.0;

void OffboardControl::timer_callback()
{
    // 1. Sempre publique o modo e verifique o contador para armar
    if (offboard_setpoint_counter_ == 10) {
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        this->arm();
    }
    publish_offboard_control_mode();

    // 2. Definição segura de Waypoints
    std::vector<std::array<float, 3>> waypoints = {
        {0.0f, 0.0f, -5.0f},
        {0.0f, 5.0f, -5.0f},
        {5.0f, 5.0f, -5.0f},
        {5.0f, 0.0f, -5.0f},
        {0.0f, 0.0f, -5.0f}
    };

    TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.yaw = -3.14f; // Valor padrão para o yaw

    if (!mission_finished_) {
        // Pega o alvo atual de forma segura
        std::array<float, 3> current_target = waypoints[current_wp_];

        msg.position[0] = current_target[0];
        msg.position[1] = current_target[1];
        msg.position[2] = current_target[2];

        // Lógica de debug
        RCLCPP_INFO(this->get_logger(), "📍 Indo para WP %d: [%.1f, %.1f, %.1f]", 
                    current_wp_, msg.position[0], msg.position[1], msg.position[2]);

        // Verificação de chegada
        float tol = 0.3f;
        if (fabs(x_ - current_target[0]) < tol &&
            fabs(y_ - current_target[1]) < tol &&
            fabs(z_ - current_target[2]) < tol) 
        {
            RCLCPP_INFO(this->get_logger(), "✅ WP %d concluído!", current_wp_);
            current_wp_++;
            if (current_wp_ >= (int)waypoints.size()) {
                mission_finished_ = true;
            }
        }
    } else {
        // Lógica de Pouso: Mantém posição horizontal e busca o solo
        msg.position[0] = x_;
        msg.position[1] = y_;
        msg.position[2] = 0.0f; 
        RCLCPP_INFO_ONCE(this->get_logger(), "🏁 Missão finalizada! Iniciando descida...");
    }

    // Publica o setpoint (seja ele waypoint ou pouso)
    trajectory_setpoint_publisher_->publish(msg);

    if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
    }
}
