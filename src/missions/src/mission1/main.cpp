#include "mission_cycle.h"

int main(int argc, char * argv[])
{
	std::cout << "Iniciando nó Offboard Control..." << std::endl;
	rclcpp::init(argc, argv);
	
    // Cria o nó de ciclo de vida
	auto node = std::make_shared<OffboardControl>();
    
    // Executa o nó
	rclcpp::spin(node->get_node_base_interface());
    
	rclcpp::shutdown();
	return 0;
}
