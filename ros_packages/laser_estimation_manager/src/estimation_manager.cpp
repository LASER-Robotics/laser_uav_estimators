/* Main function //{ */

/**
 * @file main.cpp
 * @brief Ponto de entrada para o nó EstimationManager.
 * @author Wagner Dantas Garcia <wagnergarcia@eng.ci.ufpb.br>
 * @date 25 de Junho de 2025
 *
 * Este arquivo contém a função principal responsável por inicializar o nó ROS2, criar uma instância
 * do `EstimationManager`, e executar a lógica associada à classe.
 */

#include <rclcpp/rclcpp.hpp>                               // Biblioteca ROS2 para comunicação.
#include <laser_estimation_manager/estimation_manager.hpp> // Classe EstimationManager.

/**
 * @brief Função principal do programa.
 *
 * A função inicializa o sistema ROS2, cria uma instância da classe `EstimationManager`,
 * e executa o loop principal até que o nó seja encerrado.
 *
 * @param argc Número de argumentos passados na linha de comando.
 * @param argv Array de strings com os argumentos passados na linha de comando.
 * @return Retorna 0 ao encerrar o programa com sucesso.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Inicializa o sistema ROS2.

    // Cria uma instância do EstimationManager como um nó ROS2 gerenciado.
    auto node = std::make_shared<laser_estimation_manager::EstimationManager>();

    // Inicia o loop principal do nó utilizando a interface base do nó.
    rclcpp::spin(node->get_node_base_interface());

    // Encerra o sistema ROS2 antes de sair.
    rclcpp::shutdown();
    return 0; // Retorna 0 indicando término bem-sucedido.
}

/*//}*/