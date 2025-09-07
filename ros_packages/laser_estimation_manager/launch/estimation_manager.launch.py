"""
Arquivo de Lançamento para o nó EstimationManager.
@author Wagner Dantas Garcia <wagnergarcia@eng.ci.ufpb.br>
@date 25 de Junho de 2025

Este launch file é responsável por:
1. Declarar argumentos configuráveis (namespace, arquivo de parâmetros).
2. Iniciar o 'estimation_manager' como um nó de ciclo de vida (LifecycleNode).
3. Passar o arquivo de configuração de parâmetros para o nó.
4. Automatizar as transições de ciclo de vida para que o nó inicie, se configure e
   se ative automaticamente, sem intervenção manual.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg


def generate_launch_description():
    """Gera a configuração completa do lançamento."""

    # --- 1. Declaração dos Argumentos de Lançamento ---
    # Permite que o lançamento seja customizado via linha de comando ou outros launch files.
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            # Tenta obter o namespace da variável de ambiente 'UAV_NAME',
            # se não encontrar, usa 'uav' como padrão.
            default_value=EnvironmentVariable('UAV_NAME', default_value='uav'),
            description='Namespace de nível superior para o nó.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'estimation_manager_params_file',
            # Constrói o caminho completo para o arquivo de parâmetros de forma robusta,
            # encontrando o pacote e juntando os caminhos.
            default_value=PathJoinSubstitution([
                FindPackageShare('laser_estimation_manager'),
                'params',
                'estimation_manager.yaml'
            ]),
            description='Caminho completo para o arquivo de parâmetros do nó.'
        )
    )

    # --- 2. Captura dos Valores dos Argumentos ---
    # LaunchConfiguration() é um placeholder que será substituído pelo valor
    # final do argumento no momento da execução.
    namespace = LaunchConfiguration('namespace')
    estimation_manager_params_file = LaunchConfiguration('estimation_manager_params_file')

    # --- 3. Definição do Nó de Ciclo de Vida ---
    estimation_manager_node = LifecycleNode(
        package='laser_estimation_manager',
        executable='estimation_manager',
        name='estimation_manager',
        namespace=namespace,
        output='screen',
        parameters=[
            estimation_manager_params_file,
            # Passa o namespace também como um parâmetro ROS, útil se o nó precisar
            # internamente do seu próprio namespace.
            {'UAV_NAME': namespace}
        ],
        # Remapeia os tópicos internos do nó (com prefixo ~/) para tópicos globais.
        # Ex: O código C++ que assina "~/odometry_in" irá, na verdade, assinar
        # o tópico global "/px4_api/odometry".
        remappings=[
            ('~/odometry_in', 'px4_api/odometry'),
            ('~/vio_in', 'vins_republisher/odom'),
            ('~/odometry_out', 'estimation_manager/estimation'),
            ('~/set_odometry', 'estimation_manager/active_odometry'),
        ]
    )

    # --- 4. Handlers de Eventos para Automação do Ciclo de Vida ---
    # Esta é a parte "mágica" que gerencia as transições do nó automaticamente.
    event_handlers = [
        # Este handler é ativado quando o processo do nosso nó começa a ser executado.
        RegisterEventHandler(
            OnProcessStart(
                target_action=estimation_manager_node,
                on_start=[
                    # Assim que o nó iniciar, emite um evento para requisitar a transição 'CONFIGURE'.
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(estimation_manager_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
        # Este handler observa as transições de estado do nó.
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=estimation_manager_node,
                start_state='configuring',
                goal_state='inactive', # Ocorre quando a transição 'CONFIGURE' termina com sucesso.
                entities=[
                    # Assim que o nó chegar ao estado 'inactive', emite um evento para requisitar a transição 'ACTIVATE'.
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(estimation_manager_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        )
    ]

    # --- 5. Composição Final da Descrição do Lançamento ---
    ld = LaunchDescription()

    # Adiciona as ações (argumentos, nós, handlers) à descrição final.
    for arg in declared_arguments:
        ld.add_action(arg)

    ld.add_action(estimation_manager_node)

    for handler in event_handlers:
        ld.add_action(handler)

    return ld