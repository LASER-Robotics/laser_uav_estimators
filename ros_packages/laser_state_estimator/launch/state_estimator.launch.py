"""
Arquivo de Lançamento para o nó StateEstimator com automação de ciclo de vida e remapeamento de tópicos.
@author Seu Nome <seu_email@dominio.com>
@date 01 de Setembro de 2025

Este launch file é responsável por:
1. Declarar argumentos de lançamento configuráveis, incluindo use_sim_time.
2. Iniciar o 'state_estimator' como um nó de ciclo de vida (LifecycleNode).
3. Passar o arquivo de configuração de parâmetros para o nó.
4. Remapear os tópicos de entrada e saída do nó.
5. Automatizar as transições de ciclo de vida (configure, activate) para que o nó
   esteja pronto para uso imediatamente após o lançamento.
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

    # --- 1. DECLARAÇÃO DOS ARGUMENTOS ---

    # Argumento para o ficheiro de parâmetros
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('laser_state_estimator'),
            'params',
            'state_estimator_params.yaml'
        ]),
        description='Caminho para o ficheiro de parâmetros do estimador.'
    )

    # NOVO: Argumento para controlar o tempo de simulação
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Define se o tempo de simulação (do tópico /clock) deve ser usado.'
    )


    # --- 2. DEFINIÇÃO DO NÓ ---

    # Definição do nó de ciclo de vida
    estimator_node = LifecycleNode(
        package='laser_state_estimator',
        executable='state_estimator_main',
        name='estimation_manager',
        namespace=EnvironmentVariable('UAV_NAME', default_value='uav'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            # MODIFICADO: Usa o valor do argumento 'use_sim_time'
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry_in', 'px4_api/odometry'),
            ('odometry_fast_lio_in', 'fast_lio/odometry'),
            ('odometry_openvins_in', 'vins_republisher/odom'),
            ('imu_in', 'px4_api/imu'),
            ('control_in', 'control_manager/diagnostics'),
            ('odometry_out', 'estimation_manager/estimation'),
            ('odometry_predict', 'estimation_manager/estimation_predict'),
            ('set_odometry', 'set_odometry'),
            ('diagnostics', 'estimation_manager/diagnostics'),
        ]
    )


    # --- 3. HANDLERS DE CICLO DE VIDA (sem alterações) ---

    configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=estimator_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(estimator_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=estimator_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(estimator_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # --- 4. RETORNO DA DESCRIÇÃO DE LANÇAMENTO ---
    
    return LaunchDescription([
        # Adiciona o novo argumento à descrição
        params_file_arg,
        use_sim_time_arg,

        # Restantes ações
        estimator_node,
        configure_event_handler,
        activate_event_handler,
    ])