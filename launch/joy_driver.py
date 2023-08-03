from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define launch arguments
    rname_arg = DeclareLaunchArgument('rname', default_value='a1')
    ctrl_level_arg = DeclareLaunchArgument('ctrl_level', default_value='highlevel')
    firmwork_arg = DeclareLaunchArgument('firmwork', default_value='3_2')

    # Define nodes
    lcm_server_node = Node(
        package='unitree_legged_real',
        executable='lcm_server_3_2',
        name='node_lcm_server',
        output='screen',
        arguments=[LaunchConfiguration('rname'), LaunchConfiguration('ctrl_level')]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        arguments=[LaunchConfiguration('rname'), LaunchConfiguration('ctrl_level')]
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(rname_arg)
    ld.add_action(ctrl_level_arg)
    ld.add_action(firmwork_arg)
    ld.add_action(lcm_server_node)
    ld.add_action(joy_node)

    return ld


# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(package='unitree_legged_real',
#                                executable='lcm_server_3_2',
#                                name='node_lcm_server',
#                                respawn=False,
#                                output="screen",
#                                parameters=["a1", "highlevel"])
#         ])
