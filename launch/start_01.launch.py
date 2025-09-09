from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    drones = [
        {"id": 1, "fcu_url": "udp://:14540@127.0.0.1:14557", "offset": [0.0, 0.0, 0.0]},
        {"id": 2, "fcu_url": "udp://:14541@127.0.0.1:14558", "offset": [-5.0, 2.9, 0.0]},
        {"id": 3, "fcu_url": "udp://:14542@127.0.0.1:14559", "offset": [-5.0, -2.9, 0.0]},
    ]

    actions = []

    for drone in drones:
        ns = f"drone{drone['id']}"

        # MAVROS for this drone
        actions.append(
            Node(
                package="mavros",
                executable="mavros_node",
                namespace=ns,
                name=f"mavros_node_{drone['id']}",
                output="screen",
                parameters=[{
                    "fcu_url": drone["fcu_url"],
                    "system_id": drone["id"],
                    "plugin_whitelist": ["command" , "local_position" , "sys_status"]  # Only load command plugin
                }]
            )
        )

        actions.append(
        ExecuteProcess(
            cmd=[
                "gnome-terminal", "--",
                "ros2", "run", "px4_test_scripts", "ground_controller"
            ],
            output="screen"
        )
    )


    return LaunchDescription(actions)
