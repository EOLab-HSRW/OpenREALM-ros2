from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    demo_node = Node(
        package="realm_ros2",
        executable="demo_node",
        name="demo_node",
        parameters=[{
            "type": "master",
            "master": True
        }]
    )
    ld.add_action(demo_node)

    demo_node2 = Node(
        package="realm_ros2",
        executable="demo_node",
        name="demo_node2",
        parameters=[{
            "type": "worker",
            "master": False
        }]
    )
    ld.add_action(demo_node2)


    return ld