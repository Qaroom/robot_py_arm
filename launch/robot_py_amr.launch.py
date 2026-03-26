from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    multi_joint_pid_controller = Node(
        package="robot_py_arm", executable="multi_joint_pid_controller",
        name="multi_joint_pid_controller", output="screen"
    )
    position_control_node = Node(
        package="robot_py_arm", executable="position_control_node",
        name="position_control_node", output="screen"
    )
    robot_inverse_kinematics = Node(
        package="robot_py_arm", executable="robot_inverse_kinematics",
        name="robot_inverse_kinematics", output="screen", 
    )

    return LaunchDescription([multi_joint_pid_controller, position_control_node, robot_inverse_kinematics])