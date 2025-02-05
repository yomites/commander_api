#!/usr/bin/env python3
import rclpy         # Needed to establish ROS2 communication
# needed to initialize simple commander
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from ament_index_python.packages import get_package_share_directory
import os


def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(
        0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'panther/map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose


def main():
    # --- Init
    rclpy.init()        # This is where the communication is initialized.
    nav = BasicNavigator(namespace='panther')

    filepath = os.path.join(
        get_package_share_directory('commander_api'), 'config', 'panther_waypoints.txt')

    # --- Set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2 to be active
    nav.waitUntilNav2Active()

    filename = filepath
    position_x = 0.0
    position_y = 0.0
    orientation_z = 0.0
    waypoints = []

    with open(filename, 'r') as file:
        for line in file:
            pose_value = line.strip()
            if pose_value == '':
                continue
            position_x = float(pose_value.split()[0])
            position_y = float(pose_value.split()[1])
            orientation_z = float(pose_value.split()[2])
            waypoints.append(create_pose_stamped(
                nav, position_x, position_y, orientation_z))

    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(waypoints))
            )

    print(nav.getResult())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
