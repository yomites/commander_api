#!/usr/bin/env python3
import rclpy         # Needed to establish ROS2 communication
# needed to initialize simple commander
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(
        0.0, 0.0, orientation_z)
    pose = PoseStamped()
    # pose.header.frame_id = 'panther/map'
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
    # nav = BasicNavigator(namespace='panther')
    nav = BasicNavigator(namespace='panther')

    # --- Set initial pose
    # initial_pose = create_pose_stamped(nav, 4.0, 1.0, 1.57079632679)
    # nav.setInitialPose(initial_pose)
    # --- Wait for Nav2 to be active
    nav.waitUntilNav2Active()

    position_x = 0.0
    position_y = 0.0
    orientation_z = 0.0

    # --- Go to pose
    goal_pose = create_pose_stamped(nav, position_x, position_y, orientation_z)
    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print('Feedback: ', feedback)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
