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
    pose.header.frame_id = 'panther/map'
    # pose.header.frame_id = 'map'
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
    # nav = BasicNavigator()

    # --- Set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2 to be active
    nav.waitUntilNav2Active()

    # --- Send Nav2 goal
    # PI == 3.14 == 180
    # --- Send Nav2 goal (Rosbot_xl waypoint in world map)
    goal_pose1 = create_pose_stamped(nav, 4.0, -3.0, 0.0)
    goal_pose2 = create_pose_stamped(nav, 9.0, 10.0, 3.14)
    goal_pose3 = create_pose_stamped(nav, -2.8, 11.0, 3.14)
    goal_pose4 = create_pose_stamped(nav, -7.0, 9.0, -1.57)
    goal_pose5 = create_pose_stamped(nav, -7.0, 0.0, 3.14)
    goal_pose6 = create_pose_stamped(nav, -9.0, -6.5, 0.0)
    goal_pose7 = create_pose_stamped(nav, 2.0, -4.0, 1.57)
    goal_pose8 = create_pose_stamped(nav, 0.0, 0.0, 0.00)

    # --- Go to one pose
    # nav.goToPose(goal_pose1)

    # while not nav.isTaskComplete():
    #    feedback = nav.getFeedback()
    # print(feedback)

    # --- Follow waypoints
    waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose4,
                 goal_pose5, goal_pose6, goal_pose7, goal_pose8]

    # waypoints = [goal_pose4,
    #             goal_pose5, goal_pose6, goal_pose7, goal_pose8]

    i = 0
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
