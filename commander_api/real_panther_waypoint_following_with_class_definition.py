import rclpy
from rclpy.node import Node
import tf_transformations
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import pyttsx3
from ament_index_python.packages import get_package_share_directory
import csv
import os


class PantherWaypointFollowing(Node):

    def __init__(self):
        super().__init__('real_panther_waypoint_following')
        self.nav = BasicNavigator(namespace='panther')
        self.voice_engine = pyttsx3.init()
        self.filepath = os.path.join(get_package_share_directory(
            'commander_api'), 'config', 'real_panther_new_waypoints.csv')
        self.waypoints = []
        self.waypoint_reached = False
        self.read_waypoints_from_file()  # Read waypoints from the CSV file
        self.follow_waypoints()

    def create_pose_stamped(self, navigator: BasicNavigator, position_x, position_y, orientation_z):
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

    def read_waypoints_from_file(self):
        with open(self.filepath, newline='') as file:
            reader = csv.reader(file, delimiter=',')
            next(reader)
            for row in reader:
                position_x = float(row[1])
                position_y = float(row[2])
                orientation_z = float(row[3])
                self.waypoints.append(self.create_pose_stamped(
                    self.nav, position_x, position_y, orientation_z))

    def text_to_speech(self, text):
        self.voice_engine.say(text)
        self.voice_engine.runAndWait()

    def follow_waypoints(self):
        self.nav.waitUntilNav2Active()
        self.text_to_speech('Following waypoints...')
        self.nav.followWaypoints(self.waypoints)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if feedback:
                print(
                    'Executing current waypoint: '
                    + str(feedback.current_waypoint + 1)
                    + '/'
                    + str(len(self.waypoints))
                )

        print(self.nav.getResult())
        if self.nav.getResult() == TaskResult.SUCCEEDED:
            print('All waypoints reached successfully!')
            self.text_to_speech('All waypoints reached successfully!')
        else:
            print('Failed to reach all waypoints!')
            self.text_to_speech('Failed to reach all waypoints!')


def main():
    rclpy.init()
    panther_waypoint_following = PantherWaypointFollowing()
    panther_waypoint_following.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
