import rclpy
from rclpy.node import Node
import tf_transformations
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import pyttsx3
from ament_index_python.packages import get_package_share_directory
import csv
import os


class NavigateThroughPoses(Node):

    def __init__(self):
        super().__init__('navigate_through_poses')
        self.nav = BasicNavigator(namespace='panther')
        self.voice_engine = pyttsx3.init()
        self.filepath = os.path.join(get_package_share_directory(
            'commander_api'), 'config', 'panther_new_poses.csv')
        self.poses = []
        self.read_poses_from_file()  # Read poses from the CSV file
        self.follow_poses()

    def create_pose_stamped(self, navigator: BasicNavigator, position_x, position_y, orientation_w):

        self.pose = PoseStamped()
        self.pose.header.frame_id = 'panther/map'
        self.pose.header.stamp = navigator.get_clock().now().to_msg()
        self.pose.pose.position.x = position_x
        self.pose.pose.position.y = position_y
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = orientation_w
        return self.pose

    def read_poses_from_file(self):
        with open(self.filepath, newline='') as file:
            reader = csv.reader(file, delimiter=',')
            next(reader)
            for row in reader:
                position_x = float(row[1])
                position_y = float(row[2])
                orientation_w = float(row[3])
                self.poses.append(self.create_pose_stamped(
                    self.nav, position_x, position_y, orientation_w))

    def text_to_speech(self, text):
        self.voice_engine.say(text)
        self.voice_engine.runAndWait()

    def follow_poses(self):
        self.nav.waitUntilNav2Active()
        self.text_to_speech('Following poses...')
        self.nav.goThroughPoses(self.poses)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

            if feedback:
                print(
                    'Executing current waypoint: ' + str(feedback.current_pose))

        print(self.nav.getResult())
        if self.nav.getResult() == TaskResult.SUCCEEDED:
            self.text_to_speech('All poses reached!')
        else:
            self.text_to_speech('Failed to reach all poses!')


def main():
    rclpy.init()
    panther_nav_through_poses = NavigateThroughPoses()
    panther_nav_through_poses.nav.lifecycleShutdown()
    rclpy.shutdown()
