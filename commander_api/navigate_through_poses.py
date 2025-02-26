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
        super().__init__('panther_nav_through_poses')
        self.nav = BasicNavigator(namespace='panther')
        self.voice_engine = pyttsx3.init()
        self.filepath = os.path.join(get_package_share_directory(
            'commander_api'), 'config', 'panther_new_poses.csv')
        self.poses = []
        self.read_poses_from_file()  # Read poses from the CSV file
        self.follow_poses()

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

    def read_poses_from_file(self):
        with open(self.filepath, newline='') as file:
            reader = csv.reader(file, delimiter=',')
            next(reader)
            for row in reader:
                position_x = float(row[1])
                position_y = float(row[2])
                orientation_z = float(row[3])
                self.poses.append(self.create_pose_stamped(
                    self.nav, position_x, position_y, orientation_z))

    def text_to_speech(self, text):
        self.voice_engine.say(text)
        self.voice_engine.runAndWait()

    def follow_poses(self):
        self.nav.waitUntilNav2Active()
        self.text_to_speech('Following poses...')
        self.nav.goThroughPoses(self.poses)
        self.counter = 0
        while not self.nav.isTaskComplete():
            self.counter += 1
            feedback = self.nav.getFeedback()
            if feedback:
                print(
                    'Executing current waypoint: '
                    + str(self.counter)
                    + '/'
                    + str(len(self.poses))
                )

        print(self.nav.getResult())
        if self.nav.getResult() == TaskResult.SUCCEEDED:
            self.text_to_speech('All poses reached!')
        else:
            self.text_to_speech('Failed to reach all poses!')


def main():
    rclpy.init()
    panther_nav_through_poses = NavigateThroughPoses()
    panther_nav_through_poses.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
