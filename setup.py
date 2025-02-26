from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'commander_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='marshall_400@yahoo.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosbot_waypoint_following = commander_api.rosbot_waypoint_following:main',
            'panther_waypoint_following = commander_api.panther_waypoint_following:main',
            'real_panther_waypoint_following_with_namespace = commander_api.real_panther_waypoint_following_with_namespace:main',
            'real_panther_waypoint_following_without_namespace = commander_api.real_panther_waypoint_following_without_namespace:main',
            'panther_go_to_pose = commander_api.panther_go_to_pose:main',
            'panther_waypoint_with_class = commander_api.panther_waypoint_with_class:main',
            'navigate_through_poses = commander_api.navigate_through_poses:main',
            'real_panther_waypoint_following_with_class_definition = commander_api.real_panther_waypoint_following_with_class_definition:main',
        ],
    },
)
