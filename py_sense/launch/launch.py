from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml
from yaml import load, Loader

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory('py_sense'), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)
    return LaunchDescription([
        Node(
            package="py_sense",
            namespace=data['node_prop']['namespace'],
            executable="pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_IMU_nodeName" : data['topic_IMU']['nodeName'], 
                    "topic_IMU_topicName" : data['topic_IMU']['topicName'], 
                    "topic_IMU_pubInterval" : data['topic_IMU']['publishInterval'], 
                    "topic_ENV_nodeName" : data['topic_ENV']['nodeName'], 
                    "topic_ENV_topicName" : data['topic_ENV']['topicName'], 
                    "topic_ENV_pubInterval" : data['topic_ENV']['publishInterval'], 
                    "mainNodeName" : data['node_prop']['nodeName'], 
                }
            ]
        )
    ])
