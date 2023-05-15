from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml
from yaml import load, Loader

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory('py_ultrasound'), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)
    return LaunchDescription([
        Node(
            package="py_ultrasound",
            namespace=data['node_prop']['namespace'],
            executable="pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_Ultrasound_nodeName" : data['topic_Ultrasound']['nodeName'], 
                    "topic_Ultrasound_topicName" : data['topic_Ultrasound']['topicName'], 
                    "topic_Ultrasound_pubInterval" : data['topic_Ultrasound']['publishInterval'], 
                    "mainNodeName" : data['node_prop']['nodeName'], 
                }
            ]
        )
    ])
