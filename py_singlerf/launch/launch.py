from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml
from yaml import load, Loader

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory('py_singlerf'), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)
    return LaunchDescription([
        Node(
            package="py_singlerf",
            namespace=data['node_prop']['namespace'],
            executable="pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_RFCommSend_nodeName" : data['topic_RFCommSend']['nodeName'], 
                    "topic_RFCommSend_topicName" : data['topic_RFCommSend']['topicName'], 
                    "topic_RFCommRecv_nodeName" : data['topic_RFCommRecv']['nodeName'], 
                    "topic_RFCommRecv_topicName" : data['topic_RFCommRecv']['topicName'], 
                    "topic_RFCommRecv_pubInterval" : data['topic_RFCommRecv']['publishInterval'], 
                    "mainNodeName" : data['node_prop']['nodeName'], 
                    "RF_operationMode" : data['RF_prop']['operationMode'], 
                    "RF_address" : data['RF_prop']['address'], 
                    "RF_protocol" : data['RF_prop']['protocol'], 
                    "RF_channel" : data['RF_prop']['channel'], 
                    "RF_dataRate" : data['RF_prop']['dataRate'], 
                }
            ]
        )
    ])
