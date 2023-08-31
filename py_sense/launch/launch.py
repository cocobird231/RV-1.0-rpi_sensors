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
            namespace=data['generic_prop']['namespace'],
            executable="pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_IMU_nodeName" : data['topic_IMU']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "topic_IMU_topicName" : data['topic_IMU']['topicName'] + '_' + str(data['generic_prop']['id']), 
                    "topic_IMU_pubInterval_s" : data['topic_IMU']['publishInterval_s'], 
                    "topic_ENV_nodeName" : data['topic_ENV']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "topic_ENV_topicName" : data['topic_ENV']['topicName'] + '_' + str(data['generic_prop']['id']), 
                    "topic_ENV_pubInterval_s" : data['topic_ENV']['publishInterval_s'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "qosService" : data['generic_prop']['qosService'], 
                    "qosDirPath" : data['generic_prop']['qosDirPath'], 
                    "safetyService" : data['generic_prop']['safetyService'], 
                    "timesyncService" : data['generic_prop']['timesyncService'], 
                    "timesyncPeriod_ms" : data['generic_prop']['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : data['generic_prop']['timesyncAccuracy_ms'], 
                }
            ]
        )
    ])
