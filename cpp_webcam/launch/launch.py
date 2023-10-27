#ver=1.4
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml
from yaml import load, Loader

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory('cpp_webcam'), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)
    return LaunchDescription([
        Node(
            package="cpp_webcam",
            namespace=data['generic_prop']['namespace'],
            executable=data['launch_node'],
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_Webcam_nodeName" : data['topic_Webcam']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "topic_Webcam_topicName" : data['topic_Webcam']['topicName'] + '_' + str(data['generic_prop']['id']), 
                    "topic_Webcam_pubInterval_s" : data['topic_Webcam']['publishInterval_s'], 
                    "topic_Webcam_width" : data['topic_Webcam']['width'], 
                    "topic_Webcam_height" : data['topic_Webcam']['height'], 
                    "camera_cap_id" : data['camera_prop']['cap_id'], 
                    "camera_fps" : data['camera_prop']['fps'], 
                    "camera_width" : data['camera_prop']['width'], 
                    "camera_height" : data['camera_prop']['height'], 
                    "camera_use_color" : data['camera_prop']['use_color'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devInfoService" : data['generic_prop']['devInfoService'], 
                    "devInterface" : data['generic_prop']['devInterface'], 
                    "devMultiNode" : data['generic_prop']['devMultiNode'], 
                    "qosService" : data['generic_prop']['qosService'], 
                    "qosDirPath" : data['generic_prop']['qosDirPath'], 
                    "safetyService" : data['generic_prop']['safetyService'], 
                    "timesyncService" : data['generic_prop']['timesyncService'], 
                    "timesyncPeriod_ms" : data['generic_prop']['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : data['generic_prop']['timesyncAccuracy_ms'], 
                    "timesyncWaitService" : data['generic_prop']['timesyncWaitService'], 
                }
            ]
        )
    ])