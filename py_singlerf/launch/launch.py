#ver=2.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json
import yaml
from yaml import load, Loader

pkgName = 'py_singlerf'

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)

    serviceFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/service.json')
    with open(serviceFilePath, 'r') as f:
        serviceData = json.load(f)

    return LaunchDescription([
        Node(
            package=pkgName,
            namespace=data['generic_prop']['namespace'],
            executable="pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_RFCommSend_nodeName" : data['topic_RFCommSend']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "topic_RFCommSend_topicName" : data['topic_RFCommSend']['topicName'], 
                    "topic_RFCommRecv_nodeName" : data['topic_RFCommRecv']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "topic_RFCommRecv_topicName" : data['topic_RFCommRecv']['topicName'], 
                    "topic_RFCommRecv_pubInterval_s" : data['topic_RFCommRecv']['publishInterval_s'], 
                    "RF_operationMode" : data['RF_prop']['operationMode'], 
                    "RF_address" : data['RF_prop']['address'], 
                    "RF_protocol" : data['RF_prop']['protocol'], 
                    "RF_channel" : data['RF_prop']['channel'], 
                    "RF_dataRate" : data['RF_prop']['dataRate'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devInfoService" : serviceData['devInfoService'], 
                    "devInterface" : serviceData['devInterface'], 
                    "devMultiNode" : serviceData['devMultiNode'], 
                    "qosService" : serviceData['qosService'], 
                    "qosDirPath" : serviceData['qosDirPath'], 
                    "safetyService" : serviceData['safetyService'], 
                    "timesyncService" : serviceData['timesyncService'], 
                    "timesyncPeriod_ms" : serviceData['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : serviceData['timesyncAccuracy_ms'], 
                    "timesyncWaitService" : serviceData['timesyncWaitService'], 
                }
            ]
        )
    ])
