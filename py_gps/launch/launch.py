#ver=2.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json
import yaml
from yaml import load, Loader

pkgName = 'py_gps'

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
                    "topic_GPS_nodeName" : data['topic_GPS']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "topic_GPS_topicName" : data['topic_GPS']['topicName'] + '_' + str(data['generic_prop']['id']), 
                    "topic_GPS_pubInterval_s" : data['topic_GPS']['publishInterval_s'], 
                    "module" : data['GPS_prop']['module'], 
                    "device" : data['GPS_prop']['device'], 
                    "baud_dec" : data['GPS_prop']['baud_dec'], 
                    "caster" : data['GPS_prop']['caster'], 
                    "port" : data['GPS_prop']['port'], 
                    "mountpoint" : data['GPS_prop']['mountpoint'], 
                    "username" : data['GPS_prop']['username'], 
                    "password" : data['GPS_prop']['password'], 

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
