import time
import threading

import rclpy
from rclpy.node import Node
from vehicle_interfaces.msg import GPS
from vehicle_interfaces.params import GenericParams
from vehicle_interfaces.vehicle_interfaces import VehicleServiceNode

from gps3.agps3threaded import AGPS3mechanism
from ntripClient import NtripClient
import ntripClient


class Params(GenericParams):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.topic_GPS_nodeName = 'gps_publisher_node'
        self.topic_GPS_topicName = 'topic_GPS'
        self.topic_GPS_pubInterval_s = 0.5

        self.module = 'ZED-F9P'
        self.caster = ''
        self.port = 2101
        self.mountpoint = ''
        self.username = ''
        self.password = ''

        self.declare_parameter('topic_GPS_nodeName', self.topic_GPS_nodeName)
        self.declare_parameter('topic_GPS_topicName', self.topic_GPS_topicName)
        self.declare_parameter('topic_GPS_pubInterval_s', self.topic_GPS_pubInterval_s)

        self.declare_parameter('module', self.module)
        self.declare_parameter('caster', self.caster)
        self.declare_parameter('port', self.port)
        self.declare_parameter('mountpoint', self.mountpoint)
        self.declare_parameter('username', self.username)
        self.declare_parameter('password', self.password)
        self._getParam()
    
    def _getParam(self):
        self.topic_GPS_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_GPS_nodeName').get_parameter_value())
        self.topic_GPS_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_GPS_topicName').get_parameter_value())
        self.topic_GPS_pubInterval_s = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_GPS_pubInterval_s').get_parameter_value())

        self.module = rclpy.parameter.parameter_value_to_python(self.get_parameter('module').get_parameter_value())
        self.caster = rclpy.parameter.parameter_value_to_python(self.get_parameter('caster').get_parameter_value())
        self.port = rclpy.parameter.parameter_value_to_python(self.get_parameter('port').get_parameter_value())
        self.mountpoint = rclpy.parameter.parameter_value_to_python(self.get_parameter('mountpoint').get_parameter_value())
        self.username = rclpy.parameter.parameter_value_to_python(self.get_parameter('username').get_parameter_value())
        self.password = rclpy.parameter.parameter_value_to_python(self.get_parameter('password').get_parameter_value())



class GPSPublisher(VehicleServiceNode):
    def __init__(self, params):
        super().__init__(params)
        self.__params = params

        self.addQoSCallbackFunc(self.__qosCallback)
        
        prof = self.addQoSTracking(params.topic_GPS_topicName)
        if (prof != None):
            self.__publisher = self.create_publisher(GPS, params.topic_GPS_topicName, prof)
        else:
            self.__publisher = self.create_publisher(GPS, params.topic_GPS_topicName, 10)
        
        self.__frame_id = 0
        self.__timer = self.create_timer(params.topic_GPS_pubInterval_s, self.timer_callback)

        self.__module = params.module

        if (self.__module == 'M8Q'):
            self.gpsThread = AGPS3mechanism()
            self.gpsThread.stream_data()
            self.gpsThread.run_thread()
        elif (self.__module == 'ZED-F9P'):
            ntripArgs = {}
            ntripArgs['user'] = params.username + ":" + params.password
            ntripArgs['caster'] = params.caster
            ntripArgs['port'] = int(params.port)
            ntripArgs['mountpoint'] = params.mountpoint
            if (ntripArgs['mountpoint'][:1] != "/"):
                ntripArgs['mountpoint'] = "/" + ntripArgs['mountpoint']

            self.ntripCli = NtripClient(**ntripArgs)
            self.ntripCliTh = threading.Thread(target=self.ntripCli.readData, daemon=True)
            self.ntripCliTh.start()
    
    def __del__(self):
        self.ntripCliTh.join()
    
    def __qosCallback(self, qmap):
        self.get_logger().info('[GPSPublisher.__qosCallback] Get qmap size: %d' %len(qmap))
        for topic in qmap:
            self.get_logger().info('[GPSPublisher.__qosCallback] Get qmap[%s]' %topic)

    def timer_callback(self):
        msg = GPS()
        msg.header.priority = msg.header.PRIORITY_SENSOR
        msg.header.device_type = msg.header.DEVTYPE_GPS
        msg.header.device_id = self.__params.nodeName
        msg.header.frame_id = self.__frame_id
        self.__frame_id += 1
        msg.header.stamp_type = self.getTimestampType()
        msg.header.stamp = self.getTimestamp().to_msg()
        msg.header.stamp_offset = self.getCorrectDuration().nanoseconds
        msg.header.ref_publish_time_ms = self.__params.topic_GPS_pubInterval_s * 1000.0

        if (self.__module == 'M8Q'):
            if (self.gpsThread.data_stream.lat != 'n/a' and self.gpsThread.data_stream.lon != 'n/a'):
                msg.gps_status = GPS.GPS_SPP
                # gpsTime = self.gpsThread.data_stream.time
                try:
                    msg.latitude = self.gpsThread.data_stream.lat
                    msg.longitude = self.gpsThread.data_stream.lon
                    msg.altitude = self.gpsThread.data_stream.alt
                except:
                    pass
                # speed = self.gpsThread.data_stream.speed
                # track = self.gpsThread.data_stream.track
        elif (self.__module == 'ZED-F9P'):
            ntripClient.ros2DictLock.acquire()
            tmp = ntripClient.gpsDict
            ntripClient.ros2DictLock.release()
            try:
                msg.gps_status = int(tmp['status'])
                msg.latitude = float(tmp['lat'])
                msg.longitude = float(tmp['lon'])
                msg.altitude = float(tmp['alt'])
            except:
                pass

        self.__publisher.publish(msg)
        self.get_logger().info('Publishing: "%d, %f, %f, %f"' %(msg.gps_status, msg.latitude, msg.longitude, msg.altitude))


def main(args=None):
    rclpy.init(args=args)
    params = Params('gps_params_node')
    gps_publisher = GPSPublisher(params)
    rclpy.spin(gps_publisher)

    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()