import rclpy
from rclpy.node import Node

from vehicle_interfaces.msg import IMU
from vehicle_interfaces.msg import Environment
from vehicle_interfaces.params import GenericParams
from vehicle_interfaces.vehicle_interfaces import VehicleServiceNode

from sense_hat import SenseHat
import numpy as np

# def euler_to_quaternion(yaw, pitch, roll):# XYZ order
#     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     return [qx, qy, qz, qw]

def euler_to_quaternion(yaw, pitch, roll):# ZYX order
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) - np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


class Params(GenericParams):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.topic_IMU_nodeName = 'imu_publisher_node'
        self.topic_IMU_topicName = 'topic_IMU'
        self.topic_IMU_pubInterval_s = 0.5
        self.topic_ENV_nodeName = 'env_publisher_node'
        self.topic_ENV_topicName = 'topic_ENV'
        self.topic_ENV_pubInterval_s = 0.5

        self.declare_parameter('topic_IMU_nodeName', self.topic_IMU_nodeName)
        self.declare_parameter('topic_IMU_topicName', self.topic_IMU_topicName)
        self.declare_parameter('topic_IMU_pubInterval_s', self.topic_IMU_pubInterval_s)
        self.declare_parameter('topic_ENV_nodeName', self.topic_ENV_nodeName)
        self.declare_parameter('topic_ENV_topicName', self.topic_ENV_topicName)
        self.declare_parameter('topic_ENV_pubInterval_s', self.topic_ENV_pubInterval_s)
        self._getParam()
    
    def _getParam(self):
        self.topic_IMU_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_IMU_nodeName').get_parameter_value())
        self.topic_IMU_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_IMU_topicName').get_parameter_value())
        self.topic_IMU_pubInterval_s = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_IMU_pubInterval_s').get_parameter_value())
        self.topic_ENV_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_ENV_nodeName').get_parameter_value())
        self.topic_ENV_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_ENV_topicName').get_parameter_value())
        self.topic_ENV_pubInterval_s = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_ENV_pubInterval_s').get_parameter_value())

class SensePublisher(VehicleServiceNode):
    def __init__(self, params : Params):
        super().__init__(params)
        self.__params = params
        self.addQoSCallbackFunc(self.__qosCallback)

        prof = self.addQoSTracking(params.topic_IMU_topicName)
        if (prof != None):
            self.__imuPublisher = self.create_publisher(IMU, params.topic_IMU_topicName, prof)
        else:
            self.__imuPublisher = self.create_publisher(IMU, params.topic_IMU_topicName, 10)
        
        prof = self.addQoSTracking(params.topic_ENV_topicName)
        if (prof != None):
            self.__envPublisher = self.create_publisher(Environment, params.topic_ENV_topicName, prof)
        else:
            self.__envPublisher = self.create_publisher(Environment, params.topic_ENV_topicName, 10)
        
        self.__sense = SenseHat()
        self.__sense.set_imu_config(True, True, True)
        self.__imu_frame_id = 0
        self.__env_frame_id = 0
        self.__imuTimer = self.create_timer(params.topic_IMU_pubInterval_s, self.imu_timer_callback)
        self.__envTimer = self.create_timer(params.topic_ENV_pubInterval_s, self.env_timer_callback)

    def __qosCallback(self):
        self.get_logger().info('[SensePublisher.__qosCallback] Get qmap size: %d' %len(qmap))
        for topic in qmap:
            self.get_logger().info('[SensePublisher.__qosCallback] Get qmap[%s]' %topic)

    def imu_timer_callback(self):
        # IMU Msg
        imuMsg = IMU()
        imuMsg.header.priority = imuMsg.header.PRIORITY_SENSOR
        imuMsg.header.device_type = imuMsg.header.DEVTYPE_IMU
        imuMsg.header.device_id = self.__params.nodeName + "_IMU"
        imuMsg.header.frame_id = self.__imu_frame_id
        self.__imu_frame_id += 1
        imuMsg.header.stamp_type = self.getTimestampType()
        imuMsg.header.stamp = self.getTimestamp().to_msg()
        imuMsg.header.stamp_offset = self.getCorrectDuration().nanoseconds
        imuMsg.header.ref_publish_time_ms = self.__params.topic_IMU_pubInterval_s * 1000.0

        imuMsg.unit_type = imuMsg.UNIT_ACC_GS | imuMsg.UNIT_ROT_RAD
        
        rot = self.__sense.get_orientation_radians()
        quaternion = euler_to_quaternion(rot['yaw'], rot['pitch'], rot['roll'])
        imuMsg.orientation[0] = quaternion[0]
        imuMsg.orientation[1] = quaternion[1]
        imuMsg.orientation[2] = quaternion[2]
        imuMsg.orientation[3] = quaternion[3]

        gyro = self.__sense.get_gyroscope_raw()# rad/sec
        imuMsg.angular_velocity[0] = gyro['x']
        imuMsg.angular_velocity[1] = gyro['y']
        imuMsg.angular_velocity[2] = gyro['z']

        acc = self.__sense.get_accelerometer_raw()# G
        imuMsg.linear_acceleration[0] = acc['x']# * 9.80665
        imuMsg.linear_acceleration[1] = acc['y']# * 9.80665
        imuMsg.linear_acceleration[2] = acc['z']# * 9.80665

        self.__imuPublisher.publish(imuMsg)
        self.get_logger().info('Publishing: IMU')

    def env_timer_callback(self):
        # Environment Msg
        envMsg = Environment()
        envMsg.header.priority = envMsg.header.PRIORITY_SENSOR
        envMsg.header.device_type = envMsg.header.DEVTYPE_ENVIRONMENT
        envMsg.header.device_id = self.__params.nodeName + "_ENV"
        envMsg.header.frame_id = self.__env_frame_id
        self.__env_frame_id += 1
        envMsg.header.stamp_type = self.getTimestampType()
        envMsg.header.stamp = self.getTimestamp().to_msg()
        envMsg.header.stamp_offset = self.getCorrectDuration().nanoseconds
        envMsg.header.ref_publish_time_ms = self.__params.topic_ENV_pubInterval_s * 1000.0

        envMsg.unit_type = envMsg.UNIT_TEMP_CELSIUS | envMsg.UNIT_PRESS_MBAR

        envMsg.temperature = float((self.__sense.get_temperature_from_humidity() + self.__sense.get_temperature_from_pressure()) / 2)
        envMsg.relative_humidity = float(self.__sense.get_humidity() / 100.0)# normalized percentage
        envMsg.pressure = float(self.__sense.get_pressure())# * 100.0# mbar to Pa, 1bar = 100kPa

        self.__envPublisher.publish(envMsg)
        self.get_logger().info('Publishing: ENV %fC, %f%%rH, %fmbar' %(envMsg.temperature, envMsg.relative_humidity, envMsg.pressure))


def main(args=None):
    rclpy.init(args=args)
    params = Params('sense_params_node')
    sense_publisher = SensePublisher(params)
    rclpy.spin(sense_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sense_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
