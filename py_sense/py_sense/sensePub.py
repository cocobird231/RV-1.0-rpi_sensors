import rclpy
from rclpy.node import Node

from vehicle_interfaces.msg import IMU
from vehicle_interfaces.msg import Environment

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


class Params(Node):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.topic_IMU_nodeName = 'imu_publisher_node'
        self.topic_IMU_topicName = 'topic_IMU'
        self.topic_IMU_pubInterval = 0.5
        self.topic_ENV_nodeName = 'env_publisher_node'
        self.topic_ENV_topicName = 'topic_ENV'
        self.topic_ENV_pubInterval = 0.5
        self.mainNodeName = 'sense_node'

        self.declare_parameter('topic_IMU_nodeName', self.topic_IMU_nodeName)
        self.declare_parameter('topic_IMU_topicName', self.topic_IMU_topicName)
        self.declare_parameter('topic_IMU_pubInterval', self.topic_IMU_pubInterval)
        self.declare_parameter('topic_ENV_nodeName', self.topic_ENV_nodeName)
        self.declare_parameter('topic_ENV_topicName', self.topic_ENV_topicName)
        self.declare_parameter('topic_ENV_pubInterval', self.topic_ENV_pubInterval)
        self.declare_parameter('mainNodeName', self.mainNodeName)
        self._getParam()
    
    def _getParam(self):
        self.topic_IMU_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_IMU_nodeName').get_parameter_value())
        self.topic_IMU_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_IMU_topicName').get_parameter_value())
        self.topic_IMU_pubInterval = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_IMU_pubInterval').get_parameter_value())
        self.topic_ENV_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_ENV_nodeName').get_parameter_value())
        self.topic_ENV_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_ENV_topicName').get_parameter_value())
        self.topic_ENV_pubInterval = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_ENV_pubInterval').get_parameter_value())
        self.mainNodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('mainNodeName').get_parameter_value())

class SensePublisher(Node):

    def __init__(self, nodeName : str, topicName_IMU : str, topicName_ENV : str, interval_s):
        super().__init__(nodeName)
        self.nodeName_ = nodeName
        self.imuPublisher_ = self.create_publisher(IMU, topicName_IMU, 10)
        self.envPublisher_ = self.create_publisher(Environment, topicName_ENV, 10)
        self.sense = SenseHat()
        self.sense.set_imu_config(True, True, True)
        self.frame_id_ = 0
        timer_period = interval_s  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # IMU Msg
        imuMsg = IMU()
        imuMsg.header.priority = imuMsg.header.PRIORITY_SENSOR
        imuMsg.header.device_type = imuMsg.header.DEVTYPE_IMU
        imuMsg.header.device_id = self.nodeName_ + "_IMU"
        imuMsg.header.frame_id = self.frame_id_
        imuMsg.header.stamp_type = imuMsg.header.STAMPTYPE_NO_SYNC
        imuMsg.header.stamp = self.get_clock().now().to_msg()

        imuMsg.unit_type = imuMsg.UNIT_ACC_GS | imuMsg.UNIT_ROT_RAD
        
        rot = self.sense.get_orientation_radians()
        quaternion = euler_to_quaternion(rot['yaw'], rot['pitch'], rot['roll'])
        imuMsg.orientation[0] = quaternion[0]
        imuMsg.orientation[1] = quaternion[1]
        imuMsg.orientation[2] = quaternion[2]
        imuMsg.orientation[3] = quaternion[3]

        gyro = self.sense.get_gyroscope_raw()# rad/sec
        imuMsg.angular_velocity[0] = gyro['x']
        imuMsg.angular_velocity[1] = gyro['y']
        imuMsg.angular_velocity[2] = gyro['z']

        acc = self.sense.get_accelerometer_raw()# G
        imuMsg.linear_acceleration[0] = acc['x']# * 9.80665
        imuMsg.linear_acceleration[1] = acc['y']# * 9.80665
        imuMsg.linear_acceleration[2] = acc['z']# * 9.80665
        
        # Environment Msg
        envMsg = Environment()
        envMsg.header.priority = envMsg.header.PRIORITY_SENSOR
        envMsg.header.device_type = envMsg.header.DEVTYPE_ENVIRONMENT
        envMsg.header.device_id = self.nodeName_ + "_ENV"
        envMsg.header.frame_id = self.frame_id_
        envMsg.header.stamp_type = envMsg.header.STAMPTYPE_NO_SYNC
        envMsg.header.stamp = self.get_clock().now().to_msg()

        envMsg.unit_type = envMsg.UNIT_TEMP_CELSIUS | envMsg.UNIT_PRESS_MBAR

        envMsg.temperature = float((self.sense.get_temperature_from_humidity() + self.sense.get_temperature_from_pressure()) / 2)
        envMsg.relative_humidity = float(self.sense.get_humidity() / 100.0)# normalized percentage
        envMsg.pressure = float(self.sense.get_pressure())# * 100.0# mbar to Pa, 1bar = 100kPa

        self.frame_id_ += 1
        
        # Publish
        self.imuPublisher_.publish(imuMsg)
        self.envPublisher_.publish(envMsg)
        self.get_logger().info('Publishing: IMU, %fC, %f%%rH, %fmbar' \
                                %(envMsg.temperature, envMsg.relative_humidity, envMsg.pressure))


def main(args=None):
    rclpy.init(args=args)
    params = Params('sense_params_node')
    sense_publisher = SensePublisher(params.mainNodeName, params.topic_IMU_topicName, params.topic_ENV_topicName, params.topic_IMU_pubInterval)
    rclpy.spin(sense_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sense_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
