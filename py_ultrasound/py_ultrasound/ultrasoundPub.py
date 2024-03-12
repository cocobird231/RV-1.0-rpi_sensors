import RPi.GPIO as GPIO
import time

import rclpy
from rclpy.node import Node
from vehicle_interfaces.msg import Distance
from vehicle_interfaces.params import GenericParams
from vehicle_interfaces.vehicle_interfaces import VehicleServiceNode

class Params(GenericParams):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.topicIds = [0.0, 1.0, 2.0]
        self.topicName = 'ultrasound_0'
        self.pubInterval_s = 0.1
        self.maxRange_m = 5.0

        self.declare_parameter('topicIds', self.topicIds)
        self.declare_parameter('topicName', self.topicName)
        self.declare_parameter('pubInterval_s', self.pubInterval_s)
        self.declare_parameter('maxRange_m', self.maxRange_m)
        self._getParam()

    def _getParam(self):
        self.topicIds = self.get_parameter('topicIds').get_parameter_value().double_array_value
        self.topicName = self.get_parameter('topicName').get_parameter_value().string_value
        self.pubInterval_s = self.get_parameter('pubInterval_s').get_parameter_value().double_value
        self.maxRange_m = self.get_parameter('maxRange_m').get_parameter_value().double_value



class UltraSoundPublisher(VehicleServiceNode):
    def __init__(self, params : Params):
        super().__init__(params)
        self.__params = params
        self.__quantity = len(params.topicIds)# Quantity of used ultrasound sensors.
        self.__timeout_s = params.maxRange_m * 2 / 343.2# Max distance = 0.03 * 343.2 / 2 ~= 5m
        self.__pubList = []
        self.addQoSCallbackFunc(self.__qosCbFunc)

        for id in params.topicIds:
            tName = params.topicName + "_" + str(int(id))
            prof = self.addQoSTracking(tName)
            if (prof != None):
                self.__pubList.append(self.create_publisher(Distance, tName, prof))
            else:
                self.__pubList.append(self.create_publisher(Distance, tName, 10))

        self.__frame_id = 0
        self.__timer = self.create_timer(params.pubInterval_s, self.__timerCbFunc)

        self.__GPIO_TRIGGER1 = 17# Left ultrasound trigger
        self.__GPIO_TRIGGER2 = 27# Mid ultrasound trigger
        self.__GPIO_TRIGGER3 = 22# Right ultrasound trigger

        self.__GPIO_ECHO1 = 5# Left ultrasound echo
        self.__GPIO_ECHO2 = 6# Mid ultrasound echo
        self.__GPIO_ECHO3 = 13# Right ultrasound echo

        GPIO.setmode(GPIO.BCM)# GPIO Mode (BOARD / BCM)
        GPIO.setup(self.__GPIO_TRIGGER1, GPIO.OUT)#set GPIO direction (IN / OUT)
        GPIO.setup(self.__GPIO_TRIGGER2, GPIO.OUT)#set GPIO direction (IN / OUT)
        GPIO.setup(self.__GPIO_TRIGGER3, GPIO.OUT)#set GPIO direction (IN / OUT)
        GPIO.setup(self.__GPIO_ECHO1, GPIO.IN)#set GPIO direction (IN / OUT)
        GPIO.setup(self.__GPIO_ECHO2, GPIO.IN)#set GPIO direction (IN / OUT)
        GPIO.setup(self.__GPIO_ECHO3, GPIO.IN)#set GPIO direction (IN / OUT)

        self.__d1 = 10000.0
        self.__d2 = 10000.0
        self.__d3 = 10000.0

    def __qosCbFunc(self, qmap):
        self.get_logger().info('[UltraSoundPublisher.__qosCbFunc] Get qmap size: %d' %len(qmap))
        for topic in qmap:
            self.get_logger().info('[UltraSoundPublisher.__qosCbFunc] Get qmap[%s]' %topic)

    def __timerCbFunc(self):
        msg = Distance()
        msg.header.priority = msg.header.PRIORITY_SENSOR
        msg.header.device_type = msg.header.DEVTYPE_ULTRASONIC
        msg.header.device_id = self.__params.nodeName
        msg.header.frame_id = self.__frame_id
        self.__frame_id += 1
        msg.header.ref_publish_time_ms = self.__params.pubInterval_s * 1000.0

        msg.unit_type = msg.UNIT_METER

        msg.min = 0.02
        msg.max = self.__params.maxRange_m

        if (self.__quantity >= 1):
            self.__getDistance(1, self.__GPIO_TRIGGER1, self.__GPIO_ECHO1)
            msg.distance = self.__d1
            msg.header.stamp_type = self.getTimestampType()
            msg.header.stamp = self.getTimestamp().to_msg()
            msg.header.stamp_offset = self.getCorrectDuration().nanoseconds
            self.__pubList[0].publish(msg)
        if (self.__quantity >= 2):
            self.__getDistance(2, self.__GPIO_TRIGGER2, self.__GPIO_ECHO2)
            msg.distance = self.__d2
            msg.header.stamp_type = self.getTimestampType()
            msg.header.stamp = self.getTimestamp().to_msg()
            msg.header.stamp_offset = self.getCorrectDuration().nanoseconds
            self.__pubList[1].publish(msg)
        if (self.__quantity >= 3):
            self.__getDistance(3, self.__GPIO_TRIGGER3, self.__GPIO_ECHO3)
            msg.distance = self.__d3
            msg.header.stamp_type = self.getTimestampType()
            msg.header.stamp = self.getTimestamp().to_msg()
            msg.header.stamp_offset = self.getCorrectDuration().nanoseconds
            self.__pubList[2].publish(msg)

        self.get_logger().info('Distance (m): %.2f %.2f %.2f' %(self.__d1, self.__d2, self.__d3))

    def __getDistance(self, distNum : int, trigger : int, echo : int):
        GPIO.output(trigger, True)# set Trigger to HIGH
        time.sleep(0.00001)
        GPIO.output(trigger, False)# set Trigger after 0.01ms to LOW

        errF = False
        StartTime = time.time()
        StopTime = time.time()

        measureTime = time.time()
        while (GPIO.input(echo) == 0 and not errF):# save StartTime
            StartTime = time.time()# unit: sec
            if (time.time() - measureTime > self.__timeout_s):
                errF = True
                break
        measureTime = time.time()
        while (GPIO.input(echo) == 1 and not errF):# save time of arrival
            StopTime = time.time()# unit: sec
            if (time.time() - measureTime > self.__timeout_s):
                errF = True
                break

        if (errF):
            dist = 10000.0
        else:
            dist = (StopTime - StartTime) * 343.2 / 2.0# unit: meter

        if (distNum == 1):
            self.__d1 = dist
        elif (distNum == 2):
            self.__d2 = dist
        elif (distNum == 3):
            self.__d3 = dist
        return



def main(args=None):
    rclpy.init(args=args)
    params = Params('ultrasound_params_node')
    ultrasound_publisher = UltraSoundPublisher(params)
    rclpy.spin(ultrasound_publisher)

    ultrasound_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

