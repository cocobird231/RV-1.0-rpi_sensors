import RPi.GPIO as GPIO
import time
import threading

import rclpy
from rclpy.node import Node
from vehicle_interfaces.msg import Distance
from vehicle_interfaces.params import GenericParams
from vehicle_interfaces.vehicle_interfaces import VehicleServiceNode

class Params(GenericParams):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.topic_Ultrasound_nodeName = 'ultrasound_publisher_node'
        self.topic_Ultrasound_topicName = 'topic_Ultrasound'
        self.topic_Ultrasound_pubInterval_s = 0.1

        self.declare_parameter('topic_Ultrasound_nodeName', self.topic_Ultrasound_nodeName)
        self.declare_parameter('topic_Ultrasound_topicName', self.topic_Ultrasound_topicName)
        self.declare_parameter('topic_Ultrasound_pubInterval_s', self.topic_Ultrasound_pubInterval_s)
        self._getParam()
    
    def _getParam(self):
        self.topic_Ultrasound_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_Ultrasound_nodeName').get_parameter_value())
        self.topic_Ultrasound_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_Ultrasound_topicName').get_parameter_value())
        self.topic_Ultrasound_pubInterval_s = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_Ultrasound_pubInterval_s').get_parameter_value())

class UltraSoundPublisher(VehicleServiceNode):

    def __init__(self, params : Params):
        super().__init__(params)
        self.params_ = params

        self.addQoSCallbackFunc(self.__qosCallback)
        
        prof = self.addQoSTracking(params.topic_Ultrasound_topicName)
        if (prof != None):
            self.__publisher = self.create_publisher(Distance, params.topic_Ultrasound_topicName, prof)
        else:
            self.__publisher = self.create_publisher(Distance, params.topic_Ultrasound_topicName, 10)

        self.__frame_id = 0
        self.__timer = self.create_timer(params.topic_Ultrasound_pubInterval_s, self.timer_callback)
        
        self.GPIO_TRIGGER1 = 17# Left ultrasound trigger
        self.GPIO_TRIGGER2 = 27# Mid ultrasound trigger
        self.GPIO_TRIGGER3 = 22# Right ultrasound trigger

        self.GPIO_ECHO1 = 5# Left ultrasound echo
        self.GPIO_ECHO2 = 6# Mid ultrasound echo
        self.GPIO_ECHO3 = 13# Right ultrasound echo
        
        GPIO.setmode(GPIO.BCM)# GPIO Mode (BOARD / BCM)
        GPIO.setup(self.GPIO_TRIGGER1, GPIO.OUT)#set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER2, GPIO.OUT)#set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER3, GPIO.OUT)#set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_ECHO1, GPIO.IN)#set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_ECHO2, GPIO.IN)#set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_ECHO3, GPIO.IN)#set GPIO direction (IN / OUT)

        self.d1 = -1
        self.d2 = -1
        self.d3 = -1

        self.distLock = threading.Lock()

    def __qosCallback(self):
        self.get_logger().info('[UltraSoundPublisher.__qosCallback] Get qmap size: %d' %len(qmap))
        for topic in qmap:
            self.get_logger().info('[UltraSoundPublisher.__qosCallback] Get qmap[%s]' %topic)
    
    def getDistance(self, distNum : int, trigger : int, echo : int):
        GPIO.output(trigger, True)# set Trigger to HIGH
        time.sleep(0.00001)
        GPIO.output(trigger, False)# set Trigger after 0.01ms to LOW

        errF = False
        StartTime = time.time()
        StopTime = time.time()
        
        measureTime = time.time()
        while (GPIO.input(echo) == 0 and not errF):# save StartTime
            StartTime = time.time()# unit: sec
            if (time.time() - measureTime > 0.05):
                errF = True
                break
        measureTime = time.time()
        while (GPIO.input(echo) == 1 and not errF):# save time of arrival
            StopTime = time.time()# unit: sec
            if (time.time() - measureTime > 0.05):
                errF = True
                break
        
        if (errF):
            dist = 100000.0
        else:
            dist = (StopTime - StartTime) * 343.2 / 2.0# unit: meter
        
        self.distLock.acquire()
        if (distNum == 1):
            self.d1 = dist
        elif (distNum == 2):
            self.d2 = dist
        elif (distNum == 3):
            self.d3 = dist
        self.distLock.release()
        return


    def timer_callback(self):
        msg = Distance()
        msg.header.priority = msg.header.PRIORITY_SENSOR
        msg.header.device_type = msg.header.DEVTYPE_ULTRASONIC
        msg.header.device_id = self.params_.nodeName
        msg.header.frame_id = self.__frame_id
        self.__frame_id += 1
        msg.header.stamp_type = self.getTimestampType()
        msg.header.stamp = self.getTimestamp().to_msg()
        msg.header.stamp_offset = self.getCorrectDuration().nanoseconds
        msg.header.ref_publish_time_ms = self.params_.topic_Ultrasound_pubInterval_s * 1000.0

        msg.unit_type = msg.UNIT_METER

        msg.min = 0.02
        msg.max = 4.5
        
        threads = []
        threads.append(threading.Thread(target = self.getDistance, args = (1, self.GPIO_TRIGGER1, self.GPIO_ECHO1)))
        threads.append(threading.Thread(target = self.getDistance, args = (2, self.GPIO_TRIGGER2, self.GPIO_ECHO2)))
        threads.append(threading.Thread(target = self.getDistance, args = (3, self.GPIO_TRIGGER3, self.GPIO_ECHO3)))
        for th in threads:
            th.start()
        for th in threads:
            th.join()
        
        msg.distance = min(self.d1, self.d2, self.d3)

        self.__publisher.publish(msg)
        self.get_logger().info('Publishing: %.2f m (%.2f %.2f %.2f)' %(msg.distance, self.d1, self.d2, self.d3))


def main(args=None):
    rclpy.init(args=args)
    params = Params('ultrasound_params_node')
    ultrasound_publisher = UltraSoundPublisher(params)
    rclpy.spin(ultrasound_publisher)

    ultrasound_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
