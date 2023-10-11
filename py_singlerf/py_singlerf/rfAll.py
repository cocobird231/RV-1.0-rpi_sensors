# Code referenced from: https://github.com/bjarne-hansen/py-nrf24
from datetime import datetime
import struct
import sys
import time
import traceback
import threading

import pigpio
from nrf24 import *

import rclpy
from rclpy.node import Node

from vehicle_interfaces.msg import WheelState
from vehicle_interfaces.params import GenericParams
from vehicle_interfaces.vehicle_interfaces import VehicleServiceNode

class Params(GenericParams):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.topic_RFCommSend_nodeName = 'rfcommsend_subscriber_node'
        self.topic_RFCommSend_topicName = 'topic_RFCommSend'
        self.topic_RFCommRecv_nodeName = 'rfcommrecv_publisher_node'
        self.topic_RFCommRecv_topicName = 'topic_RFCommRecv'
        self.topic_RFCommRecv_pubInterval_s = 0.1

        self.RF_operationMode = 'send'
        self.RF_address = '1SNSR'
        self.RF_protocol = 0x01
        self.RF_channel = 100
        self.RF_dataRate = 250

        self.declare_parameter('topic_RFCommSend_nodeName', self.topic_RFCommSend_nodeName)
        self.declare_parameter('topic_RFCommSend_topicName', self.topic_RFCommSend_topicName)
        self.declare_parameter('topic_RFCommRecv_nodeName', self.topic_RFCommRecv_nodeName)
        self.declare_parameter('topic_RFCommRecv_topicName', self.topic_RFCommRecv_topicName)
        self.declare_parameter('topic_RFCommRecv_pubInterval_s', self.topic_RFCommRecv_pubInterval_s)

        self.declare_parameter('RF_operationMode', self.RF_operationMode)
        self.declare_parameter('RF_address', self.RF_address)
        self.declare_parameter('RF_protocol', self.RF_protocol)
        self.declare_parameter('RF_channel', self.RF_channel)
        self.declare_parameter('RF_dataRate', self.RF_dataRate)
        self._getParam()
    
    def _getParam(self):
        self.topic_RFCommSend_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_RFCommSend_nodeName').get_parameter_value())
        self.topic_RFCommSend_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_RFCommSend_topicName').get_parameter_value())
        self.topic_RFCommRecv_nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_RFCommRecv_nodeName').get_parameter_value())
        self.topic_RFCommRecv_topicName = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_RFCommRecv_topicName').get_parameter_value())
        self.topic_RFCommRecv_pubInterval_s = rclpy.parameter.parameter_value_to_python(self.get_parameter('topic_RFCommRecv_pubInterval_s').get_parameter_value())

        self.RF_operationMode = rclpy.parameter.parameter_value_to_python(self.get_parameter('RF_operationMode').get_parameter_value())
        self.RF_address = rclpy.parameter.parameter_value_to_python(self.get_parameter('RF_address').get_parameter_value())
        self.RF_protocol = rclpy.parameter.parameter_value_to_python(self.get_parameter('RF_protocol').get_parameter_value())
        self.RF_channel = rclpy.parameter.parameter_value_to_python(self.get_parameter('RF_channel').get_parameter_value())
        self.RF_dataRate = rclpy.parameter.parameter_value_to_python(self.get_parameter('RF_dataRate').get_parameter_value())

class SingleRFSubscriber(VehicleServiceNode):# Send mode
    def __init__(self, params):
        super().__init__(params)

        # Connect to pigpiod
        print(f'Connecting to GPIO daemon on localhost:8888 ...')
        self.pi = pigpio.pi('localhost', 8888)
        if not self.pi.connected:
            print("Not connected to Raspberry Pi ... goodbye.")
            sys.exit()

        # Create NRF24 object.
        self.address = params.RF_address
        self.protocol = params.RF_protocol
        self.nrf = NRF24(self.pi, ce=25, payload_size=RF24_PAYLOAD.DYNAMIC, channel=params.RF_channel, data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.MAX)
        self.nrf.set_address_bytes(len(self.address))
        self.nrf.open_writing_pipe(self.address)
    
        # Display the content of NRF24L01 device registers.
        self.nrf.show_registers()

        self.addQoSCallbackFunc(self.__qosCallback)
        prof = self.addQoSTracking(params.topic_RFCommSend_topicName)
        if (prof != None):
            self.__subscription = self.create_subscription(
                WheelState,
                params.topic_RFCommSend_topicName,
                self.listener_callback,
                prof)
        else:
            self.__subscription = self.create_subscription(
                WheelState,
                params.topic_RFCommSend_topicName,
                self.listener_callback,
                10)

    def __qosCallback(self, qmap):
        self.get_logger().info('[SingleRFSubscriber.__qosCallback] Get qmap size: %d' %len(qmap))
        for topic in qmap:
            self.get_logger().info('[SingleRFSubscriber.__qosCallback] Get qmap[%s]' %topic)
    
    def listener_callback(self, msg):
        payload = struct.pack("<BBllllBB", self.protocol, int(msg.gear), int(msg.steering), int(msg.pedal_throttle), int(msg.pedal_brake), int(msg.pedal_clutch), \
                                            int(msg.button), int(msg.func))

        # Send the payload to the address specified above.
        self.nrf.reset_packages_lost()
        self.nrf.send(payload)
        try:
            self.nrf.wait_until_sent()
        except TimeoutError:
            print('Timeout waiting for transmission to complete.')
        
        if self.nrf.get_packages_lost() == 0:
            print(f"Success: lost={self.nrf.get_packages_lost()}, retries={self.nrf.get_retries()}")
        else:
            print(f"Error: lost={self.nrf.get_packages_lost()}, retries={self.nrf.get_retries()}")
            
        self.get_logger().info('I heard: %03d | %05d %05d %05d %05d | %03d %03d' \
            %(msg.gear, msg.steering, msg.pedal_throttle, msg.pedal_brake, msg.pedal_clutch, \
                msg.button, msg.func))

class SingleRFPublisher(VehicleServiceNode):# Recv mode
    def __init__(self, params):
        super().__init__(params)
        self.params_ = params
        # Connect to pigpiod
        print(f'Connecting to GPIO daemon on localhost:8888 ...')
        self.pi = pigpio.pi('localhost', 8888)
        if not self.pi.connected:
            print("Not connected to Raspberry Pi ... goodbye.")
            sys.exit()
        
        # Create NRF24 object.
        self.address = params.RF_address
        self.protocol = params.RF_protocol
        self.nrf = NRF24(self.pi, ce=25, payload_size=RF24_PAYLOAD.DYNAMIC, channel=params.RF_channel, data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.MAX)
        self.nrf.set_address_bytes(len(self.address))
        self.nrf.open_reading_pipe(RF24_RX_ADDR.P1, self.address)
        self.nrf.show_registers()

        self.addQoSCallbackFunc(self.__qosCallback)
        prof = self.addQoSTracking(params.topic_RFCommRecv_topicName)
        if (prof != None):
            self.__publisher = self.create_publisher(WheelState, params.topic_RFCommRecv_topicName, prof)
        else:
            self.__publisher = self.create_publisher(WheelState, params.topic_RFCommRecv_topicName, 10)

        self.__frame_id = 0
        self.runRFRecv()
    
    def __qosCallback(self, qmap):
        self.get_logger().info('[SingleRFPublisher.__qosCallback] Get qmap size: %d' %len(qmap))
        for topic in qmap:
            self.get_logger().info('[SingleRFPublisher.__qosCallback] Get qmap[%s]' %topic)
    
    def checkLong(self, val):
        return (1, val) if (-32768 <= val <= 32767) else (0, 0)
    
    def checkUchar(self, val):
        return (1, val) if (0 <= val <= 255) else (0, 0)
    
    def runRFRecv(self):
        # Enter a loop receiving data on the address specified.
        try:
            while True:
                while self.nrf.data_ready():# As long as data is ready for processing, process it.
                    now = datetime.now()
                    
                    # Read pipe and payload for message.
                    pipe = self.nrf.data_pipe()
                    payload = self.nrf.get_payload()    
                    
                    # Show message received as hex.
                    hex = ':'.join(f'{i:02x}' for i in payload)
                    print(f"{now:%Y-%m-%d %H:%M:%S.%f}: pipe: {pipe}, len: {len(payload)}, bytes: {hex}")

                    if len(payload) == 20 and payload[0] == self.protocol:
                        values = struct.unpack("<BBllllBB", payload)
                        valChk = [0 for i in range(6)]
                        msg = WheelState()
                        valChk[0], msg.gear = self.checkUchar(values[1])
                        valChk[1], msg.steering = self.checkLong(values[2])
                        valChk[2], msg.pedal_throttle = self.checkLong(values[3])
                        valChk[3], msg.pedal_brake = self.checkLong(values[4])
                        valChk[4], msg.pedal_clutch = self.checkLong(values[5])
                        valChk[5], msg.button = self.checkUchar(values[6])
                        chkByte = 0
                        for i in range(6):
                            chkByte = chkByte | (valChk[i] << i)
                        msg.func = chkByte

                        msg.header.priority = msg.header.PRIORITY_CONTROL
                        msg.header.device_type = msg.header.DEVTYPE_RF
                        msg.header.device_id = self.params_.nodeName
                        msg.header.frame_id = self.__frame_id
                        self.__frame_id += 1
                        msg.header.stamp_type = self.getTimestampType()
                        msg.header.stamp = self.getTimestamp().to_msg()
                        msg.header.stamp_offset = self.getCorrectDuration().nanoseconds
                        msg.header.ref_publish_time_ms = 0

                        self.__publisher.publish(msg)
                        self.get_logger().info('Publishing: %03d | %05d %05d %05d %05d | %03d %03d' \
                            %(msg.gear, msg.steering, msg.pedal_throttle, msg.pedal_brake, msg.pedal_clutch, \
                            msg.button, msg.func))
                    
                time.sleep(0.05)
        except:
            traceback.print_exc()
            self.nrf.power_down()
            self.pi.stop()


def main_send(params):
    wsSub = SingleRFSubscriber(params)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(wsSub)
    executorTH = threading.Thread(target=executor.spin, daemon=True)
    executorTH.start()
    executorTH.join()

def main_recv(params):
    wsPub = SingleRFPublisher(params)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(wsPub)
    executorTH = threading.Thread(target=executor.spin, daemon=True)
    executorTH.start()
    executorTH.join()
    
def main():
    # ROS2 implement
    rclpy.init(args=None)
    params = Params('rfcomm_params_node')
    if (params.RF_operationMode == 'send'):
        main_send(params)
    elif (params.RF_operationMode == 'recv'):
        main_recv(params)
    rclpy.shutdown()


#
# A simple NRF24L receiver that connects to a PIGPIO instance on a hostname and port, default "localhost" and 8888, and
# starts receiving data on the address specified.  Use the companion program "simple-sender.py" to send data to it from
# a different Raspberry Pi.
#
if __name__ == "__main__":
    main()