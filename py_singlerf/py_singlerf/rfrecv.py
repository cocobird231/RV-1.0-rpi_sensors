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

address = '1SNSR'
protocol = 0x01

class WheelStatePublisher(Node):
    def __init__(self, nodeName, topicName):
        super().__init__(nodeName)
        global address
        self.nodeName_ = nodeName
        self.publisher_ = self.create_publisher(WheelState, topicName, 10)

        # Connect to pigpiod
        print(f'Connecting to GPIO daemon on localhost:8888 ...')
        self.pi = pigpio.pi('localhost', 8888)
        if not self.pi.connected:
            print("Not connected to Raspberry Pi ... goodbye.")
            sys.exit()
        
        # Create NRF24 object.
        self.nrf = NRF24(self.pi, ce=25, payload_size=RF24_PAYLOAD.DYNAMIC, channel=100, data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.HIGH)
        self.nrf.set_address_bytes(len(address))
        self.nrf.open_reading_pipe(RF24_RX_ADDR.P1, address)
        self.nrf.show_registers()
        
        self.runRFRecv()
    
    def checkLong(self, val):
        return (1, val) if (-32768 <= val <= 32767) else (0, 0)
    
    def checkUchar(self, val):
        return (1, val) if (0 <= val <= 255) else (0, 0)
    

    def runRFRecv(self):
        global protocol
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

                    if len(payload) == 20 and payload[0] == protocol:
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
                        msg.header.device_id = self.nodeName_
                        msg.header.stamp_type = msg.header.STAMPTYPE_NO_SYNC
                        msg.header.stamp = self.get_clock().now().to_msg()

                        self.publisher_.publish(msg)
                        self.get_logger().info('Publishing: %03d | %05d %05d %05d %05d | %03d %03d' \
                            %(msg.gear, msg.steering, msg.pedal_throttle, msg.pedal_brake, msg.pedal_clutch, \
                            msg.button, msg.func))
                    
                time.sleep(0.05)
        except:
            traceback.print_exc()
            self.nrf.power_down()
            self.pi.stop()


def main():
    # ROS2 implement
    rclpy.init(args=None)
    wsPub = WheelStatePublisher('RF_wheelstate_publisher', "topic_RFWheelState")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(wsPub)
    executorTH = threading.Thread(target=executor.spin, daemon=True)
    executorTH.start()

    executorTH.join()
    rclpy.shutdown()


#
# A simple NRF24L receiver that connects to a PIGPIO instance on a hostname and port, default "localhost" and 8888, and
# starts receiving data on the address specified.  Use the companion program "simple-sender.py" to send data to it from
# a different Raspberry Pi.
#
if __name__ == "__main__":
    main()