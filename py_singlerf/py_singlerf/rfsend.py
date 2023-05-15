# Code referenced from: https://github.com/bjarne-hansen/py-nrf24
import struct
import sys
import threading

import pigpio
from nrf24 import *

import rclpy
from rclpy.node import Node

from vehicle_interfaces.msg import WheelState

address = '1SNSR'
protocol = 0x01

class WheelStateSubscriber(Node):
    def __init__(self, nodeName, topicName):
        super().__init__(nodeName)
        global address
        self.subscription = self.create_subscription(
            WheelState,
            topicName,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Connect to pigpiod
        print(f'Connecting to GPIO daemon on localhost:8888 ...')
        self.pi = pigpio.pi('localhost', 8888)
        if not self.pi.connected:
            print("Not connected to Raspberry Pi ... goodbye.")
            sys.exit()

        # Create NRF24 object.
        self.nrf = NRF24(self.pi, ce=25, payload_size=RF24_PAYLOAD.DYNAMIC, channel=100, data_rate=RF24_DATA_RATE.RATE_250KBPS, pa_level=RF24_PA.HIGH)
        self.nrf.set_address_bytes(len(address))
        self.nrf.open_writing_pipe(address)
    
        # Display the content of NRF24L01 device registers.
        self.nrf.show_registers()

    def listener_callback(self, msg):
        global protocol
        payload = struct.pack("<BBllllBB", protocol, int(msg.gear), int(msg.steering), int(msg.pedal_throttle), int(msg.pedal_brake), int(msg.pedal_clutch), \
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


def main(args=None):
    # ROS2 implement
    rclpy.init(args=None)
    wsSub = WheelStateSubscriber('RF_wheelstate_subscriber', "topic_RFWheelState")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(wsSub)
    executorTH = threading.Thread(target=executor.spin, daemon=True)
    executorTH.start()

    executorTH.join()
    rclpy.shutdown()


#
# A simple NRF24L sender that connects to a PIGPIO instance on a hostname and port, default "localhost" and 8888, and
# starts sending data on the address specified.  Use the companion program "simple-receiver.py" to receive the data
# from it on a different Raspberry Pi.
#
if __name__ == "__main__":
    main()