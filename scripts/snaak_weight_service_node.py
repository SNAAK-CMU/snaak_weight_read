#!/usr/bin/env python3
import serial
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from snaak_weight_read.srv import ReadWeight
import time

class WeightServiceNode(Node):
    def __init__(self):
        super().__init__("weight_service_node")
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value

        self.weight = 0
        self.weight_history = []
        self.max_history_size = 10

        self.timer = self.create_timer(0.01, self.read_weight)      
        self.srv = self.create_service(ReadWeight, f'snaak_weight_read/{self.get_name()}/read_weight', self.weight_service_callback)  
        self.serial_port = serial.Serial(serial_port, 9600, timeout=1)
        
        self.timer = self.create_timer(0.01, self.read_weight)      
        self.publisher = self.create_publisher(Float64, f'snaak_weight_read/{self.get_name()}/weight', 10)
        self.publisher_timer = self.create_timer(0.5, self.publish_weight)

        while rclpy.ok():
            if self.serial_port.in_waiting > 0: break
            else: self.get_logger().info("Waiting for serial port to be ready...")
            time.sleep(0.2)
        self.serial_port.flushInput()
        self.get_logger().info(f"Weighing Scale Node Started")

    def read_weight(self):
        serial_data = self.serial_port.readline().decode('utf-8').strip()
        if (serial_data != ''):
            try:
                weight = float(re.findall(r'\d+\.?\d*', serial_data)[0])
                if '-' in serial_data:
                    weight = -weight
                self.weight_history.append(weight)
                if len(self.weight_history) > self.max_history_size:
                    self.weight_history.pop(0)

                self.weight = sum(self.weight_history) / len(self.weight_history)
            except ValueError:
                self.get_logger().warn(f"Invalid data received: {serial_data}")

    def weight_service_callback(self, request, response):
        # Ensure response.weight is of type Float64
        response.weight = Float64()
        response.weight.data = self.weight  # Assign the float value to the 'data' field
        return response

    def publish_weight(self):
        msg = Float64()
        msg.data = self.weight
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WeightServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
