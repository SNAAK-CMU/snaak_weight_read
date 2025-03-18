#!/usr/bin/env python3
import serial
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from snaak_weight_read.srv import ReadWeight


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
        self.serial_port = serial.Serial(serial_port, 9600, timeout=1)

    def read_weight(self):
        serial_data = self.serial_port.readline().decode('utf-8').strip()
        if (serial_data != ''):
            try:
                weight = float(re.findall(r'-?\d+\.?\d*', serial_data)[0])
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


def main(args=None):
    rclpy.init(args=args)
    node = WeightServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
