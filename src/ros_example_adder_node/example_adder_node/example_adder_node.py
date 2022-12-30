# -*- coding: utf-8 -*-
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from example_interfaces.msg import Bool
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node

SUBSCRIBER1_TOPIC = "/input_1"
SUBSCRIBER2_TOPIC = "/input_2"
PUBLISHER_TOPIC = "/output"
SERVICE_TOPIC = "/add"


class ExampleAdderNode(Node):
    def __init__(self):
        super().__init__("example_adder_node")  # type: ignore
        self.get_logger().info("Node started")

        self.subscriber1_ = self.create_subscription(
            Bool, SUBSCRIBER1_TOPIC, self.on_sub_1, 1
        )
        self.subscriber2_ = self.create_subscription(
            Bool, SUBSCRIBER2_TOPIC, self.on_sub_2, 1
        )
        self.publisher_ = self.create_publisher(Bool, PUBLISHER_TOPIC, 1)
        self.service = self.create_service(
            AddTwoInts, SERVICE_TOPIC, self.on_add_numbers
        )

    sub_1_value = False
    sub_2_value = False

    def on_sub_1(self, msg: Bool):
        self.sub_1_value = msg.data
        self.get_logger().info(f"Sub 1: {msg.data}")
        self.publish_result()

    def on_sub_2(self, msg: Bool):
        self.sub_2_value = msg.data
        self.get_logger().info(f"Sub 2: {msg.data}")
        self.publish_result()

    def on_add_numbers(
        self, request: AddTwoInts.Request, response: AddTwoInts.Response
    ):
        response.sum = request.a + request.b
        return response

    def publish_result(self):
        msg = Bool()
        msg.data = self.sub_1_value and self.sub_2_value
        self.get_logger().info(f"Publishing result: {msg.data}")
        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    example_adder_node = ExampleAdderNode()
    rclpy.spin(example_adder_node)
    example_adder_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
