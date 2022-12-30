# -*- coding: utf-8 -*-
import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription(
        [
            Node(
                name="adder_1",
                package="example_adder_node",
                executable="example_adder_node",
                remappings=[
                    ("input_1", "adder_3/output"),
                    ("input_2", "adder_3/output"),
                    ("output", "adder_1/output"),
                ],
            ),
            Node(
                name="adder_2",
                package="example_adder_node",
                executable="example_adder_node",
                remappings=[
                    ("input_1", "input_1"),
                    ("input_2", "input_2"),
                    ("output", "adder_2/output"),
                ],
            ),
            Node(
                name="adder_3",
                package="example_adder_node",
                executable="example_adder_node",
                remappings=[
                    ("input_1", "adder_1/output"),
                    ("input_2", "adder_2/output"),
                    ("output", "adder_3/output"),
                ],
            ),
        ]
    )
