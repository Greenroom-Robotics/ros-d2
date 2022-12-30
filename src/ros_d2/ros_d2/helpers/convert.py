# -*- coding: utf-8 -*-
from py_d2.D2Connection import D2Connection, Direction
from py_d2.D2Diagram import D2Diagram
from py_d2.D2Shape import D2Shape, Shape
from py_d2.D2Style import D2Style

from ros_d2.helpers.get_ros_architecture import RosArchitecture

node_style = D2Style(
    opacity=0.6,
    three_d=True,
)


def convert(ros_architecture: RosArchitecture) -> str:
    """Converts a list of NodeInfo objects to a d2 graph."""
    diagram = D2Diagram()

    for topic in ros_architecture.topics:
        shape = D2Shape(name=topic.name, shape=Shape.classs)
        diagram.add_shape(shape)

    for node_info in ros_architecture.nodes:
        shape = D2Shape(name=node_info.name, style=node_style)
        diagram.add_shape(shape)
        for sub in node_info.subs:
            diagram.add_connection(
                D2Connection(
                    shape_1=node_info.name,
                    shape_2=sub.name,
                    direction=Direction.TO,
                )
            )
        for pub in node_info.pubs:
            diagram.add_connection(
                D2Connection(
                    shape_1=node_info.name,
                    shape_2=pub.name,
                    direction=Direction.FROM,
                )
            )

    return str(diagram)
