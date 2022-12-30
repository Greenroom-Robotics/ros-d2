# -*- coding: utf-8 -*-
from typing import List

from py_d2.D2Connection import D2Connection, Direction
from py_d2.D2Diagram import D2Diagram
from py_d2.D2Shape import D2Shape, Shape
from py_d2.D2Style import D2Style

from ros_d2.helpers.node_info import NodeInfo

node_style = D2Style(
    opacity=0.6,
    three_d=True,
)


def convert(node_infos: List[NodeInfo]) -> str:
    """Converts a list of NodeInfo objects to a d2 graph."""
    # Get all the unique topics
    topics_unique = set()
    for node_info in node_infos:
        for sub in node_info.subs:
            topics_unique.add(sub.topic_name)
        for pub in node_info.pubs:
            topics_unique.add(pub.topic_name)

    # Convert to D2
    diagram = D2Diagram()
    for topic in topics_unique:
        shape = D2Shape(name=topic, shape=Shape.classs)
        diagram.add_shape(shape)

    for node_info in node_infos:
        shape = D2Shape(name=node_info.node_name, style=node_style)
        diagram.add_shape(shape)
        for sub in node_info.subs:
            diagram.add_connection(
                D2Connection(
                    shape_1=node_info.node_name,
                    shape_2=sub.topic_name,
                    direction=Direction.TO,
                )
            )
        for pub in node_info.pubs:
            diagram.add_connection(
                D2Connection(
                    shape_1=node_info.node_name,
                    shape_2=pub.topic_name,
                    direction=Direction.FROM,
                )
            )

    return str(diagram)
