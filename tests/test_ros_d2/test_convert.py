# -*- coding: utf-8 -*-
from ros_d2.helpers.convert import convert
from ros_d2.helpers.get_ros_architecture import NodeInfo, RosArchitecture, TopicInfo


def test_convert():
    ros_architecture = RosArchitecture(
        nodes=[
            NodeInfo(
                name="node1",
                subs=[],
                pubs=[
                    TopicInfo(name="topic1", types=["std_msgs/String"]),
                    TopicInfo(name="topic2", types=["std_msgs/String"]),
                ],
                services=[],
                clients=[],
            ),
            NodeInfo(
                name="node2",
                subs=[
                    TopicInfo(name="topic1", types=["std_msgs/String"]),
                    TopicInfo(name="topic2", types=["std_msgs/String"]),
                ],
                pubs=[],
                services=[],
                clients=[],
            ),
        ],
        topics=[
            TopicInfo(name="topic1", types=["std_msgs/String"]),
            TopicInfo(name="topic2", types=["std_msgs/String"]),
        ],
    )
    d2_graph = convert(ros_architecture)

    assert d2_graph == "\n".join(
        [
            "topic1: {",
            "  shape: class",
            "}",
            "topic2: {",
            "  shape: class",
            "}",
            "node1: {",
            "  style: {",
            "    opacity: 0.6",
            "    3d: true",
            "  }",
            "}",
            "node2: {",
            "  style: {",
            "    opacity: 0.6",
            "    3d: true",
            "  }",
            "}",
            "node1 -> topic1",
            "node1 -> topic2",
            "node2 <- topic1",
            "node2 <- topic2",
        ]
    )
