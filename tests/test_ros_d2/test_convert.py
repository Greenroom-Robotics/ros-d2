from ros_d2.helpers.convert import convert
from ros_d2.helpers.node_info import NodeInfo, TopicInfo

def test_convert():
    node_infos = [
        NodeInfo(
            node_name="node1",
            subs=[],
            pubs=[
                TopicInfo(topic_name="topic1", topic_types=["std_msgs/String"]),
                TopicInfo(topic_name="topic2", topic_types=["std_msgs/String"]),
            ],
        ),
        NodeInfo(
            node_name="node2",
            subs=[
                TopicInfo(topic_name="topic1", topic_types=["std_msgs/String"]),
                TopicInfo(topic_name="topic2", topic_types=["std_msgs/String"]),
            ],
            pubs=[],
        ),
    ]
    d2_graph = convert(node_infos)

    assert d2_graph == ""

    