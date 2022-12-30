from ros_d2.helpers.node_info import NodeInfo
from typing import List

style_node = """: {
  style: {
    opacity: 0.6
    3d: true
  }
}"""


style_topic = """: {
  shape: class
}"""


def convert(node_infos: List[NodeInfo]):
    output = ["direction: right"]
    topics_unique = set()
    for node_info in node_infos:
        for sub in node_info.subs:
            topics_unique.add(sub.topic_name)
        for pub in node_info.pubs:
            topics_unique.add(pub.topic_name)

    # Render each node
    for node_info in node_infos:
        output.append("")
        output.append(node_info.node_name + style_node)
        for sub in node_info.subs:
            output.append(f"{sub.topic_name} -> {node_info.node_name}")
        for pub in node_info.pubs:
            output.append(f"{node_info.node_name} -> {pub.topic_name}")

    # Render all the topics
    for topic in topics_unique:
        output.append(topic+":asffsa"+style_topic)

    return "\n".join(output)