# -*- coding: utf-8 -*-
from dataclasses import dataclass
from typing import List, Tuple

from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names

TOPICS_TO_IGNORE = [
    "/parameter_events",
    "/rosout",
]


@dataclass
class TopicInfo:
    topic_name: str
    topic_types: List[str]


@dataclass
class NodeInfo:
    node_name: str
    subs: List[TopicInfo]
    pubs: List[TopicInfo]


def filter_topics(topics: List[TopicInfo]) -> List[TopicInfo]:
    return [t for t in topics if t.topic_name not in TOPICS_TO_IGNORE]


def topic_tuple_to_info(topic_tuple: Tuple[str, List[str]]) -> TopicInfo:
    return TopicInfo(topic_name=topic_tuple[0], topic_types=topic_tuple[1])


def get_node_info() -> List[NodeInfo]:
    node_infos: List[NodeInfo] = []
    with NodeStrategy([]) as node:
        node_names = get_node_names(node=node)
        for n in node_names:
            node_name = n.full_name
            subs_tuple = node.get_subscriber_names_and_types_by_node(
                n.name, n.namespace
            )
            pubs_tuple = node.get_publisher_names_and_types_by_node(n.name, n.namespace)
            subs = [topic_tuple_to_info(t) for t in subs_tuple]
            pubs = [topic_tuple_to_info(t) for t in pubs_tuple]

            subs = filter_topics(subs)
            pubs = filter_topics(pubs)

            node_info = NodeInfo(node_name=node_name, subs=subs, pubs=pubs)
            node_infos.append(node_info)

    return node_infos
