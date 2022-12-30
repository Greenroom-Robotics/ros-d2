# -*- coding: utf-8 -*-
from dataclasses import dataclass
from typing import List, Tuple

from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names

TOPICS_TO_IGNORE = [
    "/parameter_events",
    "/rosout",
]

TOPIC_TYPES_TO_IGNORE = [
    "rcl_interfaces/srv/ListParameters",
    "rcl_interfaces/srv/SetParameters",
    "rcl_interfaces/srv/GetParameterTypes",
    "rcl_interfaces/srv/GetParameters",
    "rcl_interfaces/srv/SetParametersAtomically",
    "rcl_interfaces/srv/DescribeParameters",
]


@dataclass
class TopicInfo:
    # Topic name
    name: str
    # Topic types
    types: List[str]


@dataclass
class NodeInfo:
    # Node name
    name: str
    # Node publisher topics
    subs: List[TopicInfo]
    # Node subscriber topics
    pubs: List[TopicInfo]
    # Node services
    services: List[TopicInfo]
    # Node service clients
    clients: List[TopicInfo]


@dataclass
class RosArchitecture:
    nodes: List[NodeInfo]
    topics: List[TopicInfo]


def filter_topics(topics: List[TopicInfo]) -> List[TopicInfo]:
    return [
        t
        for t in topics
        if t.name not in TOPICS_TO_IGNORE and t.types[0] not in TOPIC_TYPES_TO_IGNORE
    ]


def topic_tuple_to_info(topic_tuple: Tuple[str, List[str]]) -> TopicInfo:
    return TopicInfo(name=topic_tuple[0], types=topic_tuple[1])


def get_ros_architecture() -> RosArchitecture:

    # Get all the nodes and their oublisher and subscriber topics
    nodes: List[NodeInfo] = []
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
            services_tuple = node.get_service_names_and_types_by_node(
                n.name, n.namespace
            )
            services = [topic_tuple_to_info(t) for t in services_tuple]
            clients_tuple = node.get_client_names_and_types_by_node(n.name, n.namespace)
            clients = [topic_tuple_to_info(t) for t in clients_tuple]

            subs = filter_topics(subs)
            pubs = filter_topics(pubs)
            services = filter_topics(services)
            clients = filter_topics(clients)

            node_info = NodeInfo(
                name=node_name, subs=subs, pubs=pubs, services=services, clients=clients
            )
            nodes.append(node_info)

    # Get all the unique topics
    topic_names = set()
    topics: List[TopicInfo] = []
    for node_info in nodes:
        for sub in node_info.subs:
            if sub.name not in topic_names:
                topic_names.add(sub.name)
                topics.append(sub)
        for pub in node_info.pubs:
            if pub.name not in topic_names:
                topic_names.add(pub.name)
                topics.append(pub)
        for srv in node_info.services:
            if srv.name not in topic_names:
                topic_names.add(srv.name)
                topics.append(srv)

    # Order them alphabetically
    topics.sort(key=lambda t: t.name)
    nodes.sort(key=lambda n: n.name)

    return RosArchitecture(
        topics=topics,
        nodes=nodes,
    )
