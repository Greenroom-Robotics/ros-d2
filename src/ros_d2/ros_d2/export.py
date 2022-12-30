# -*- coding: utf-8 -*-
from pathlib import Path

from ros_d2.helpers.convert import convert
from ros_d2.helpers.node_info import get_node_info


def export(output_file: Path):
    node_infos = get_node_info()
    d2_graph = convert(node_infos)
    open(output_file, "w+").write(d2_graph)
