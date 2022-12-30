# -*- coding: utf-8 -*-
from pathlib import Path

from ros_d2.helpers.convert import convert
from ros_d2.helpers.get_ros_architecture import get_ros_architecture


def export(output_file: Path):
    ros_architecture = get_ros_architecture()
    d2_graph = convert(ros_architecture)
    open(output_file, "w+").write(d2_graph)
