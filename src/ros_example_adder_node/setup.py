from setuptools import find_packages
from setuptools import setup


package_name = "example_adder_node"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="David Revay",
    maintainer_email="david.revay@greenroomrobotics.com",
    description="A generic node for stubbing std sensors types",
    license="Copyright (c) 2021 Greenroom Robotics Pty Ltd",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "example_adder_node = example_adder_node.example_adder_node:main",
        ],
    },
)
