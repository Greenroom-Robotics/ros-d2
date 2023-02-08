# ros_d2

![Banner](docs/images/banner.png)

Exports ROS2 nodes (publishers, subcribers and services) into a [D2](https://d2lang.com/tour/intro/) file so they can be visualised and serialised for later use.

## Features

- [x] Export runtime ROS2 system architecture (nodes, services, topics, clients) to .d2 (using `ros2cli`)
- [x] Powerful visualisation and theming provided by [D2](https://d2lang.com/tour/intro/)
- [x] Simple serialisable diagram format
- [ ] Ros node descriptions pulled from the package.xml
- [ ] Topic QOS descriptions
- [ ] Action servers / clients?
- [x] Topic Types (`--verbose` flag)
- [ ] Advanced theming
- [ ] Pytest helpers that ensure your ROS2 System architecture matches that of your .d2 diagram
- [ ] Better testing on this repo!
- [x] `ros_d2 export-launch-files` command that will export a .d2 file for each of your launch files
- [x] `ros_d2 render` command that will render all .d2 files in the current directory


## Examples

### Simple

![Simple example](example_output/ros-diagram-dagre-verbose.svg)


### More complex applications
![Example](example_output/ros-complex-example-dagre.svg)

## Installation

```bash
pip install ros-d2
ros_d2 export --help # prints help
# If you get an error about "No module named 'ros2cli'" - ros has not been sourced!
```

## Requirements

- ROS2 + Colcon
- [D2](https://github.com/terrastruct/d2#install)

## Usage

### Exporting a diagram
1. Launch your ROS2 system
2. Source ROS
3. Use `ros_d2` to export your runtime ros system to a .d2 file

  ```bash
  ros_d2 export ros-diagram.d2
  ```

4. Render your diagram using `d2`.

  ```bash
  ros_d2 render # to render ALL .d2 files in the current directory
  # or...
  d2 --layout dagre ros-diagram.d2 # to use d2 directly
  ```

### Exporting diagrams for all launch files
If you want to create a `.d2` and `.svg` alongside each of your ros2 `.launch.py` files. You can use the following script:

  ```bash
  # Source ros: eg) `source /opt/ros/humble/setup.bash`
  # Build your ros2 workspace: eg) `colcon build`
  # Source your ros2 workspace: eg) `source install/setup.bash`
  ros_d2 export-launch-files
  ros_d2 render
  ```


## Development
### Prerequisite

- [Python 3.7+](https://www.python.org/)
- [pre-commit](https://pre-commit.com/)

### Installation

following the steps below to setup the project:

```bash

```bash
# Clone the repository
git clone git@github.com:Greenroom-Robotics/ros-d2.git && cd ros-d2

# Install all dependencies
pip install -e ./src/ros_d2
```

### Example usage

From the root of this repo:

1. Build this project using colcon
  ```bash
  source /opt/ros/galactic/setup.bash
  colcon build

  ros_d2 --help # prints help
  ```

2. Let's bring up the example nodes:

  ```bash
  source install/setup.sh
  ros2 launch ./src/ros_example_adder_node/launch/example_adder_node.launch.py
  ```

3. Now that ROS is running, lets use `ros_d2` to generate a diagram of all the nodes in the system:

  ```bash
  ros_d2 export example_output/ros-diagram.d2
  ```

4. We will now have the [example_output/ros-diagram.d2](example_output/ros-diagram.d2) file. We can render this using `d2`:

  ```bash
  d2 --layout dagre example_output/ros-diagram.d2 example_output/ros-diagram-dagre.svg
  # or
  d2 --layout elk example_output/ros-diagram.d2 example_output/ros-diagram-dagre.svg
  # Note - d2 provides other layouts and themes. See `d2 --help` for more info
  ```

  This will produce the following diagram:

  Dagre:
  ![dagre](example_output/ros-diagram-dagre.svg)

  Elk:
  ![elk](example_output/ros-diagram-elk.svg)

5. We can also export with more info using `ros_d2 export example_output/ros-diagram.d2`

![dagre](example_output/ros-diagram-dagre-verbose.svg)
