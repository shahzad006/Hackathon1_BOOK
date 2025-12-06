---
sidebar_position: 2
---

# ROS 2 Nodes

In ROS 2, a **node** is the fundamental unit of computation. You can think of a node as a small, single-purpose program within your robot's software ecosystem. For example, you might have one node for controlling the wheel motors, another for reading data from a laser scanner, and a third for planning a path for the robot to follow.

Each node in a ROS 2 system can communicate with other nodes using ROS 2's communication features, such as topics, services, and actions. We'll cover these in the upcoming lessons.

## Creating a ROS 2 Node in Python

Let's create our first ROS 2 node using the `rclpy` (ROS Client Library for Python). This simple node will initialize itself and then shut down.

First, make sure you have a ROS 2 workspace. If you don't, you can create one with the following commands:

```bash
mkdir -p ros2_ws/src
cd ros2_ws
colcon build
```

Now, let's create a Python package for our node:

```bash
cd src
ros2 pkg create --build-type ament_python my_first_node --dependencies rclpy
```

This will create a new directory called `my_first_node` with a few files in it. Now, let's create a file for our node. Inside the `my_first_node/my_first_node` directory, create a new file called `simple_node.py` with the following content:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create a new node
    node = Node('my_simple_node')

    # Print a message to the console
    node.get_logger().info('Hello, ROS 2!')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    # Shutdown the rclpy library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Node

To run this node, you need to add an entry point to your `package.xml` and `setup.py`.

In `setup.py`, add the following to the `entry_points` in the `console_scripts` section:

```python
'console_scripts': [
    'my_simple_node = my_first_node.simple_node:main',
],
```

Now, build your package again:

```bash
cd ~/ros2_ws
colcon build
```

And source the setup file:

```bash
. install/setup.bash
```

Finally, you can run your node with the following command:

```bash
ros2 run my_first_node my_simple_node
```

You should see the "Hello, ROS 2!" message printed to your console.

Congratulations! You have created and run your first ROS 2 node. In the next lessons, we'll see how to make nodes do more interesting things by communicating with each other.
