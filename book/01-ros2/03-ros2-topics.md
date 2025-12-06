---
sidebar_position: 3
---

# ROS 2 Topics

Topics are one of the main ways that nodes communicate in ROS 2. A **topic** is like a named bus that nodes can use to send and receive data. Nodes that send data to a topic are called **publishers**, and nodes that receive data from a topic are called **subscribers**.

One of the key features of topics is that they are anonymous. This means that a publisher doesn't know which nodes are subscribing to the topic, and a subscriber doesn't know which nodes are publishing to it. This decouples the nodes from each other, making the system more modular and scalable.

## Creating a Publisher

Let's create a node that publishes a simple string message to a topic. In your `my_first_node` package, create a new file called `publisher.py` with the following content:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):

    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS 2! {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This node creates a publisher for a topic named `my_topic`. Every half a second, the `timer_callback` function is called, which creates a new string message, publishes it to the topic, and logs it to the console.

## Creating a Subscriber

Now, let's create a node that subscribes to the `my_topic` topic and prints the received messages to the console. Create a new file called `subscriber.py` in the same directory:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This node creates a subscriber for the `my_topic` topic. Whenever a message is published to the topic, the `listener_callback` function is called, which simply logs the message to the console.

## Running the Publisher and Subscriber

Don't forget to add the entry points for your new nodes in `setup.py`:

```python
'console_scripts': [
    'my_simple_node = my_first_node.simple_node:main',
    'my_publisher = my_first_node.publisher:main',
    'my_subscriber = my_first_node.subscriber:main',
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

Finally, open two new terminals. In the first one, run the publisher:

```bash
ros2 run my_first_node my_publisher
```

In the second one, run the subscriber:

```bash
ros2 run my_first_node my_subscriber
```

You should see the publisher printing the messages it's sending, and the subscriber printing the messages it's receiving.

This is the basic mechanism of communication in ROS 2. In the next lessons, we'll explore other communication patterns.
