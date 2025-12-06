---
sidebar_position: 5
---

# ROS 2 Actions

Actions are another communication pattern in ROS 2, designed for long-running tasks. They are similar to services, but with a few key differences:

-   **Asynchronous**: Unlike services, actions are non-blocking. A client can send a goal to an action server and continue with other tasks.
-   **Feedback**: The action server can provide regular feedback to the client about the progress of the task.
-   **Preemptible**: The client can cancel a goal that is being executed by the action server.

A good example of a use case for an action is a navigation task. The client can send a goal to the navigation action server (e.g., "go to position X, Y"), and the server will start moving the robot. While the robot is moving, the server can send feedback to the client (e.g., the robot's current position). The client can also send a cancel request to the server to stop the robot.

## Creating an Action Server

Let's create an action that simulates a Fibonacci sequence calculation. First, we need to define the action interface. Create a new directory called `action` in your `my_first_node` package. Inside `action`, create a new file called `Fibonacci.action` with the following content:

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

The three sections, separated by `---`, are the goal, the result, and the feedback.

-   **Goal**: `order` is the order of the Fibonacci sequence to compute.
-   **Result**: `sequence` is the final Fibonacci sequence.
-   **Feedback**: `partial_sequence` is the sequence as it's being computed.

Now, add the action to your `package.xml` so that ROS 2 knows about it:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Now, let's create the action server node. Create a new file called `action_server.py`:

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from my_first_node.action import Fibonacci

class MyActionServer(Node):

    def __init__(self):
        super().__init__('my_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    my_action_server = MyActionServer()
    rclpy.spin(my_action_server)

if __name__ == '__main__':
    main()
```

## Creating an Action Client

Now, let's create a client that calls our action. Create a new file called `action_client.py`:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_first_node.action import Fibonacci

class MyActionClient(Node):

    def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.partial_sequence}')

def main(args=None):
    rclpy.init(args=args)
    my_action_client = MyActionClient()
    my_action_client.send_goal(10)
    rclpy.spin(my_action_client)

if __name__ == '__main__':
    main()
```

## Running the Action

Add the entry points to `setup.py`:

```python
'console_scripts': [
    'my_simple_node = my_first_node.simple_node:main',
    'my_publisher = my_first_node.publisher:main',
    'my_subscriber = my_first_node.subscriber:main',
    'my_service_server = my_first_node.service_server:main',
    'my_service_client = my_first_node.service_client:main',
    'my_action_server = my_first_node.action_server:main',
    'my_action_client = my_first_node.action_client:main',
],
```

Build, source, and run the server and client in separate terminals. You'll see the server providing feedback as it computes the sequence, and the client receiving it.
