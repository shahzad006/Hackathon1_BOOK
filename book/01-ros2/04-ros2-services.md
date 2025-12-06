---
sidebar_position: 4
---

# ROS 2 Services

While topics are great for continuous streams of data, they are not ideal for request/response interactions. For this, ROS 2 provides **services**.

A service is a request/response communication pattern. A node that provides a service is called a **service server**, and a node that requests the service is called a **service client**. The service client sends a request to the service server and waits for a response.

Unlike topics, services are synchronous. The client blocks until it receives a response from the server.

## Creating a Service Server

Let's create a service that adds two integers. First, we need to define the service interface. Create a new directory called `srv` in your `my_first_node` package. Inside `srv`, create a new file called `AddTwoInts.srv` with the following content:

```
int64 a
int64 b
---
int64 sum
```

The three dashes `---` separate the request from the response. `a` and `b` are the request fields, and `sum` is the response field.

Now, we need to tell ROS 2 about our new service definition. Add the following to your `package.xml`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
And make sure you have `find_package(rosidl_default_generators REQUIRED)` in your `CMakeLists.txt` if you are using a C++ package. For python packages, adding the dependencies in `package.xml` is enough for `colcon` to find and build your `.srv` file.

Now, let's create the service server node. Create a new file called `service_server.py` in your `my_first_node` package:

```python
import rclpy
from rclpy.node import Node
from my_first_node.srv import AddTwoInts

class MyServiceServer(Node):

    def __init__(self):
        super().__init__('my_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: [{response.sum}]')
        return response

def main(args=None):
    rclpy.init(args=args)
    my_service_server = MyServiceServer()
    rclpy.spin(my_service_server)
    my_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Service Client

Now, let's create a client that calls our service. Create a new file called `service_client.py`:

```python
import sys
import rclpy
from rclpy.node import Node
from my_first_node.srv import AddTwoInts

class MyServiceClient(Node):

    def __init__(self):
        super().__init__('my_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    my_service_client = MyServiceClient()
    my_service_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(my_service_client)
        if my_service_client.future.done():
            try:
                response = my_service_client.future.result()
            except Exception as e:
                my_service_client.get_logger().info(
                    f'Service call failed {e}')
            else:
                my_service_client.get_logger().info(
                    f'Result of add_two_ints: for {my_service_client.req.a} + {my_service_client.req.b} = {response.sum}')
            break

    my_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Service

Don't forget to add the entry points for your new nodes in `setup.py`:

```python
'console_scripts': [
    'my_simple_node = my_first_node.simple_node:main',
    'my_publisher = my_first_node.publisher:main',
    'my_subscriber = my_first_node.subscriber:main',
    'my_service_server = my_first_node.service_server:main',
    'my_service_client = my_first_node.service_client:main',
],
```

Build your package again, and run the server in one terminal:

```bash
ros2 run my_first_node my_service_server
```

And the client in another, passing two numbers as arguments:

```bash
ros2 run my_first_node my_service_client 2 3
```

You should see the server printing the request it received, and the client printing the response.

```