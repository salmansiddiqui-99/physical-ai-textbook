---
id: ch1-ros2-basics
title: Chapter 1.1 - ROS 2 Fundamentals
sidebar_label: ROS 2 Fundamentals
sidebar_position: 1
description: Learn ROS 2 architecture, nodes, topics, services, and build your first ROS 2 application with Python.
keywords:
  - ROS 2
  - rclpy
  - nodes
  - topics
  - publisher
  - subscriber
  - services
prerequisites:
  - Python 3 programming (functions, classes, basic OOP)
  - Linux terminal basics (bash commands, file navigation)
  - ROS 2 Humble or Iron installed
learning_objectives:
  - Create and run ROS 2 nodes using rclpy
  - Implement publishers and subscribers for inter-node communication
  - Design service-based request-response patterns
  - Debug ROS 2 applications using command-line tools
estimated_time: 90 minutes
---

# Chapter 1.1 - ROS 2 Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:

- Create ROS 2 nodes that run as independent processes in a robotics system
- Implement publishers and subscribers to exchange data between nodes using topics
- Design and call ROS 2 services for request-response communication patterns
- Debug and inspect running ROS 2 systems using ros2 CLI tools

## Prerequisites

Before starting this chapter, you should:

- Have Python 3 programming experience (functions, classes, object-oriented programming)
- Be comfortable with Linux terminal commands (cd, ls, source, running scripts)
- Have ROS 2 Humble or Iron installed on Ubuntu 22.04 (see [installation guide](../intro.md))

## Introduction

ROS 2 (Robot Operating System 2) is the de facto middleware standard for modern robotics. From warehouse robots to humanoid platforms like Boston Dynamics' Atlas and Tesla's Optimus, ROS 2 provides the communication infrastructure that allows sensors, actuators, and control algorithms to work together seamlessly.

In this chapter, you'll learn the core building blocks of ROS 2: **nodes**, **topics**, and **services**. These abstractions allow you to decompose complex robotic systems into modular, reusable components that communicate via well-defined interfaces. This architectural pattern is critical for humanoid robotics, where separate subsystems (vision, locomotion, manipulation) must coordinate in real-time.

By the end of this chapter, you'll build a simple publisher-subscriber system and a service-based calculator—foundational patterns you'll use throughout this course when controlling humanoid robots.

---

## Section 1: ROS 2 Architecture and Core Concepts

ROS 2 represents a complete redesign of the original ROS, built on top of the Data Distribution Service (DDS) standard for real-time systems. Understanding its architecture is essential for building robust robotic applications.

### Subsection 1.1: The ROS 2 Graph

At its core, ROS 2 organizes computation into a **graph** of processes called **nodes**. Each node is an independent executable that performs a specific task:

- A **camera node** might publish image data
- A **perception node** might subscribe to images and detect objects
- A **control node** might send motor commands to actuators

Nodes communicate through three primary mechanisms:

1. **Topics** (publish-subscribe): Streaming data like sensor readings
2. **Services** (request-response): Synchronous operations like "get robot pose"
3. **Actions** (goal-based): Long-running tasks like "navigate to waypoint" (covered in Chapter 1.2)

### Subsection 1.2: DDS Middleware Layer

Unlike ROS 1 (which used a centralized `roscore` master), ROS 2 uses DDS for **peer-to-peer discovery**. This means:

- Nodes automatically discover each other on the network
- No single point of failure (no master node required)
- Support for Quality of Service (QoS) policies for real-time systems

**Key Terminology**:
- **Node**: An independent process in the ROS 2 computational graph
- **Topic**: A named bus over which nodes exchange messages using publish-subscribe
- **Message**: A data structure (e.g., `geometry_msgs/Twist` for velocity commands)
- **Service**: A synchronous request-response communication pattern
- **Package**: A container for organizing nodes, launch files, and resources

---

## Section 2: Creating Your First ROS 2 Node

Let's build a simple publisher node that sends string messages. This demonstrates the most common communication pattern in ROS 2.

### Example: String Publisher Node

**Purpose**: Publish periodic messages to a topic, demonstrating the basic structure of a ROS 2 node in Python.

**Environment**: ROS 2 Humble (Python 3.10)

**Dependencies**:

```bash
# Ensure ROS 2 Humble is installed
source /opt/ros/humble/setup.bash

# Install Python dependencies (should be included with ROS 2)
sudo apt update
sudo apt install python3-rclpy
```

**Code**:

```python
#!/usr/bin/env python3
"""
Simple ROS 2 Publisher Node

Purpose: Publish string messages to /chatter topic at 2 Hz
Inputs: None
Outputs: std_msgs/String messages on /chatter
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal publisher that sends periodic string messages.

    This node demonstrates:
    - Node initialization and inheritance
    - Timer-based callbacks
    - Topic publishing with std_msgs
    """

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create publisher: topic name, message type, queue size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer: period in seconds, callback function
        timer_period = 0.5  # 2 Hz (0.5 seconds between messages)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0  # Message counter
        self.get_logger().info('MinimalPublisher node started')

    def timer_callback(self):
        """
        Called every 0.5 seconds by the timer.
        Publishes a string message with an incrementing counter.
        """
        msg = String()
        msg.data = f'Hello, ROS 2! Message count: {self.i}'
        self.publisher_.publish(msg)

        # Log to console (viewable with ros2 topic echo)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()

    try:
        rclpy.spin(node)  # Keep node alive until Ctrl+C
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Code Explanation**:

Let's break down this example step-by-step:

1. **Lines 10-11**: We import `rclpy` (ROS Client Library for Python) and the base `Node` class. All ROS 2 nodes inherit from `Node`.

2. **Line 15**: The `MinimalPublisher` class inherits from `Node` and calls `super().__init__('minimal_publisher')` to register itself in the ROS 2 graph with the name "minimal_publisher".

3. **Lines 29-30**: `create_publisher()` sets up a publisher for the `/chatter` topic using the `String` message type. The queue size (10) determines how many messages to buffer if publishing faster than subscribers can receive.

4. **Lines 33-34**: `create_timer()` creates a repeating timer that calls `timer_callback()` every 0.5 seconds. This is the recommended pattern for periodic tasks in ROS 2.

5. **Lines 39-48**: The `timer_callback()` method constructs a `String` message, assigns data to it, and publishes it. `get_logger().info()` prints to the console with automatic timestamping.

6. **Lines 51-60**: The `main()` function initializes the ROS 2 client library, creates the node, and calls `spin()` to keep the node running until interrupted.

**Running the Code**:

```bash
# Step 1: Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Step 2: Make the script executable
chmod +x minimal_publisher.py

# Step 3: Run the node
python3 minimal_publisher.py

# Step 4 (in a new terminal): Verify the topic exists
source /opt/ros/humble/setup.bash
ros2 topic list

# Step 5: Echo the messages
ros2 topic echo /chatter
```

**Expected Output**:

Terminal 1 (publisher):
```
[INFO] [minimal_publisher]: MinimalPublisher node started
[INFO] [minimal_publisher]: Publishing: "Hello, ROS 2! Message count: 0"
[INFO] [minimal_publisher]: Publishing: "Hello, ROS 2! Message count: 1"
[INFO] [minimal_publisher]: Publishing: "Hello, ROS 2! Message count: 2"
...
```

Terminal 2 (subscriber):
```
data: Hello, ROS 2! Message count: 5
---
data: Hello, ROS 2! Message count: 6
---
...
```

**Troubleshooting**:

| Issue | Solution |
|-------|----------|
| `ModuleNotFoundError: No module named 'rclpy'` | Source ROS 2 setup: `source /opt/ros/humble/setup.bash` |
| "No executable found" error | Make script executable: `chmod +x minimal_publisher.py` |
| Topic not visible with `ros2 topic list` | Ensure publisher is running; check for errors in terminal |
| Messages not appearing with `ros2 topic echo` | Verify topic name matches exactly (`/chatter`), check QoS compatibility |

---

## Section 3: Subscribing to Topics

Publishers are only half the story. Now let's create a **subscriber** that receives messages from the `/chatter` topic.

### Example: String Subscriber Node

**Purpose**: Subscribe to `/chatter` and print received messages.

**Code**:

```python
#!/usr/bin/env python3
"""
Simple ROS 2 Subscriber Node

Purpose: Subscribe to /chatter topic and print received messages
Inputs: std_msgs/String messages on /chatter
Outputs: Console logging of received data
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal subscriber that receives string messages.

    Demonstrates:
    - Topic subscription
    - Callback-based message processing
    """

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create subscription: topic, message type, callback, queue size
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('MinimalSubscriber node started')

    def listener_callback(self, msg):
        """
        Called automatically when a message arrives on /chatter.

        Args:
            msg (std_msgs.msg.String): The received message
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Code Explanation**:

1. **Lines 28-33**: `create_subscription()` registers a callback function (`listener_callback`) that's invoked whenever a message arrives on `/chatter`.

2. **Lines 36-44**: The `listener_callback` method receives the message as a typed parameter (`msg` of type `String`). We simply log the data field.

3. **Message Flow**: When the publisher calls `publish(msg)`, DDS routes the message to all subscribers. The subscriber's callback is invoked on the node's thread managed by `spin()`.

**Running Both Nodes**:

```bash
# Terminal 1: Start publisher
source /opt/ros/humble/setup.bash
python3 minimal_publisher.py

# Terminal 2: Start subscriber
source /opt/ros/humble/setup.bash
python3 minimal_subscriber.py
```

You should see the subscriber printing the same messages published by the publisher, demonstrating ROS 2's publish-subscribe communication.

---

## Section 4: Services for Request-Response Communication

Topics are ideal for streaming data, but sometimes you need **synchronous request-response** behavior—like asking a service "What is 5 + 3?" and waiting for the answer. ROS 2 services provide this capability.

### Example: Calculator Service

**Purpose**: Create a service that adds two integers and returns the result.

**Service Definition**:

ROS 2 includes the `example_interfaces` package with common service types. We'll use `AddTwoInts.srv`:

```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

**Service Server Code**:

```python
#!/usr/bin/env python3
"""
ROS 2 Service Server: Add Two Integers

Purpose: Provide an add_two_ints service that sums two integers
Inputs: AddTwoInts.srv requests (two int64 values)
Outputs: AddTwoInts.srv responses (sum as int64)
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AdditionService(Node):
    """
    Service server that adds two integers.

    Service: /add_two_ints (example_interfaces/AddTwoInts)
    """

    def __init__(self):
        super().__init__('addition_service')

        # Create service: service name, service type, callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Addition service ready at /add_two_ints')

    def add_two_ints_callback(self, request, response):
        """
        Service callback that processes addition requests.

        Args:
            request (AddTwoInts.Request): Contains a and b integers
            response (AddTwoInts.Response): Object to populate with sum

        Returns:
            AddTwoInts.Response: Populated response object
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AdditionService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Service Client Code**:

```python
#!/usr/bin/env python3
"""
ROS 2 Service Client: Add Two Integers

Purpose: Call the /add_two_ints service with user-provided values
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AdditionClient(Node):
    """Service client that calls /add_two_ints service."""

    def __init__(self):
        super().__init__('addition_client')

        # Create client: service name, service type
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """Send addition request and wait for response."""
        self.req.a = a
        self.req.b = b

        # Call service asynchronously
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: addition_client.py <int> <int>')
        return

    node = AdditionClient()

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    response = node.send_request(a, b)
    node.get_logger().info(f'Result: {a} + {b} = {response.sum}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Running the Service**:

```bash
# Terminal 1: Start service server
source /opt/ros/humble/setup.bash
python3 addition_service.py

# Terminal 2: Call service from client
source /opt/ros/humble/setup.bash
python3 addition_client.py 5 8

# Alternative: Call service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 15}"
```

**Expected Output**:

Server terminal:
```
[INFO] [addition_service]: Addition service ready at /add_two_ints
[INFO] [addition_service]: Request: 5 + 8 = 13
[INFO] [addition_service]: Request: 10 + 15 = 25
```

Client terminal:
```
[INFO] [addition_client]: Result: 5 + 8 = 13
```

---

## Hands-On Project: Multi-Node Communication System

**Goal**: Build a three-node system where a sensor simulator publishes data, a processor node transforms it, and a monitor node logs the results.

**Duration**: 30 minutes

**What You'll Learn**:
- Chaining topics for data pipelines
- Running multiple nodes simultaneously
- Using `ros2 node list` and `ros2 topic info` for debugging

### Step 1: Create Sensor Simulator

Create `sensor_simulator.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')
        self.publisher_ = self.create_publisher(Float32, 'raw_sensor_data', 10)
        self.timer = self.create_timer(1.0, self.publish_sensor_data)
        self.get_logger().info('Sensor simulator started')

    def publish_sensor_data(self):
        msg = Float32()
        msg.data = random.uniform(20.0, 30.0)  # Simulate temperature sensor
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published raw sensor: {msg.data:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 2: Create Data Processor

Create `data_processor.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')

        # Subscribe to raw data
        self.subscription = self.create_subscription(
            Float32,
            'raw_sensor_data',
            self.process_data,
            10
        )

        # Publish processed data
        self.publisher_ = self.create_publisher(Float32, 'processed_data', 10)
        self.get_logger().info('Data processor started')

    def process_data(self, msg):
        # Convert Celsius to Fahrenheit
        fahrenheit = (msg.data * 9/5) + 32

        processed_msg = Float32()
        processed_msg.data = fahrenheit
        self.publisher_.publish(processed_msg)

        self.get_logger().info(f'Processed: {msg.data:.2f}°C -> {fahrenheit:.2f}°F')


def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Create Monitor Node

Create `monitor.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Monitor(Node):
    def __init__(self):
        super().__init__('monitor')
        self.subscription = self.create_subscription(
            Float32,
            'processed_data',
            self.monitor_callback,
            10
        )
        self.get_logger().info('Monitor started')

    def monitor_callback(self, msg):
        if msg.data > 86.0:  # Alert if over 86°F (30°C)
            self.get_logger().warn(f'HIGH TEMPERATURE: {msg.data:.2f}°F')
        else:
            self.get_logger().info(f'Temperature OK: {msg.data:.2f}°F')


def main(args=None):
    rclpy.init(args=args)
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Run All Nodes

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
python3 sensor_simulator.py

# Terminal 2
source /opt/ros/humble/setup.bash
python3 data_processor.py

# Terminal 3
source /opt/ros/humble/setup.bash
python3 monitor.py

# Terminal 4: Inspect the system
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list
ros2 topic hz /raw_sensor_data  # Check publishing rate
```

**Checkpoint**: You should see data flowing from sensor → processor → monitor, with temperature warnings appearing when values exceed 86°F.

---

## Challenge: Test Your Understanding

Try these exercises to reinforce your learning:

1. **Basic**: Modify the sensor simulator to publish random humidity values (0-100%) instead of temperature.
   - *Hint*: Change the random range and update the log message.

2. **Intermediate**: Add a second subscriber to `data_processor.py` that listens to a `/calibration_offset` topic and adds this offset to all sensor readings.
   - *Hint*: Create a second subscription with a callback that stores the offset in an instance variable.

3. **Advanced**: Implement a service in the monitor node that returns statistics (min, max, average) of the last 10 temperature readings.
   - *Hint*: Use `collections.deque` to maintain a sliding window of values, and create a custom `.srv` file or use `std_srvs/Trigger`.

**Solutions**: Validate your work by checking that `ros2 topic echo` shows expected outputs and services respond correctly.

---

## Summary

In this chapter, you learned:

- **ROS 2 architecture**: Nodes communicate via topics (pub-sub) and services (request-response) using DDS middleware
- **Publishers and subscribers**: How to stream data between nodes using `create_publisher()` and `create_subscription()`
- **Services**: Synchronous request-response patterns using `create_service()` and `create_client()`
- **Debugging tools**: `ros2 topic list/echo`, `ros2 node list`, and `ros2 service call` for inspecting running systems

**Key Commands**:

```bash
ros2 topic list              # List all active topics
ros2 topic echo /topic_name  # Print messages from a topic
ros2 node list               # List all running nodes
ros2 service list            # List all available services
ros2 service call /service example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
ros2 topic hz /topic_name    # Measure publishing frequency
```

**Core Concepts**:
- **Node**: An independent process in the ROS 2 graph (e.g., sensor driver, controller)
- **Topic**: A named channel for asynchronous data streaming (e.g., `/camera/image_raw`)
- **Service**: A synchronous RPC mechanism for request-response (e.g., `/get_pose`)

---

## Further Reading

Official Documentation:
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [rclpy API Reference](https://docs.ros2.org/latest/api/rclpy/)
- [Understanding ROS 2 Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

Tutorials and Examples:
- [ROS 2 Beginner Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Example Code Repository](https://github.com/ros2/examples)

Community Resources:
- [ROS Discourse Forum](https://discourse.ros.org/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)

---

## Next Steps

You're now ready to move on to the next chapter!

**Next Chapter**: [Chapter 1.2: ROS 2 for Humanoid Robots](./ch2-ros2-humanoids.md)

In the next chapter, you'll learn how to apply these ROS 2 fundamentals to humanoid robotics—controlling joint states, publishing sensor data from IMUs and force sensors, and coordinating multiple controllers for locomotion and manipulation.

**Optional Practice**:
- Implement a "robot state monitor" service that tracks when the last message was received on multiple topics
- Explore ROS 2 launch files to start all three project nodes simultaneously
- Read about QoS (Quality of Service) profiles and how they affect message delivery reliability
