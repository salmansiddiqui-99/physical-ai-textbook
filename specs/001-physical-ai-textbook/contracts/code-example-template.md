# Code Example Template

**Purpose**: This template provides a standardized structure for all code examples in the Physical AI & Humanoid Robotics textbook. Use this template to ensure consistency, clarity, and educational value.

---

## Example: [Descriptive Title]

**Purpose**: [One sentence explaining what this code demonstrates - focus on educational value]

**Environment**: [Select one: ROS 2 Humble | ROS 2 Iron | Gazebo Fortress | Isaac Sim | Unity]

**Difficulty**: [Select one: Beginner | Intermediate | Advanced]

---

### Dependencies

**System Packages**:
```bash
# Ubuntu/Debian packages required
sudo apt update
sudo apt install [package1] [package2] [package3]
```

**Python Packages** (if applicable):
```bash
# Python dependencies
pip3 install [package1] [package2]
```

**ROS 2 Packages** (if applicable):
```bash
# ROS 2 workspace packages
sudo apt install ros-humble-[package-name]
# OR build from source
cd ~/ros2_ws/src
git clone [repository-url]
cd ~/ros2_ws
colcon build --symlink-install
```

---

### Code

```[language]
#!/usr/bin/env python3  # [Use appropriate shebang for language]
"""
[Module/Script Docstring]

Purpose: [What this code does]
Author: Physical AI & Humanoid Robotics Course
License: [If applicable, e.g., Apache 2.0]

Usage:
    python3 [script-name.py] [arguments]

Example:
    python3 example_node.py --rate 10

Inputs:
    [Describe inputs, arguments, or topics subscribed to]

Outputs:
    [Describe outputs, published topics, or side effects]
"""

# Standard library imports
import sys
import argparse

# Third-party imports
import numpy as np

# ROS 2 imports (if applicable)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# [Group imports logically and alphabetically within groups]


class ExampleClass(Node):
    """
    [Class docstring explaining purpose, key methods, and usage]

    This class demonstrates [core concept] by implementing [functionality].

    Attributes:
        attribute1 (type): [Description]
        attribute2 (type): [Description]
    """

    def __init__(self, param1, param2):
        """
        Initialize the ExampleClass.

        Args:
            param1 (type): [Description]
            param2 (type): [Description]
        """
        super().__init__('example_node_name')

        # Initialization logic with explanatory comments
        # Comment style: Explain WHY, not just WHAT
        self.attribute1 = param1  # Purpose: [Why this is needed]

        # Set up publishers/subscribers/timers (for ROS 2 examples)
        self.publisher_ = self.create_publisher(
            String,           # Message type
            'output_topic',   # Topic name
            10                # Queue size
        )

        # Create timer for periodic callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Node initialized successfully')


    def timer_callback(self):
        """
        [Method docstring explaining when this is called and what it does]

        This callback is executed every [timer_period] seconds and [purpose].
        """
        # Implementation with step-by-step comments
        msg = String()
        msg.data = f'Example message at {self.get_clock().now().to_msg()}'

        # Publish message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')


    def helper_method(self, input_data):
        """
        [Method docstring]

        Args:
            input_data (type): [Description]

        Returns:
            type: [Description of return value]
        """
        # Helper method implementation
        result = input_data * 2  # Example operation
        return result


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.

    Args:
        args (list, optional): Command-line arguments. Defaults to None.
    """
    # Parse command-line arguments (if needed)
    parser = argparse.ArgumentParser(description='[Script description]')
    parser.add_argument('--param1', type=str, default='value',
                        help='[Parameter description]')
    parsed_args = parser.parse_args()

    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node instance
    node = ExampleClass(
        param1=parsed_args.param1,
        param2='default_value'
    )

    try:
        # Spin node to execute callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### Code Explanation

Let's break down this example step-by-step:

#### Imports (Lines X-Y)
- `import rclpy`: [Why this is needed - e.g., "Core ROS 2 Python client library"]
- `from rclpy.node import Node`: [Why - e.g., "Base class for all ROS 2 nodes"]
- `from std_msgs.msg import String`: [Why - e.g., "Standard message type for text data"]

#### Class Definition (Line Z)
- **Inheritance**: `ExampleClass(Node)` inherits from ROS 2's `Node` class
- **Purpose**: [What problem this class solves or what pattern it demonstrates]

#### Initialization Method (Lines A-B)
1. `super().__init__('example_node_name')`: Initialize parent Node class with unique name
2. `self.create_publisher(...)`: Set up publisher for [purpose]
3. `self.create_timer(...)`: Create timer to call `timer_callback` every [period]

**Key Concept**: [Explain the educational concept - e.g., "Timers enable periodic execution without blocking the main thread"]

#### Callback Method (Lines C-D)
- **Triggered**: Every [timer_period] seconds by the timer
- **Functionality**: [What it does step-by-step]
- **ROS 2 Pattern**: [What ROS 2 design pattern this demonstrates]

#### Main Function (Lines E-F)
- **Line E**: `rclpy.init()` initializes ROS 2 communications
- **Line F**: `rclpy.spin(node)` keeps node alive and processes callbacks
- **Line G**: Cleanup in `finally` block ensures proper shutdown

---

### Running the Code

#### Step 1: Set Up Environment

```bash
# Source ROS 2 workspace (adjust path for your distribution)
source /opt/ros/humble/setup.bash

# If using custom workspace, also source it
source ~/ros2_ws/install/setup.bash
```

#### Step 2: Run the Node

```bash
# Basic execution
python3 example_node.py

# With arguments (if applicable)
python3 example_node.py --param1 custom_value
```

#### Step 3: Verify Output

**In the same terminal**, you should see:
```
[INFO] [example_node_name]: Node initialized successfully
[INFO] [example_node_name]: Published: "Example message at ..."
[INFO] [example_node_name]: Published: "Example message at ..."
```

**In a new terminal**, check published topics:
```bash
# List all active topics
ros2 topic list

# Echo messages from the topic
ros2 topic echo /output_topic

# Check topic publishing rate
ros2 topic hz /output_topic
```

---

### Expected Output

**Terminal 1 (Node Output)**:
```
[INFO] [1638360000.123456789] [example_node_name]: Node initialized successfully
[INFO] [1638360000.623456789] [example_node_name]: Published: "Example message at sec=1638360000 nanosec=623456789"
[INFO] [1638360001.123456789] [example_node_name]: Published: "Example message at sec=1638360001 nanosec=123456789"
^C[INFO] [1638360005.789012345] [example_node_name]: Node stopped by user
```

**Terminal 2 (Topic Echo)**:
```bash
$ ros2 topic echo /output_topic
data: 'Example message at sec=1638360000 nanosec=623456789'
---
data: 'Example message at sec=1638360001 nanosec=123456789'
---
```

---

### Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| `ModuleNotFoundError: No module named 'rclpy'` | ROS 2 environment not sourced | Run `source /opt/ros/humble/setup.bash` |
| `No executable found` | Script not executable | Run `chmod +x example_node.py` |
| Node starts but doesn't publish | Topic name mismatch | Verify topic with `ros2 topic list` |
| `ImportError: cannot import name 'String'` | Missing ROS 2 packages | Install with `sudo apt install ros-humble-std-msgs` |
| High CPU usage | Timer period too short | Increase `timer_period` value (e.g., from 0.1 to 0.5) |

**Debug Commands**:
```bash
# Check ROS 2 daemon status
ros2 daemon status

# List all nodes
ros2 node list

# Get node info
ros2 node info /example_node_name

# View parameter values (if node uses parameters)
ros2 param list /example_node_name
```

---

### Variations and Extensions

**Beginner Challenge**: Modify this example to publish a counter that increments with each message.

<details>
<summary>Hint (click to expand)</summary>

Add an integer counter in `__init__` and increment it in `timer_callback`.
</details>

**Intermediate Challenge**: Create a subscriber node that listens to `/output_topic` and prints received messages.

<details>
<summary>Hint (click to expand)</summary>

Use `self.create_subscription(String, 'output_topic', callback, 10)` in `__init__`.
</details>

**Advanced Challenge**: Extend this to a request-response pattern using ROS 2 services instead of topics.

<details>
<summary>Hint (click to expand)</summary>

Replace publisher with service server using `self.create_service(...)`. See ROS 2 service documentation.
</details>

---

### Related Examples

- [Link to previous example if building on concepts]
- [Link to next example in sequence]
- [Link to official ROS 2 tutorial for deeper dive]

---

### Additional Resources

**Official Documentation**:
- [ROS 2 rclpy API](https://docs.ros2.org/latest/api/rclpy/)
- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts.html)

**Video Tutorials** (optional):
- [Link to relevant video if available]

**Source Code**:
- [Link to downloadable version in `/static/code/moduleX/` if >100 lines]

---

## Template Notes

**For Textbook Authors**:
- Keep code examples under 100 lines when possible
- For longer examples, provide downloadable file and show key excerpts
- Always include error handling and cleanup (try/except/finally)
- Use type hints where they improve clarity: `def method(self, x: int) -> str:`
- Comment ratio: ~30% of lines should have explanatory comments
- Test all code examples before publication (validation_status: tested-passes)

**Code Style**:
- Follow PEP 8 for Python
- Use snake_case for variables and functions
- Use descriptive names (avoid single-letter except for loop indices)
- Maximum line length: 88 characters (Black formatter standard)
- Add blank lines between logical sections for readability

**Educational Focus**:
- Prioritize clarity over cleverness
- Show one concept at a time (don't combine too many advanced features)
- Include "Why" comments, not just "What" comments
- Provide multiple difficulty levels where applicable
