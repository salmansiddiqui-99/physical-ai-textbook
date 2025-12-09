---
id: ch2-cognitive-planning
title: "Cognitive Planning: LLM-Powered Task Decomposition"
sidebar_label: "Cognitive Planning"
sidebar_position: 11
description: Learn to use large language models (LLMs) to decompose high-level tasks into executable action sequences, enabling humanoid robots to understand and plan complex multi-step behaviors autonomously.
keywords:
  - LLM
  - GPT-4
  - task planning
  - task decomposition
  - action graphs
  - cognitive architectures
  - RAG
  - few-shot prompting
prerequisites:
  - Chapter 4.1 (Voice-to-Action)
  - Python 3.10+ with OpenAI or Anthropic SDK
  - Understanding of ROS 2 actions and services
learning_objectives:
  - Explain how LLMs enable cognitive planning for humanoid robots
  - Implement task decomposition pipelines using GPT-4 or Claude
  - Create action graphs from natural language task descriptions
  - Integrate LLM planners with ROS 2 execution systems
estimated_time: 90 minutes
---

# Cognitive Planning: LLM-Powered Task Decomposition

## Learning Objectives

By the end of this chapter, you will be able to:

- Use LLMs (GPT-4, Claude) to decompose complex tasks into action sequences
- Implement few-shot prompting for robust task planning
- Create executable action graphs from LLM outputs
- Integrate cognitive planning with ROS 2 humanoid control systems

## Prerequisites

Before starting this chapter, you should:

- Have completed [Chapter 4.1: Voice-to-Action](./ch1-voice-to-action.md)
- Understand ROS 2 actions from [Module 1](../module1-ros2/ch2-ros2-humanoids.md)
- Have OpenAI API key or Anthropic API key for LLM access
- Familiarity with JSON and structured data formats

## Introduction

Traditional robot task planning uses symbolic AI: domain-specific languages (PDDL), finite state machines, or behavior trees. These approaches require expert knowledge to encode every scenario. When a user says "clean the living room", a classical planner needs pre-programmed rules defining "clean" as a sequence like: `navigate_to(living_room) → detect_objects(trash) → grasp(trash) → navigate_to(bin) → release(trash)`.

**Large Language Models (LLMs)** like GPT-4 and Claude change this paradigm. Trained on vast amounts of human knowledge, LLMs understand common-sense reasoning: "cleaning" involves picking up clutter, vacuuming floors, and organizing items. By prompting an LLM with a task description and available robot actions, we can generate executable plans **without handcrafted rules**.

This chapter covers **cognitive planning** for humanoid robots: using LLMs to transform high-level goals ("prepare dinner") into action graphs (`navigate → open_fridge → grasp_vegetables → close_fridge → navigate_kitchen → place_on_counter → ...`). You'll learn prompt engineering techniques, action graph execution, and error recovery strategies for real-world deployment.

---

## Section 1: LLMs for Robot Task Planning

### Subsection 1.1: Why Use LLMs for Planning?

**Advantages**:
- **Common-sense reasoning**: Understands implicit knowledge (e.g., "set the table" requires plates, utensils)
- **Natural language interface**: Users describe tasks conversationally
- **Few-shot learning**: Provide 2-3 examples, LLM generalizes to new tasks
- **Adaptability**: No need to reprogram for new scenarios

**Limitations**:
- **Hallucinations**: May generate invalid actions ("fly to ceiling")
- **Non-deterministic**: Slight prompt changes alter plans
- **Latency**: API calls take 1-3 seconds (not real-time)
- **Cost**: GPT-4 costs $0.03 per 1K tokens (~$1-2 per hour of planning)

**Best Practices**:
- **Validate outputs**: Check generated actions against available action set
- **Use structured output**: JSON format for parsing
- **Iterative refinement**: Re-prompt with error feedback if plan fails

### Subsection 1.2: LLM API Setup

**OpenAI GPT-4**:
```bash
pip install openai

export OPENAI_API_KEY="sk-..."  # Your API key from platform.openai.com
```

**Anthropic Claude**:
```bash
pip install anthropic

export ANTHROPIC_API_KEY="sk-ant-..."  # Your API key from console.anthropic.com
```

**Example: Test API Connection**:
```python
import openai

openai.api_key = "sk-..."

response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[{"role": "user", "content": "Say 'API connected!'"}]
)

print(response.choices[0].message.content)
# Output: API connected!
```

---

## Section 2: Task Decomposition with Few-Shot Prompting

### Subsection 2.1: Prompt Engineering Basics

**Effective Prompt Structure**:
1. **System prompt**: Define role and output format
2. **Few-shot examples**: Show input/output pairs
3. **Task description**: User's high-level goal
4. **Available actions**: List of robot capabilities

**Example Prompt Template**:
```
You are a task planning AI for a humanoid robot.

Given a high-level task, decompose it into a sequence of primitive actions.

Available actions:
- navigate_to(location)
- detect_objects(object_type)
- grasp(object_id)
- place(location)
- open(container)
- close(container)

Examples:

Task: "Bring me a water bottle"
Plan:
[
  {"action": "navigate_to", "params": {"location": "kitchen"}},
  {"action": "detect_objects", "params": {"object_type": "bottle"}},
  {"action": "grasp", "params": {"object_id": "bottle_0"}},
  {"action": "navigate_to", "params": {"location": "user"}},
  {"action": "place", "params": {"location": "user_hand"}}
]

Task: "Clean the table"
Plan:
[
  {"action": "navigate_to", "params": {"location": "dining_table"}},
  {"action": "detect_objects", "params": {"object_type": "clutter"}},
  {"action": "grasp", "params": {"object_id": "clutter_0"}},
  {"action": "navigate_to", "params": {"location": "trash_bin"}},
  {"action": "place", "params": {"location": "trash_bin"}},
  {"action": "navigate_to", "params": {"location": "dining_table"}},
  {"action": "detect_objects", "params": {"object_type": "clutter"}},
  ...
]

Now decompose this task:

Task: "{user_task}"
Plan:
```

### Subsection 2.2: Implementing LLM Task Planner

**Example: GPT-4 Task Planner**:

```python
#!/usr/bin/env python3
"""
LLM-based task planner for humanoid robots

Purpose: Decompose natural language tasks into executable action sequences
Environment: Python 3.10+, OpenAI API
"""

import openai
import json
from typing import List, Dict, Any


class LLMTaskPlanner:
    """Task decomposition using GPT-4"""

    def __init__(self, api_key: str, model: str = "gpt-4"):
        self.model = model
        openai.api_key = api_key

        # Define available robot actions
        self.actions = [
            "navigate_to(location)",
            "detect_objects(object_type)",
            "grasp(object_id)",
            "place(location)",
            "open(container)",
            "close(container)",
            "rotate(angle_degrees)",
            "wait(duration_seconds)"
        ]

        # Few-shot examples
        self.examples = [
            {
                "task": "Bring me a water bottle",
                "plan": [
                    {"action": "navigate_to", "params": {"location": "kitchen"}},
                    {"action": "detect_objects", "params": {"object_type": "bottle"}},
                    {"action": "grasp", "params": {"object_id": "bottle_0"}},
                    {"action": "navigate_to", "params": {"location": "user"}},
                    {"action": "place", "params": {"location": "user_hand"}}
                ]
            },
            {
                "task": "Clean the table",
                "plan": [
                    {"action": "navigate_to", "params": {"location": "dining_table"}},
                    {"action": "detect_objects", "params": {"object_type": "clutter"}},
                    {"action": "grasp", "params": {"object_id": "clutter_0"}},
                    {"action": "navigate_to", "params": {"location": "trash_bin"}},
                    {"action": "place", "params": {"location": "trash_bin"}}
                ]
            }
        ]

    def plan(self, task: str) -> List[Dict[str, Any]]:
        """
        Decompose task into action sequence

        Args:
            task: Natural language task description

        Returns:
            List of action dictionaries
        """
        # Build prompt
        prompt = self._build_prompt(task)

        # Call GPT-4
        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": "You are a task planning AI for humanoid robots. Output valid JSON only."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.2,  # Low temperature for consistency
            max_tokens=1000
        )

        # Parse response
        try:
            plan_text = response.choices[0].message.content
            # Extract JSON array (handle markdown code blocks)
            if "```json" in plan_text:
                plan_text = plan_text.split("```json")[1].split("```")[0]
            elif "```" in plan_text:
                plan_text = plan_text.split("```")[1].split("```")[0]

            plan = json.loads(plan_text.strip())
            return plan

        except json.JSONDecodeError as e:
            print(f"Failed to parse LLM output: {e}")
            print(f"Raw output: {plan_text}")
            return []

    def _build_prompt(self, task: str) -> str:
        """Build few-shot prompt with examples"""
        prompt = "You are a task planning AI for a humanoid robot.\n\n"
        prompt += "Given a high-level task, decompose it into a sequence of primitive actions.\n\n"
        prompt += "Available actions:\n"
        for action in self.actions:
            prompt += f"- {action}\n"
        prompt += "\nExamples:\n\n"

        # Add few-shot examples
        for ex in self.examples:
            prompt += f"Task: \"{ex['task']}\"\n"
            prompt += f"Plan:\n{json.dumps(ex['plan'], indent=2)}\n\n"

        # Add user task
        prompt += f"Now decompose this task:\n\n"
        prompt += f"Task: \"{task}\"\n"
        prompt += f"Plan:\n"

        return prompt


# Usage example
if __name__ == '__main__':
    planner = LLMTaskPlanner(api_key="sk-...")  # Your API key

    tasks = [
        "Set the dining table for 2 people",
        "Make me a cup of coffee",
        "Water the plants in the living room"
    ]

    for task in tasks:
        print(f"\n{'='*60}")
        print(f"Task: {task}")
        print(f"{'='*60}")

        plan = planner.plan(task)

        print("Generated Plan:")
        for i, step in enumerate(plan):
            print(f"  {i+1}. {step['action']}({step['params']})")
```

**Example Output**:
```
============================================================
Task: Set the dining table for 2 people
============================================================
Generated Plan:
  1. navigate_to({'location': 'kitchen'})
  2. open({'container': 'cabinet'})
  3. detect_objects({'object_type': 'plates'})
  4. grasp({'object_id': 'plate_0'})
  5. close({'container': 'cabinet'})
  6. navigate_to({'location': 'dining_table'})
  7. place({'location': 'table_position_1'})
  8. navigate_to({'location': 'kitchen'})
  9. open({'container': 'drawer'})
  10. detect_objects({'object_type': 'utensils'})
  11. grasp({'object_id': 'fork_0'})
  12. close({'container': 'drawer'})
  13. navigate_to({'location': 'dining_table'})
  14. place({'location': 'table_position_1'})
  ...
```

---

## Section 3: Action Graph Execution

### Subsection 3.1: Converting Plan to ROS 2 Actions

**Action Graph Executor**:

```python
#!/usr/bin/env python3
"""
Execute LLM-generated action graphs in ROS 2

Purpose: Map LLM actions to ROS 2 action servers and execute sequentially
Environment: ROS 2 Humble
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class ActionGraphExecutor(Node):
    """Execute action sequences from LLM planner"""

    def __init__(self):
        super().__init__('action_graph_executor')

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Location database
        self.locations = {
            'kitchen': {'x': 5.0, 'y': 2.0, 'yaw': 0.0},
            'dining_table': {'x': 0.0, 'y': 3.0, 'yaw': 1.57},
            'user': {'x': -2.0, 'y': 0.0, 'yaw': 3.14},
        }

        self.get_logger().info('Action graph executor ready')

    def execute_plan(self, plan: list):
        """Execute action sequence"""
        self.get_logger().info(f"Executing plan with {len(plan)} steps")

        for i, step in enumerate(plan):
            self.get_logger().info(f"Step {i+1}/{len(plan)}: {step['action']}")

            success = self.execute_action(step)

            if not success:
                self.get_logger().error(f"Step {i+1} failed - aborting plan")
                return False

        self.get_logger().info("Plan execution complete!")
        return True

    def execute_action(self, action_dict: dict) -> bool:
        """Execute single action"""
        action = action_dict['action']
        params = action_dict['params']

        if action == 'navigate_to':
            return self.navigate_to(params['location'])
        elif action == 'detect_objects':
            return self.detect_objects(params['object_type'])
        elif action == 'grasp':
            return self.grasp(params['object_id'])
        elif action == 'place':
            return self.place(params['location'])
        elif action == 'open' or action == 'close':
            return self.manipulate_container(action, params['container'])
        elif action == 'wait':
            return self.wait(params['duration_seconds'])
        else:
            self.get_logger().warn(f"Unknown action: {action}")
            return False

    def navigate_to(self, location: str) -> bool:
        """Navigate to named location"""
        if location not in self.locations:
            self.get_logger().error(f"Unknown location: {location}")
            return False

        loc = self.locations[location]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = loc['x']
        goal_msg.pose.pose.position.y = loc['y']

        qz = math.sin(loc['yaw'] / 2.0)
        qw = math.cos(loc['yaw'] / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Navigating to {location}")
        self.nav_client.wait_for_server()

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return True

    def detect_objects(self, object_type: str) -> bool:
        """Object detection (placeholder)"""
        self.get_logger().info(f"Detecting {object_type} objects")
        # TODO: Integrate with YOLO from Module 3
        return True

    def grasp(self, object_id: str) -> bool:
        """Grasp object (placeholder)"""
        self.get_logger().info(f"Grasping {object_id}")
        # TODO: Integrate with grasp planner
        return True

    def place(self, location: str) -> bool:
        """Place object (placeholder)"""
        self.get_logger().info(f"Placing object at {location}")
        return True

    def manipulate_container(self, action: str, container: str) -> bool:
        """Open/close container (placeholder)"""
        self.get_logger().info(f"{action.capitalize()}ing {container}")
        return True

    def wait(self, duration: float) -> bool:
        """Wait for specified duration"""
        import time
        self.get_logger().info(f"Waiting {duration} seconds")
        time.sleep(duration)
        return True


def main(args=None):
    rclpy.init(args=args)
    executor = ActionGraphExecutor()

    # Example: Execute LLM-generated plan
    plan = [
        {"action": "navigate_to", "params": {"location": "kitchen"}},
        {"action": "detect_objects", "params": {"object_type": "bottle"}},
        {"action": "grasp", "params": {"object_id": "bottle_0"}},
        {"action": "navigate_to", "params": {"location": "user"}},
        {"action": "place", "params": {"location": "user_hand"}}
    ]

    success = executor.execute_plan(plan)
    print(f"Plan execution: {'SUCCESS' if success else 'FAILED'}")

    executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Section 4: Integrating LLM Planning with Voice Control

**Complete Pipeline**:

```python
#!/usr/bin/env python3
"""
Complete cognitive humanoid system: Voice → LLM Planning → Execution

Usage: ros2 run cognitive_control cognitive_humanoid_node
"""

import rclpy
from rclpy.node import Node
from llm_task_planner import LLMTaskPlanner
from action_graph_executor import ActionGraphExecutor
import threading


class CognitiveHumanoidNode(Node):
    """Cognitive planning system for humanoid robots"""

    def __init__(self):
        super().__init__('cognitive_humanoid')

        # Initialize components
        api_key = self.declare_parameter('openai_api_key', 'sk-...').value
        self.planner = LLMTaskPlanner(api_key=api_key)
        self.executor = ActionGraphExecutor()

        self.get_logger().info('Cognitive humanoid system ready')

    def execute_task(self, task_description: str):
        """Plan and execute task from natural language"""
        self.get_logger().info(f"Task received: '{task_description}'")

        # Step 1: Generate plan with LLM
        self.get_logger().info("Generating plan with LLM...")
        plan = self.planner.plan(task_description)

        if not plan:
            self.get_logger().error("Failed to generate plan")
            return False

        self.get_logger().info(f"Generated {len(plan)}-step plan")

        # Step 2: Execute plan
        self.get_logger().info("Executing plan...")
        success = self.executor.execute_plan(plan)

        if success:
            self.get_logger().info(f"Task '{task_description}' completed successfully!")
        else:
            self.get_logger().error(f"Task '{task_description}' failed during execution")

        return success


def main(args=None):
    rclpy.init(args=args)
    cognitive_node = CognitiveHumanoidNode()

    # Example tasks
    tasks = [
        "Bring me a water bottle",
        "Clean the table",
        "Set the table for dinner"
    ]

    for task in tasks:
        cognitive_node.execute_task(task)
        print()

    cognitive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Summary

In this chapter, you learned:

- **LLM task planning**: Using GPT-4/Claude for task decomposition
- **Few-shot prompting**: Providing examples for consistent outputs
- **Action graph execution**: Mapping LLM outputs to ROS 2 actions
- **Complete integration**: Voice → LLM planning → Execution pipeline

**Key Concepts**:
- **Common-sense reasoning**: LLMs understand implicit task requirements
- **Structured output**: JSON format for reliable parsing
- **Sequential execution**: Action graphs as ordered sequences
- **Error handling**: Validate plans and handle execution failures

---

## Further Reading

- [OpenAI API Documentation](https://platform.openai.com/docs)
- [Prompt Engineering Guide](https://www.promptingguide.ai/)
- [LangChain for Robotics](https://python.langchain.com/docs/use_cases/robotics)

Research Papers:
- "Do As I Can, Not As I Say" (Google SayCan, 2022) - LLMs for robot planning
- "Inner Monologue" (Google, 2023) - LLMs with environment feedback

---

## Next Steps

**Next Chapter**: [Chapter 4.3: Capstone Project](./ch3-capstone-project.md)

In the final chapter, you'll integrate everything from Modules 1-4 into a complete autonomous humanoid system: voice commands, LLM planning, Isaac Sim perception, and Nav2 navigation working together!
