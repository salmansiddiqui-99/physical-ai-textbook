#!/usr/bin/env python3
"""
Action Graph Executor for ROS 2

Purpose: Execute LLM-generated action sequences using ROS 2 action clients
Environment: ROS 2 Humble, Nav2, custom action servers
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import json
from typing import List, Dict, Any, Optional
from enum import Enum


class ExecutionStatus(Enum):
    """Status of action execution"""
    IDLE = "idle"
    EXECUTING = "executing"
    SUCCESS = "success"
    FAILED = "failed"
    CANCELLED = "cancelled"


class ActionGraphExecutor(Node):
    """Execute action graphs from LLM planner"""

    def __init__(self):
        super().__init__('action_graph_executor')

        # Subscribe to action plans from LLM planner
        self.create_subscription(
            String,
            '/cognitive/action_plan',
            self.plan_callback,
            10
        )

        # Action clients for different action types
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Additional action clients would be added here:
        # self.grasp_client = ActionClient(self, GraspObject, 'grasp_object')
        # self.place_client = ActionClient(self, PlaceObject, 'place_object')

        # Execution state
        self.current_plan: Optional[List[Dict[str, Any]]] = None
        self.current_step = 0
        self.execution_status = ExecutionStatus.IDLE

        # Known locations (in real system, would query from map service)
        self.locations = {
            'kitchen': {'x': 5.0, 'y': 3.0, 'theta': 0.0},
            'living_room': {'x': 1.0, 'y': 1.0, 'theta': 1.57},
            'bedroom': {'x': 8.0, 'y': 5.0, 'theta': 3.14},
            'entrance': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'user': {'x': 2.0, 'y': 2.0, 'theta': 0.0}
        }

        self.get_logger().info('Action Graph Executor ready')

    def plan_callback(self, msg: String):
        """
        Receive and execute action plan from LLM

        Args:
            msg: JSON-encoded action plan
        """
        try:
            plan = json.loads(msg.data)
            self.get_logger().info(f'Received plan with {len(plan)} steps')
            self.execute_plan(plan)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid plan JSON: {e}')

    def execute_plan(self, plan: List[Dict[str, Any]]):
        """
        Execute action plan sequentially

        Args:
            plan: List of action dictionaries
        """
        if self.execution_status == ExecutionStatus.EXECUTING:
            self.get_logger().warn('Already executing a plan, rejecting new plan')
            return

        self.current_plan = plan
        self.current_step = 0
        self.execution_status = ExecutionStatus.EXECUTING

        # Start execution
        self.execute_next_step()

    def execute_next_step(self):
        """Execute the next step in the plan"""
        if self.current_plan is None:
            return

        if self.current_step >= len(self.current_plan):
            # Plan completed
            self.get_logger().info('Plan execution completed successfully')
            self.execution_status = ExecutionStatus.SUCCESS
            self.current_plan = None
            return

        # Get current action
        action = self.current_plan[self.current_step]
        action_name = action.get('action')
        params = action.get('params', {})

        self.get_logger().info(f'Step {self.current_step + 1}/{len(self.current_plan)}: '
                               f'{action_name}({params})')

        # Route to appropriate action handler
        if action_name == 'navigate_to':
            self.execute_navigation(params)
        elif action_name == 'detect_objects':
            self.execute_detection(params)
        elif action_name == 'grasp':
            self.execute_grasp(params)
        elif action_name == 'place':
            self.execute_place(params)
        elif action_name == 'rotate':
            self.execute_rotation(params)
        elif action_name == 'wait':
            self.execute_wait(params)
        else:
            self.get_logger().error(f'Unknown action: {action_name}')
            self.handle_step_failure()

    def execute_navigation(self, params: Dict[str, Any]):
        """
        Execute navigation action

        Args:
            params: Must contain 'location' key
        """
        location_name = params.get('location')
        if location_name not in self.locations:
            self.get_logger().error(f'Unknown location: {location_name}')
            self.handle_step_failure()
            return

        # Build Nav2 goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        loc = self.locations[location_name]
        goal.pose.pose.position.x = loc['x']
        goal.pose.pose.position.y = loc['y']
        goal.pose.pose.position.z = 0.0

        # Convert theta to quaternion (simplified, z-axis rotation only)
        import math
        theta = loc['theta']
        goal.pose.pose.orientation.z = math.sin(theta / 2)
        goal.pose.pose.orientation.w = math.cos(theta / 2)

        # Send goal
        self.get_logger().info(f'Navigating to {location_name}: ({loc["x"]}, {loc["y"]})')

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server not available')
            self.handle_step_failure()
            return

        send_goal_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # In real implementation, would show progress to user
        pass

    def nav_goal_response_callback(self, future):
        """Handle navigation goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.handle_step_failure()
            return

        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            self.handle_step_success()
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            self.handle_step_failure()

    def execute_detection(self, params: Dict[str, Any]):
        """
        Execute object detection

        Args:
            params: Must contain 'object_type' key
        """
        object_type = params.get('object_type')
        self.get_logger().info(f'Detecting objects of type: {object_type}')

        # In real implementation, would call vision service
        # For now, simulate success after 1 second
        self.create_timer(1.0, self.handle_step_success, clock=self.get_clock())

    def execute_grasp(self, params: Dict[str, Any]):
        """
        Execute grasp action

        Args:
            params: Must contain 'object_id' key
        """
        object_id = params.get('object_id')
        self.get_logger().info(f'Grasping object: {object_id}')

        # In real implementation, would call manipulation service
        self.create_timer(2.0, self.handle_step_success, clock=self.get_clock())

    def execute_place(self, params: Dict[str, Any]):
        """
        Execute place action

        Args:
            params: Must contain 'location' key
        """
        location = params.get('location')
        self.get_logger().info(f'Placing object at: {location}')

        # In real implementation, would call manipulation service
        self.create_timer(2.0, self.handle_step_success, clock=self.get_clock())

    def execute_rotation(self, params: Dict[str, Any]):
        """
        Execute rotation action

        Args:
            params: Must contain 'angle' key (degrees)
        """
        angle = params.get('angle', 90)
        self.get_logger().info(f'Rotating {angle} degrees')

        # In real implementation, would command base controller
        self.create_timer(1.0, self.handle_step_success, clock=self.get_clock())

    def execute_wait(self, params: Dict[str, Any]):
        """
        Execute wait action

        Args:
            params: Must contain 'duration_seconds' key
        """
        duration = params.get('duration_seconds', 1.0)
        self.get_logger().info(f'Waiting {duration} seconds')

        self.create_timer(duration, self.handle_step_success, clock=self.get_clock())

    def handle_step_success(self):
        """Handle successful completion of current step"""
        self.current_step += 1
        self.execute_next_step()

    def handle_step_failure(self):
        """Handle failure of current step"""
        self.get_logger().error(f'Step {self.current_step + 1} failed, aborting plan')
        self.execution_status = ExecutionStatus.FAILED
        self.current_plan = None

        # In production, would trigger error recovery or replan


def main(args=None):
    rclpy.init(args=args)
    executor = ActionGraphExecutor()

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
