---
id: module-4-chapter-2
slug: /module-4/chapter-2
title: "Chapter 2: Cognitive Planning with LLMs"
sidebar_position: 3
---

# Chapter 2: Cognitive Planning with LLMs

## Introduction

Cognitive planning with Large Language Models (LLMs) enables humanoid robots to understand high-level goals and generate detailed action plans to achieve them. This chapter explores how to integrate LLMs like GPT models into robotic systems for sophisticated cognitive planning and decision-making.

## Learning Objectives

By the end of this chapter, you will be able to:
- Integrate LLMs with robotic planning systems
- Design prompt engineering strategies for robotic tasks
- Implement hierarchical planning with LLMs
- Handle multi-step planning and execution in robotics

## LLM Integration Architecture

```python
import openai
import json
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import asyncio

@dataclass
class PlanningTask:
    """Represents a planning task for the robot"""
    goal: str
    context: Dict[str, Any]
    constraints: List[str]
    available_actions: List[str]

@dataclass
class PlanStep:
    """Represents a single step in the plan"""
    action: str
    parameters: Dict[str, Any]
    description: str
    expected_outcome: str

class LLMPlanner:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        openai.api_key = api_key
        self.model = model

    def create_planning_prompt(self, task: PlanningTask) -> str:
        """Create a prompt for the LLM to generate a plan"""
        prompt = f"""
        You are an expert robotic planner. Generate a detailed step-by-step plan for a humanoid robot to achieve the following goal:

        GOAL: {task.goal}

        CONTEXT:
        - Robot capabilities: {', '.join(task.available_actions)}
        - Current environment: {task.context.get('environment', 'unknown')}
        - Current robot state: {task.context.get('robot_state', 'idle')}
        - Available objects: {task.context.get('objects', [])}

        CONSTRAINTS:
        {chr(10).join([f"- {constraint}" for constraint in task.constraints])}

        Generate a plan as a JSON array of steps. Each step should have:
        - "action": The specific action to take
        - "parameters": Any parameters needed for the action
        - "description": A brief description of the step
        - "expected_outcome": What should happen after executing this step

        IMPORTANT: Only use actions from the available actions list. Be specific and practical.
        """

        return prompt

    async def generate_plan(self, task: PlanningTask) -> Optional[List[PlanStep]]:
        """Generate a plan using the LLM"""
        prompt = self.create_planning_prompt(task)

        try:
            response = await openai.ChatCompletion.acreate(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are an expert robotic planner that outputs valid JSON only."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=1000
            )

            # Extract and parse the response
            content = response.choices[0].message.content.strip()

            # Clean the response to extract JSON
            if content.startswith("```json"):
                content = content[7:-3]  # Remove markdown code block markers
            elif content.startswith("```"):
                content = content[3:-3]

            plan_data = json.loads(content)

            # Convert to PlanStep objects
            plan_steps = []
            for step_data in plan_data:
                step = PlanStep(
                    action=step_data['action'],
                    parameters=step_data.get('parameters', {}),
                    description=step_data['description'],
                    expected_outcome=step_data['expected_outcome']
                )
                plan_steps.append(step)

            return plan_steps

        except Exception as e:
            print(f"Error generating plan: {e}")
            return None
```

## Robot State and Environment Representation

```python
from typing import Dict, List, Optional
import json

class RobotState:
    """Represents the current state of the robot and environment"""
    def __init__(self):
        self.position = [0.0, 0.0, 0.0]  # x, y, z
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # quaternion
        self.battery_level = 100.0
        self.current_task = None
        self.perceived_objects = []
        self.navigation_map = {}
        self.manipulator_state = "idle"
        self.gripper_state = "open"

    def to_dict(self) -> Dict:
        """Convert robot state to dictionary for LLM context"""
        return {
            "position": self.position,
            "orientation": self.orientation,
            "battery_level": self.battery_level,
            "current_task": self.current_task,
            "perceived_objects": self.perceived_objects,
            "manipulator_state": self.manipulator_state,
            "gripper_state": self.gripper_state
        }

    def update_from_sensor_data(self, sensor_data: Dict):
        """Update robot state from sensor data"""
        if 'position' in sensor_data:
            self.position = sensor_data['position']
        if 'orientation' in sensor_data:
            self.orientation = sensor_data['orientation']
        if 'battery_level' in sensor_data:
            self.battery_level = sensor_data['battery_level']
        if 'objects' in sensor_data:
            self.perceived_objects = sensor_data['objects']

class EnvironmentRepresentation:
    """Represents the environment for planning"""
    def __init__(self):
        self.rooms = {}
        self.obstacles = []
        self.navigable_areas = []
        self.furniture = []
        self.dynamic_objects = []

    def to_dict(self) -> Dict:
        """Convert environment to dictionary for LLM context"""
        return {
            "rooms": self.rooms,
            "obstacles": self.obstacles,
            "navigable_areas": self.navigable_areas,
            "furniture": self.furniture,
            "dynamic_objects": self.dynamic_objects
        }

    def update_from_perception(self, perception_data: Dict):
        """Update environment from perception data"""
        self.obstacles = perception_data.get('obstacles', [])
        self.navigable_areas = perception_data.get('navigable_areas', [])
        self.dynamic_objects = perception_data.get('dynamic_objects', [])
```

## Hierarchical Planning System

```python
import asyncio
from typing import List, Optional

class HierarchicalPlanner:
    def __init__(self, llm_planner: LLMPlanner):
        self.llm_planner = llm_planner
        self.robot_state = RobotState()
        self.environment = EnvironmentRepresentation()

    async def generate_high_level_plan(self, goal: str) -> Optional[List[PlanStep]]:
        """Generate high-level plan using LLM"""
        # Define available high-level actions
        available_actions = [
            "navigate_to_location",
            "pick_up_object",
            "place_object",
            "open_door",
            "close_door",
            "wait_for_person",
            "ask_for_help",
            "return_to_home",
            "charge_battery"
        ]

        task = PlanningTask(
            goal=goal,
            context={
                "environment": self.environment.to_dict(),
                "robot_state": self.robot_state.to_dict(),
                "objects": [obj['name'] for obj in self.robot_state.perceived_objects]
            },
            constraints=[
                "Respect human safety at all times",
                "Conserve battery power",
                "Follow navigation rules",
                "Handle objects carefully"
            ],
            available_actions=available_actions
        )

        return await self.llm_planner.generate_plan(task)

    async def generate_low_level_plan(self, high_level_step: PlanStep) -> Optional[List[PlanStep]]:
        """Generate detailed low-level plan for a high-level step"""
        # Convert high-level action to low-level implementation
        low_level_goal = f"Implement the action: {high_level_step.action} with parameters {high_level_step.parameters}"

        # Define available low-level actions
        available_actions = [
            "move_base",
            "move_arm_to_position",
            "open_gripper",
            "close_gripper",
            "turn_base",
            "tilt_head",
            "detect_objects",
            "grasp_object",
            "release_object"
        ]

        task = PlanningTask(
            goal=low_level_goal,
            context={
                "robot_state": self.robot_state.to_dict(),
                "target_object": high_level_step.parameters.get('object'),
                "target_location": high_level_step.parameters.get('location')
            },
            constraints=[
                "Maintain balance while moving",
                "Avoid collisions",
                "Execute safely"
            ],
            available_actions=available_actions
        )

        return await self.llm_planner.generate_plan(task)

    async def execute_plan(self, plan: List[PlanStep]) -> bool:
        """Execute a plan step by step"""
        for step in plan:
            print(f"Executing: {step.description}")

            # Execute the step (this would interface with actual robot)
            success = await self.execute_step(step)

            if not success:
                print(f"Failed to execute step: {step.description}")
                return False

            # Update robot state after execution
            self.update_robot_state_after_execution(step)

        return True

    async def execute_step(self, step: PlanStep) -> bool:
        """Execute a single plan step"""
        # This is a simplified execution - in practice, this would interface
        # with ROS 2 nodes and actual robot hardware
        print(f"Executing action: {step.action}")
        print(f"Parameters: {step.parameters}")

        # Simulate execution time
        await asyncio.sleep(1)

        # Return success (in practice, check actual robot feedback)
        return True

    def update_robot_state_after_execution(self, step: PlanStep):
        """Update robot state after executing a step"""
        # Update based on the expected outcome of the step
        if step.action == "navigate_to_location":
            # Update position to target location
            target_pos = step.parameters.get('location', [0, 0, 0])
            self.robot_state.position = target_pos
        elif step.action == "pick_up_object":
            # Update gripper state and remove object from environment
            self.robot_state.gripper_state = "closed"
            obj_name = step.parameters.get('object')
            # Remove object from perceived objects
            self.robot_state.perceived_objects = [
                obj for obj in self.robot_state.perceived_objects
                if obj['name'] != obj_name
            ]
```

## Context-Aware Planning

```python
class ContextAwarePlanner:
    def __init__(self, llm_planner: LLMPlanner):
        self.llm_planner = llm_planner
        self.context_history = []
        self.user_preferences = {}
        self.environment_context = {}

    def update_context(self, new_context: Dict):
        """Update planning context with new information"""
        self.context_history.append(new_context)

        # Keep only recent context to manage token usage
        if len(self.context_history) > 10:
            self.context_history = self.context_history[-10:]

    async def generate_contextual_plan(self, goal: str, user_id: str = None) -> Optional[List[PlanStep]]:
        """Generate plan considering context history and user preferences"""
        # Gather context information
        context_info = {
            "current_time": self.get_current_time(),
            "user_preferences": self.user_preferences.get(user_id, {}),
            "recent_interactions": self.context_history[-5:],  # Last 5 interactions
            "environment_state": self.environment_context
        }

        # Define specialized actions based on context
        available_actions = self.get_contextual_actions(context_info)

        task = PlanningTask(
            goal=goal,
            context=context_info,
            constraints=self.get_contextual_constraints(context_info),
            available_actions=available_actions
        )

        return await self.llm_planner.generate_plan(task)

    def get_contextual_actions(self, context: Dict) -> List[str]:
        """Get actions that are appropriate for the current context"""
        actions = [
            "navigate_to_location",
            "pick_up_object",
            "place_object"
        ]

        # Add context-specific actions
        if context.get("environment_state", {}).get("is_night", False):
            actions.extend(["turn_on_lights", "use_navigation_lights"])

        if context.get("user_preferences", {}).get("preferred_speed", "normal") == "fast":
            actions.extend(["move_quickly", "skip_non_essential_steps"])

        if context.get("current_time", {}).get("is_bedtime", False):
            actions.extend(["move_quietly", "avoid_bright_lights"])

        return actions

    def get_contextual_constraints(self, context: Dict) -> List[str]:
        """Get constraints that apply to the current context"""
        constraints = [
            "Respect human safety at all times",
            "Follow navigation rules"
        ]

        # Add context-specific constraints
        if context.get("environment_state", {}).get("people_present", False):
            constraints.append("Be polite and announce actions when people are around")

        if context.get("user_preferences", {}).get("quiet_mode", False):
            constraints.append("Minimize noise during operation")

        if context.get("robot_state", {}).get("battery_level", 100) < 20:
            constraints.append("Prioritize returning to charging station")

        return constraints

    def get_current_time(self) -> Dict:
        """Get current time context"""
        import datetime
        now = datetime.datetime.now()
        return {
            "hour": now.hour,
            "is_bedtime": now.hour >= 22 or now.hour < 6,
            "is_daytime": 6 <= now.hour < 18,
            "is_night": now.hour >= 18 or now.hour < 6
        }
```

## Plan Refinement and Adaptation

```python
class AdaptivePlanner:
    def __init__(self, llm_planner: LLMPlanner):
        self.llm_planner = llm_planner
        self.execution_history = []
        self.failure_patterns = {}

    async def refine_plan(self, original_plan: List[PlanStep], feedback: Dict) -> List[PlanStep]:
        """Refine a plan based on execution feedback"""
        # Create refinement prompt
        refinement_prompt = f"""
        You are a robotic planning expert. The following plan was executed but encountered issues:

        ORIGINAL PLAN:
        {json.dumps([{
            'action': step.action,
            'parameters': step.parameters,
            'description': step.description
        } for step in original_plan], indent=2)}

        EXECUTION FEEDBACK:
        {json.dumps(feedback, indent=2)}

        ISSUES IDENTIFIED:
        - {feedback.get('failed_step', 'Unknown step failed')}
        - {feedback.get('error_description', 'Unknown error')}

        Please generate a refined plan that addresses these issues. Consider:
        1. Alternative approaches to achieve the same goal
        2. Additional safety checks
        3. Environmental adaptations
        4. Error recovery strategies

        Return the refined plan in the same JSON format as before.
        """

        try:
            response = await openai.ChatCompletion.acreate(
                model=self.llm_planner.model,
                messages=[
                    {"role": "system", "content": "You are an expert robotic planner that outputs valid JSON only."},
                    {"role": "user", "content": refinement_prompt}
                ],
                temperature=0.4,
                max_tokens=1000
            )

            content = response.choices[0].message.content.strip()
            if content.startswith("```json"):
                content = content[7:-3]
            elif content.startswith("```"):
                content = content[3:-3]

            plan_data = json.loads(content)

            # Convert to PlanStep objects
            refined_plan = []
            for step_data in plan_data:
                step = PlanStep(
                    action=step_data['action'],
                    parameters=step_data.get('parameters', {}),
                    description=step_data['description'],
                    expected_outcome=step_data.get('expected_outcome', 'Unknown')
                )
                refined_plan.append(step)

            return refined_plan

        except Exception as e:
            print(f"Error refining plan: {e}")
            return original_plan  # Return original if refinement fails

    def learn_from_execution(self, plan: List[PlanStep], execution_result: Dict):
        """Learn from plan execution to improve future planning"""
        failed_step_idx = execution_result.get('failed_step_index', -1)

        if failed_step_idx >= 0 and execution_result.get('success') is False:
            failed_action = plan[failed_step_idx].action
            error_type = execution_result.get('error_type', 'unknown')

            # Record failure pattern
            if failed_action not in self.failure_patterns:
                self.failure_patterns[failed_action] = []

            self.failure_patterns[failed_action].append({
                'error_type': error_type,
                'context': execution_result.get('context', {}),
                'suggested_fix': execution_result.get('suggested_fix', '')
            })

    def get_failure_prediction(self, action: str) -> Optional[Dict]:
        """Predict potential failures for an action based on history"""
        if action in self.failure_patterns:
            # Return the most common failure pattern for this action
            patterns = self.failure_patterns[action]
            # Simple implementation: return first pattern
            return patterns[0] if patterns else None
        return None
```

## Integration with ROS 2

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
import asyncio

class LLMPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_planning_node')

        # Initialize LLM planner
        self.llm_planner = LLMPlanner(api_key=self.get_parameter('openai_api_key').value)
        self.hierarchical_planner = HierarchicalPlanner(self.llm_planner)
        self.context_planner = ContextAwarePlanner(self.llm_planner)
        self.adaptive_planner = AdaptivePlanner(self.llm_planner)

        # ROS 2 interfaces
        self.goal_sub = self.create_subscription(
            String, '/robot_goal', self.goal_callback, 10
        )

        self.plan_pub = self.create_publisher(
            String, '/generated_plan', 10
        )

        self.status_pub = self.create_publisher(
            String, '/planning_status', 10
        )

        # Store current plan
        self.current_plan = None

    def goal_callback(self, msg: String):
        """Handle new goal requests"""
        goal_text = msg.data
        self.get_logger().info(f"Received goal: {goal_text}")

        # Create asyncio task to handle the planning
        future = asyncio.run(self.handle_goal_async(goal_text))
        # In ROS 2, we'd typically use a separate thread or executor for async operations

    async def handle_goal_async(self, goal: str):
        """Handle goal asynchronously"""
        try:
            self.publish_status("Planning started")

            # Generate high-level plan
            high_level_plan = await self.hierarchical_planner.generate_high_level_plan(goal)

            if high_level_plan is None:
                self.publish_status("Planning failed - no plan generated")
                return

            # Execute the plan
            success = await self.hierarchical_planner.execute_plan(high_level_plan)

            if success:
                self.publish_status("Plan executed successfully")
            else:
                self.publish_status("Plan execution failed")

        except Exception as e:
            self.get_logger().error(f"Error in planning: {e}")
            self.publish_status(f"Planning error: {str(e)}")

    def publish_status(self, status: str):
        """Publish planning status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def execute_plan_with_monitoring(self, plan: List[PlanStep]):
        """Execute plan with real-time monitoring and adaptation"""
        for i, step in enumerate(plan):
            self.get_logger().info(f"Executing step {i+1}/{len(plan)}: {step.description}")

            # Execute step and monitor for issues
            execution_result = self.execute_monitored_step(step)

            if not execution_result['success']:
                # Handle failure - possibly refine and retry
                self.get_logger().warn(f"Step failed: {execution_result['error']}")

                # Learn from this failure
                self.adaptive_planner.learn_from_execution(
                    plan,
                    {**execution_result, 'failed_step_index': i}
                )

                # Try to refine and continue
                refined_plan = asyncio.run(
                    self.adaptive_planner.refine_plan(plan[i:], execution_result)
                )

                if refined_plan:
                    return self.execute_plan_with_monitoring(refined_plan)
                else:
                    return False

        return True
```

## Performance and Safety Considerations

```python
class SafeLLMPlanner:
    def __init__(self, llm_planner: LLMPlanner):
        self.llm_planner = llm_planner
        self.safety_constraints = [
            "Never harm humans",
            "Never damage property unnecessarily",
            "Always maintain balance",
            "Respect privacy",
            "Follow legal requirements"
        ]

    def validate_plan_safety(self, plan: List[PlanStep]) -> Dict[str, any]:
        """Validate that a plan meets safety requirements"""
        validation_result = {
            'is_safe': True,
            'violations': [],
            'warnings': []
        }

        for step in plan:
            # Check for unsafe actions
            if self.is_unsafe_action(step):
                validation_result['violations'].append({
                    'step': step.description,
                    'violation': 'Unsafe action detected'
                })
                validation_result['is_safe'] = False

            # Check for potential safety issues
            if self.has_safety_concern(step):
                validation_result['warnings'].append({
                    'step': step.description,
                    'concern': 'Potential safety concern'
                })

        return validation_result

    def is_unsafe_action(self, step: PlanStep) -> bool:
        """Check if an action is inherently unsafe"""
        unsafe_keywords = [
            'harm', 'damage', 'crash', 'hurt', 'injure',
            'violate', 'trespass', 'steal', 'destroy'
        ]

        action_lower = step.action.lower()
        description_lower = step.description.lower()

        for keyword in unsafe_keywords:
            if keyword in action_lower or keyword in description_lower:
                return True

        return False

    def has_safety_concern(self, step: PlanStep) -> bool:
        """Check if an action has potential safety concerns"""
        concern_keywords = [
            'fast', 'quickly', 'aggressive', 'forceful',
            'unstable', 'dangerous', 'risky', 'careless'
        ]

        description_lower = step.description.lower()
        for keyword in concern_keywords:
            if keyword in description_lower:
                return True

        return False
```

## Summary

Cognitive planning with LLMs enables humanoid robots to understand complex, high-level goals and generate detailed action plans. The integration requires careful consideration of context, safety constraints, and real-time adaptation to changing conditions. By combining hierarchical planning, context awareness, and adaptive refinement, robots can execute complex tasks while maintaining safety and efficiency.