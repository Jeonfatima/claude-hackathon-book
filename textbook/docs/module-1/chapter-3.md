---
id: module-1-chapter-3
slug: /module-1/chapter-3
title: "Chapter 3: Python Agents with ROS Controllers"
sidebar_position: 4
---

# Chapter 3: Python Agents with ROS Controllers

## Introduction

Python agents provide a powerful way to implement complex behaviors and control strategies in ROS 2. This chapter explores how to create Python-based agents that interact with ROS controllers to manage robotic systems.

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement Python agents for robotic control
- Interface with ROS controllers
- Design behavior-based architectures
- Create adaptive control strategies

## Basic Python Agent Structure

A Python agent in ROS 2 typically includes:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class RobotAgent(Node):
    def __init__(self):
        super().__init__('robot_agent')
        
        # Initialize controllers and action clients
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            'controller_name/follow_joint_trajectory'
        )
        
        # Set up callback groups for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
```

## Controller Interfaces

ROS 2 controllers typically use the following interfaces:
- Joint trajectory controllers for position/velocity control
- Cartesian controllers for end-effector control
- Impedance controllers for compliant manipulation

## Example: Joint Trajectory Agent

```python
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointTrajectoryAgent(Node):
    def __init__(self):
        super().__init__('joint_trajectory_agent')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory'
        )
    
    def send_goal(self, positions, velocities, time_from_start):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.time_from_start.sec = time_from_start
        goal_msg.trajectory.points = [point]
        
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
```

## Behavior-Based Robotics

Python agents can implement behavior-based robotics principles:

```python
class BehaviorBasedAgent(Node):
    def __init__(self):
        super().__init__('behavior_agent')
        self.behaviors = {
            'explore': self.explore_behavior,
            'avoid': self.avoid_behavior,
            'grasp': self.grasp_behavior
        }
        self.current_behavior = 'explore'
    
    def select_behavior(self, sensor_data):
        # Simple priority-based behavior selection
        if self.detect_obstacle(sensor_data):
            return 'avoid'
        elif self.detect_object(sensor_data):
            return 'grasp'
        else:
            return 'explore'
```

## Summary

Python agents provide a flexible framework for implementing complex robotic behaviors that interact with ROS controllers. By combining action clients, service calls, and topic subscriptions, agents can orchestrate sophisticated robotic operations.
