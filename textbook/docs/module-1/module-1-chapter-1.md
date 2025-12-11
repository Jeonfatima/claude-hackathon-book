---
title: "Chapter 1: ROS 2 Architecture"
sidebar_position: 2
---

# Chapter 1: ROS 2 Architecture

## Introduction

The Robot Operating System 2 (ROS 2) represents a significant evolution from its predecessor, addressing many of the architectural limitations of ROS 1. This chapter provides a comprehensive overview of the ROS 2 architecture and its key components.

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the key differences between ROS 1 and ROS 2
- Describe the DDS-based communication layer
- Understand the client library implementations
- Identify the main architectural components of ROS 2

## Key Architectural Changes

### Middleware Infrastructure

ROS 2 uses Data Distribution Service (DDS) as its middleware, providing:
- Improved real-time performance
- Better support for multi-robot systems
- Enhanced security features
- Platform and language independence

### Communication Layer

The communication layer in ROS 2 provides:
- Topics for asynchronous communication
- Services for synchronous request-response patterns
- Actions for long-running tasks with feedback
- Parameters for configuration management

## Practical Implementation

Let's examine a basic ROS 2 architecture:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Summary

ROS 2's architecture provides a robust foundation for developing complex robotic systems with improved performance, security, and scalability compared to ROS 1. The DDS-based middleware enables better real-time capabilities and multi-robot coordination.
