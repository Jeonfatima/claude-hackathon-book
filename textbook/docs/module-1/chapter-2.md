---
id: module-1-chapter-2
slug: /module-1/chapter-2
title: "Chapter 2: Nodes, Topics, Services, and Actions"
sidebar_position: 3
---

# Chapter 2: Nodes, Topics, and Services

## Introduction

In ROS 2, the fundamental building blocks of robotic applications are nodes, topics, and services. Understanding how these components interact is crucial for developing effective robotic systems.

## Learning Objectives

By the end of this chapter, you will be able to:
- Create and manage ROS 2 nodes
- Implement publisher-subscriber communication patterns
- Design service-server and service-client interactions
- Understand the lifecycle of ROS 2 communications

## Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are organized into a graph and perform computation. Here's a basic node structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here
```

## Topics and Publishers/Subscribers

Topics enable asynchronous communication between nodes using a publish/subscribe model:

```python
# Publisher example
self.publisher = self.create_publisher(String, 'topic_name', 10)

# Subscriber example
self.subscriber = self.create_subscription(
    String,
    'topic_name',
    self.listener_callback,
    10
)
```

## Services

Services provide synchronous request/response communication:

```python
# Service server
self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

# Service client
self.cli = self.create_client(AddTwoInts, 'add_two_ints')
```

## Quality of Service (QoS)

ROS 2 provides QoS profiles to specify communication requirements:

```python
from rclpy.qos import QoSProfile

qos_profile = QoSProfile(depth=10)
publisher = self.create_publisher(String, 'topic', qos_profile)
```

## Summary

Nodes, topics, and services form the core communication patterns in ROS 2. Understanding these concepts is essential for designing distributed robotic applications.
