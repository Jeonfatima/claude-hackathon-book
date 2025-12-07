---
id: module-1-chapter-4
slug: /module-1/chapter-4
title: "Chapter 4: URDF for Humanoids"
sidebar_position: 5
---

# Chapter 4: URDF for Humanoids

## Introduction

Unified Robot Description Format (URDF) is an XML format used to describe robotic systems in ROS. For humanoid robots, URDF becomes particularly important as it defines the complex kinematic chains and physical properties of bipedal systems.

## Learning Objectives

By the end of this chapter, you will be able to:
- Create URDF files for humanoid robots
- Define kinematic chains for bipedal systems
- Specify physical properties and collision geometries
- Visualize humanoid robots in RViz and Gazebo

## URDF Basics for Humanoids

A humanoid URDF typically includes:
- A base link (usually the pelvis or torso)
- Kinematic chains for legs, arms, and head
- Joint definitions with appropriate limits
- Visual and collision geometries

## Basic Humanoid Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Left leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="0 -0.1 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

## Kinematic Chains for Bipedal Systems

Humanoid robots typically have multiple kinematic chains:
- Left leg chain (hip → knee → ankle)
- Right leg chain (hip → knee → ankle)
- Left arm chain (shoulder → elbow → wrist)
- Right arm chain (shoulder → elbow → wrist)
- Head/neck chain

## Advanced URDF Features for Humanoids

### Transmission Elements

```xml
<transmission name="left_hip_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_hip_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_hip_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo-Specific Elements

```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

## Xacro for Complex Humanoid Models

For complex humanoid robots, Xacro (XML Macros) is often used:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  <xacro:property name="M_PI" value="3.14159"/>

  <xacro:macro name="leg" params="prefix reflect">
    <joint name="${prefix}_hip_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_thigh"/>
      <origin xyz="0 ${-0.1*reflect} -0.2" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>
    <!-- Additional leg links and joints -->
  </xacro:macro>

  <xacro:leg prefix="left" reflect="1"/>
  <xacro:leg prefix="right" reflect="-1"/>
</robot>
```

## Summary

URDF is essential for defining humanoid robots in ROS-based systems. Properly designed URDF files enable accurate simulation, visualization, and control of complex bipedal systems.
