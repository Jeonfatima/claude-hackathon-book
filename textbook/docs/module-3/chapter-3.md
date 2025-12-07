---
id: module-3-chapter-3
slug: /module-3/chapter-3
title: "Chapter 3: Path Planning & Bipedal Movement"
sidebar_position: 4
---

# Chapter 3: Path Planning & Bipedal Movement

## Introduction

Path planning and bipedal movement are critical capabilities for humanoid robots operating in complex environments. This chapter explores how to implement sophisticated path planning algorithms and bipedal locomotion control, with a focus on leveraging Isaac ROS and other advanced tools for efficient computation.

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement advanced path planning algorithms for humanoid robots
- Design bipedal locomotion controllers
- Integrate path planning with bipedal movement systems
- Optimize movement for stability and efficiency

## Path Planning Fundamentals

Path planning for humanoid robots must consider:
- Kinematic constraints of bipedal systems
- Dynamic stability requirements
- Obstacle avoidance in 3D environments
- Energy efficiency optimization

### Configuration Space for Humanoid Robots

```python
import numpy as np
from scipy.spatial import distance
import matplotlib.pyplot as plt

class HumanoidConfigurationSpace:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.joint_limits = robot_model.get_joint_limits()
        self.dimensions = robot_model.get_dof()

    def is_collision_free(self, configuration, obstacles):
        """Check if a configuration is collision-free"""
        # Calculate robot pose from configuration
        robot_poses = self.robot_model.forward_kinematics(configuration)

        # Check for collisions with obstacles
        for pose in robot_poses:
            for obstacle in obstacles:
                if self.check_collision(pose, obstacle):
                    return False
        return True

    def check_collision(self, pose, obstacle):
        """Check collision between robot pose and obstacle"""
        # Implementation depends on geometry representation
        dist = distance.euclidean(pose.position, obstacle.position)
        return dist < (pose.radius + obstacle.radius)
```

### RRT-Based Path Planning for Humanoids

```python
import numpy as np
import random
from scipy.spatial.distance import euclidean

class HumanoidRRTPlanner:
    def __init__(self, config_space, start_config, goal_config):
        self.config_space = config_space
        self.start_config = np.array(start_config)
        self.goal_config = np.array(goal_config)
        self.nodes = [self.start_config]
        self.edges = {}
        self.step_size = 0.1
        self.goal_bias = 0.1

    def plan_path(self, max_iterations=1000):
        for i in range(max_iterations):
            # Sample random configuration
            if random.random() < self.goal_bias:
                q_rand = self.goal_config
            else:
                q_rand = self.sample_configuration()

            # Find nearest node
            q_near = self.nearest_node(q_rand)

            # Extend towards random configuration
            q_new = self.extend_towards(q_near, q_rand)

            if q_new is not None:
                # Add new node to tree
                self.add_node(q_new, q_near)

                # Check if goal is reached
                if self.is_goal_reached(q_new):
                    return self.reconstruct_path(q_new)

        return None  # No path found

    def sample_configuration(self):
        """Sample random configuration within joint limits"""
        config = []
        for limit in self.config_space.joint_limits:
            config.append(random.uniform(limit[0], limit[1]))
        return np.array(config)

    def nearest_node(self, q_rand):
        """Find nearest node in the tree"""
        min_dist = float('inf')
        nearest = None
        for node in self.nodes:
            dist = euclidean(q_rand, node)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest

    def extend_towards(self, q_from, q_to):
        """Extend from q_from towards q_to"""
        direction = q_to - q_from
        norm = np.linalg.norm(direction)

        if norm <= self.step_size:
            q_new = q_to
        else:
            q_new = q_from + (direction / norm) * self.step_size

        # Check if new configuration is valid
        if self.config_space.is_collision_free(q_new):
            return q_new
        return None

    def is_goal_reached(self, q_current):
        """Check if goal is reached"""
        return euclidean(q_current, self.goal_config) < self.step_size

    def add_node(self, q_new, q_parent):
        """Add new node to the tree"""
        self.nodes.append(q_new)
        self.edges[tuple(q_new)] = tuple(q_parent)

    def reconstruct_path(self, q_final):
        """Reconstruct path from start to goal"""
        path = [q_final]
        current = q_final

        while not np.array_equal(current, self.start_config):
            parent = np.array(self.edges[tuple(current)])
            path.append(parent)
            current = parent

        return path[::-1]  # Reverse to get start->goal path
```

## Bipedal Movement Control

### Inverse Kinematics for Bipedal Systems

```python
import numpy as np
from scipy.optimize import minimize

class BipedalIKSolver:
    def __init__(self, robot_model):
        self.robot_model = robot_model

    def solve_leg_ik(self, target_position, target_orientation, leg_chain):
        """Solve inverse kinematics for a leg"""
        def ik_objective(joint_angles):
            # Calculate current end-effector pose
            current_pose = self.robot_model.forward_kinematics_leg(
                joint_angles, leg_chain
            )

            # Calculate error between target and current pose
            pos_error = np.linalg.norm(target_position - current_pose[:3])
            orient_error = self.calculate_orientation_error(
                target_orientation, current_pose[3:]
            )

            return pos_error + 0.1 * orient_error

        # Initial guess (current joint angles)
        initial_guess = self.robot_model.get_current_leg_angles(leg_chain)

        # Solve optimization problem
        result = minimize(
            ik_objective,
            initial_guess,
            method='BFGS',
            options={'maxiter': 100}
        )

        return result.x if result.success else initial_guess

    def calculate_orientation_error(self, target_quat, current_quat):
        """Calculate orientation error using quaternion distance"""
        # Quaternion dot product
        dot_product = np.dot(target_quat, current_quat)
        # Take absolute value to account for antipodal equivalence
        dot_product = np.clip(abs(dot_product), 0, 1)
        # Convert to angle error
        return 2 * np.arccos(dot_product)
```

### Walking Pattern Generation

```python
import numpy as np

class WalkingPatternGenerator:
    def __init__(self, robot_params):
        self.step_length = robot_params['step_length']
        self.step_height = robot_params['step_height']
        self.step_duration = robot_params['step_duration']
        self.zmp_reference = robot_params['zmp_reference']

    def generate_foot_trajectory(self, start_pos, goal_pos, step_count):
        """Generate foot trajectory for walking"""
        trajectories = []

        for i in range(step_count):
            # Calculate step position
            step_x = start_pos[0] + i * self.step_length
            step_y = start_pos[1] if i % 2 == 0 else goal_pos[1]  # Alternating feet

            # Generate trajectory for this step
            step_trajectory = self.generate_single_step(
                [step_x, step_y, 0],  # Start position
                [step_x + self.step_length, step_y, 0]  # End position
            )
            trajectories.append(step_trajectory)

        return trajectories

    def generate_single_step(self, start_pos, end_pos):
        """Generate trajectory for a single step"""
        t = np.linspace(0, self.step_duration, int(self.step_duration * 100))
        trajectory = []

        for time in t:
            # Interpolate x position
            x = np.interp(time, [0, self.step_duration/2, self.step_duration],
                         [start_pos[0], (start_pos[0] + end_pos[0])/2, end_pos[0]])

            # Interpolate y position
            y = start_pos[1]  # Keep constant for this step

            # Generate step height profile (parabolic)
            if time < self.step_duration / 2:
                z = start_pos[2] + self.step_height * (2 * time / self.step_duration) ** 2
            else:
                z = start_pos[2] + self.step_height * (2 - 2 * time / self.step_duration) ** 2

            trajectory.append([x, y, z])

        return trajectory
```

### Balance Control for Bipedal Robots

```python
import numpy as np

class BalanceController:
    def __init__(self, robot_mass, gravity=9.81):
        self.mass = robot_mass
        self.gravity = gravity
        self.com_position = np.zeros(3)  # Center of mass position
        self.com_velocity = np.zeros(3)
        self.zmp_position = np.zeros(2)  # Zero Moment Point
        self.kp = 10.0  # Proportional gain
        self.kd = 2.0   # Derivative gain

    def compute_balance_control(self, desired_com, current_com, current_com_vel):
        """Compute control forces for balance"""
        # Calculate error
        pos_error = desired_com[:2] - current_com[:2]
        vel_error = -current_com_vel[:2]  # Desired velocity is 0

        # PID control for balance
        control_force = self.kp * pos_error + self.kd * vel_error

        # Adjust ZMP to maintain balance
        self.zmp_position = self.calculate_zmp(control_force)

        return control_force

    def calculate_zmp(self, applied_force):
        """Calculate Zero Moment Point from applied force"""
        # ZMP = CoM - (g/CoM_z) * (CoM_acceleration / gravity)
        # Simplified calculation
        zmp_x = self.com_position[0] - applied_force[0] / (self.mass * self.gravity)
        zmp_y = self.com_position[1] - applied_force[1] / (self.mass * self.gravity)

        return np.array([zmp_x, zmp_y])

    def update_com_dynamics(self, control_force, dt):
        """Update center of mass dynamics"""
        # Apply control force to CoM
        acceleration = control_force / self.mass

        # Update velocity and position
        self.com_velocity[:2] += acceleration * dt
        self.com_position[:2] += self.com_velocity[:2] * dt
```

## Integration of Path Planning and Movement

### Hierarchical Control Architecture

```python
import numpy as np

class HierarchicalBipedalController:
    def __init__(self, robot_model):
        self.path_planner = HumanoidRRTPlanner(robot_model)
        self.ik_solver = BipedalIKSolver(robot_model)
        self.walk_generator = WalkingPatternGenerator({
            'step_length': 0.3,
            'step_height': 0.1,
            'step_duration': 1.0,
            'zmp_reference': np.array([0, 0])
        })
        self.balance_controller = BalanceController(robot_model.mass)

        self.current_path = []
        self.path_index = 0
        self.movement_state = 'standing'  # standing, walking, turning

    def execute_navigation(self, start_pose, goal_pose):
        """Execute navigation from start to goal"""
        # Plan path
        path = self.path_planner.plan_path(start_pose, goal_pose)

        if path is None:
            self.get_logger().error("No path found!")
            return False

        self.current_path = path
        self.path_index = 0

        # Execute path following
        success = self.follow_path()
        return success

    def follow_path(self):
        """Follow the planned path with bipedal locomotion"""
        while self.path_index < len(self.current_path) - 1:
            current_waypoint = self.current_path[self.path_index]
            next_waypoint = self.current_path[self.path_index + 1]

            # Generate walking pattern to next waypoint
            walk_pattern = self.walk_generator.generate_foot_trajectory(
                current_waypoint, next_waypoint, steps=10
            )

            # Execute walking pattern
            for step in walk_pattern:
                if not self.execute_step(step):
                    return False  # Failed to execute step

            self.path_index += 1

        return True

    def execute_step(self, step_trajectory):
        """Execute a single walking step"""
        for point in step_trajectory:
            # Solve inverse kinematics for this foot position
            joint_angles = self.ik_solver.solve_leg_ik(
                point, self.get_desired_foot_orientation(), 'left_leg'
            )

            # Send joint commands to robot
            success = self.send_joint_commands(joint_angles)

            if not success:
                return False

            # Update balance
            self.update_balance()

        return True

    def update_balance(self):
        """Update robot balance during movement"""
        current_com = self.get_current_com()
        current_com_vel = self.get_current_com_velocity()
        desired_com = self.get_desired_com()

        control_force = self.balance_controller.compute_balance_control(
            desired_com, current_com, current_com_vel
        )

        # Apply balance control
        self.apply_balance_control(control_force)

    def send_joint_commands(self, joint_angles):
        """Send joint angle commands to robot"""
        # Implementation would send commands via ROS or other interface
        return True  # Placeholder

    def get_current_com(self):
        """Get current center of mass position"""
        # Implementation would get from robot state
        return np.zeros(3)  # Placeholder

    def get_desired_com(self):
        """Get desired center of mass position"""
        # Implementation would calculate from walking pattern
        return np.zeros(3)  # Placeholder
```

## Optimization Techniques

### Trajectory Optimization for Bipedal Movement

```python
import numpy as np
from scipy.optimize import minimize

class BipedalTrajectoryOptimizer:
    def __init__(self, robot_model):
        self.robot_model = robot_model

    def optimize_trajectory(self, waypoints, constraints):
        """Optimize trajectory for energy efficiency and stability"""
        def trajectory_cost(trajectory_params):
            # Calculate trajectory from parameters
            trajectory = self.params_to_trajectory(trajectory_params, waypoints)

            # Calculate various costs
            energy_cost = self.calculate_energy_cost(trajectory)
            stability_cost = self.calculate_stability_cost(trajectory)
            smoothness_cost = self.calculate_smoothness_cost(trajectory)

            # Weighted sum of costs
            total_cost = (0.4 * energy_cost +
                         0.4 * stability_cost +
                         0.2 * smoothness_cost)

            return total_cost

        # Optimize trajectory
        result = minimize(
            trajectory_cost,
            x0=self.initial_trajectory_params(waypoints),
            method='SLSQP',
            constraints=constraints
        )

        return self.params_to_trajectory(result.x, waypoints)

    def calculate_energy_cost(self, trajectory):
        """Calculate energy consumption along trajectory"""
        energy = 0.0
        for i in range(1, len(trajectory)):
            # Approximate energy as joint torque * velocity
            joint_velocities = (trajectory[i] - trajectory[i-1]) / 0.01  # dt = 0.01s
            energy += np.sum(np.abs(joint_velocities))  # Simplified model
        return energy

    def calculate_stability_cost(self, trajectory):
        """Calculate stability cost based on ZMP"""
        stability_cost = 0.0
        for config in trajectory:
            # Calculate ZMP for this configuration
            zmp = self.calculate_zmp_for_config(config)
            # Penalize ZMP outside support polygon
            stability_cost += self.penalty_for_zmp_outside_support(zmp)
        return stability_cost
```

## Performance Considerations

For efficient path planning and movement:

- Use sampling-based planners for high-DOF humanoid systems
- Implement hierarchical control (global planning, local control)
- Optimize inverse kinematics with GPU acceleration
- Use model predictive control for dynamic balancing
- Consider terrain characteristics in path planning

## Summary

Path planning and bipedal movement for humanoid robots require sophisticated algorithms that consider both kinematic constraints and dynamic stability. By integrating advanced planning algorithms with robust control systems, humanoid robots can navigate complex environments while maintaining balance and efficiency.