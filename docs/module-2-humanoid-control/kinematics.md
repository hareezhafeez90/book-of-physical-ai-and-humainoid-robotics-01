# Humanoid Kinematics: Forward and Inverse Kinematics

## Introduction: The Geometry of Movement

Kinematics is the study of motion without considering the forces that cause it. In humanoid robotics, kinematics provides the mathematical framework for understanding how joint angles relate to end-effector positions and how to plan movements through space. This section covers both forward kinematics (predicting position from joint angles) and inverse kinematics (determining joint angles for desired positions) - fundamental concepts for controlling humanoid robots.

### Why Kinematics Matters for Humanoid Robots

Humanoid robots are complex kinematic chains with multiple degrees of freedom (DOF) arranged in a human-like structure. Understanding kinematics is crucial because:

- **Redundancy**: Humanoid robots often have more joints than strictly necessary to reach a position, providing flexibility in motion planning
- **Balance**: Maintaining balance requires coordinated movement of multiple limbs and the torso
- **Obstacle Avoidance**: Kinematic constraints must be considered when planning paths around obstacles
- **Workspace Analysis**: Understanding the reachable workspace helps in task planning

## Forward Kinematics: From Joints to Position

Forward kinematics calculates the position and orientation of the end-effector (e.g., hand or foot) given the joint angles. For humanoid robots, this involves multiple kinematic chains.

### Mathematical Foundation: Denavit-Hartenberg Parameters

The Denavit-Hartenberg (DH) convention provides a systematic way to define coordinate frames for each joint in a kinematic chain:

1. **Z-axis**: Aligned with the joint axis of rotation or translation
2. **X-axis**: Perpendicular to both the current and previous z-axes
3. **Y-axis**: Completes the right-handed coordinate system

For each joint `i`, four parameters define the transformation:
- `a_i`: Link length (distance along x_i from z_(i-1) to z_i)
- `α_i`: Link twist (angle from z_(i-1) to z_i about x_i)
- `d_i`: Link offset (distance along z_(i-1) from x_(i-1) to x_i)
- `θ_i`: Joint angle (angle from x_(i-1) to x_i about z_(i-1))

### Homogeneous Transformation Matrices

Each joint transformation is represented by a 4x4 homogeneous transformation matrix:

```
T_i = [
    cos(θ_i)   -sin(θ_i)*cos(α_i)   sin(θ_i)*sin(α_i)   a_i*cos(θ_i)
    sin(θ_i)    cos(θ_i)*cos(α_i)   -cos(θ_i)*sin(α_i)   a_i*sin(θ_i)
    0           sin(α_i)             cos(α_i)            d_i
    0           0                    0                   1
]
```

The complete transformation from base to end-effector is:
```
T_total = T_1 * T_2 * ... * T_n
```

### Python Implementation Example

```python
import numpy as np

def dh_transform(theta, d, a, alpha):
    """Calculate DH transformation matrix"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    T = np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])
    return T

def forward_kinematics(joint_angles, dh_params):
    """Calculate forward kinematics for a kinematic chain"""
    T_total = np.eye(4)

    for i, theta in enumerate(joint_angles):
        d, a, alpha = dh_params[i]
        T_i = dh_transform(theta, d, a, alpha)
        T_total = np.dot(T_total, T_i)

    return T_total

# Example: Simple 2-DOF planar arm
joint_angles = [np.pi/4, np.pi/6]  # 45° and 30°
dh_params = [
    (0, 1.0, 0),    # Joint 1: d=0, a=1.0, alpha=0
    (0, 1.0, 0)     # Joint 2: d=0, a=1.0, alpha=0
]

end_effector_pose = forward_kinematics(joint_angles, dh_params)
print(f"End-effector pose:\n{end_effector_pose}")
```

## Inverse Kinematics: From Position to Joints

Inverse kinematics (IK) determines the joint angles required to achieve a desired end-effector position and orientation. This is typically more challenging than forward kinematics and may have multiple solutions or no solution.

### Analytical vs. Numerical Methods

**Analytical Methods**:
- Provide exact solutions when they exist
- Computationally efficient
- Limited to specific kinematic structures (e.g., 6-DOF with intersecting wrist)

**Numerical Methods**:
- Applicable to arbitrary kinematic structures
- Iterative approaches that converge to solutions
- Can handle redundant systems

### Jacobian-Based IK

The Jacobian matrix relates joint velocities to end-effector velocities:

```
v_ee = J(q) * q_dot
```

Where:
- `v_ee` is the end-effector velocity vector
- `J(q)` is the Jacobian matrix
- `q_dot` is the joint velocity vector

To solve for joint angles:
```
q_dot = J^(-1) * v_ee
```

For redundant systems (more joints than necessary), we use the pseudoinverse:
```
q_dot = J^+ * v_ee
```

### Python Implementation: Jacobian Transpose Method

```python
import numpy as np

def jacobian_transpose(robot, joint_angles, target_pos, end_effector_link):
    """Calculate joint angles using Jacobian transpose method"""
    max_iterations = 100
    step_size = 0.01
    tolerance = 0.001

    for i in range(max_iterations):
        current_pos = forward_kinematics(robot, joint_angles)[0:3, 3]
        error = target_pos - current_pos

        if np.linalg.norm(error) < tolerance:
            break

        # Calculate Jacobian (simplified for this example)
        J = calculate_jacobian(robot, joint_angles, end_effector_link)

        # Jacobian transpose method
        joint_delta = step_size * np.dot(J.T, error)
        joint_angles += joint_delta

    return joint_angles

def calculate_jacobian(robot, joint_angles, end_effector_link):
    """Calculate the geometric Jacobian matrix"""
    n_joints = len(joint_angles)
    J = np.zeros((6, n_joints))  # 6 DOF: 3 translational, 3 rotational

    # Get current end-effector position
    T_end = forward_kinematics(robot, joint_angles)
    end_pos = T_end[0:3, 3]

    # Calculate for each joint
    for i in range(n_joints):
        # Get joint position and axis
        T_joint = forward_kinematics(robot, joint_angles[:i+1])
        joint_pos = T_joint[0:3, 3]
        joint_axis = T_joint[0:3, 2]  # z-axis of joint frame

        # Linear velocity component
        r = end_pos - joint_pos
        J[0:3, i] = np.cross(joint_axis, r)

        # Angular velocity component
        J[3:6, i] = joint_axis

    return J
```

## Humanoid-Specific Considerations

### Multi-Chain Kinematics

Humanoid robots have multiple kinematic chains (arms, legs) that must be coordinated:

- **Whole-Body IK**: Solving IK for all chains simultaneously to maintain balance and coordination
- **Task Prioritization**: Handling multiple simultaneous tasks (e.g., reaching while maintaining balance)
- **Constraint Handling**: Maintaining joint limits and avoiding self-collisions

### Redundancy Resolution

Humanoid robots are typically redundant (more DOF than task requirements), requiring strategies to resolve redundancy:

- **Null Space Projection**: Maintaining secondary objectives while achieving primary tasks
- **Joint Centering**: Keeping joints near neutral positions
- **Obstacle Avoidance**: Using redundancy to avoid collisions

### Balance-Aware Kinematics

Humanoid kinematics must consider balance constraints:

- **Zero-Moment Point (ZMP)**: Ensuring the robot's center of pressure remains within the support polygon
- **Center of Mass (CoM)**: Maintaining CoM within stable regions
- **Support Polygon**: Defining stable foot positions for single/double support

## Practical Applications in Humanoid Control

### Walking Pattern Generation

Walking involves complex kinematic coordination:
- Swing leg trajectory planning
- Stance leg support
- Pelvis and trunk coordination
- Arm swing for balance

### Manipulation Tasks

Humanoid manipulation requires:
- Reaching motion planning
- Grasp pose optimization
- Dual-arm coordination
- Whole-body motion planning

## Conclusion

Kinematics provides the mathematical foundation for understanding and controlling humanoid robot motion. Forward kinematics enables prediction of robot positions from joint angles, while inverse kinematics allows for task-space control by determining required joint angles. For humanoid robots, the complexity of multi-chain systems, redundancy, and balance constraints make kinematic control particularly challenging but essential for achieving human-like movement capabilities.

The next section will explore dynamics and control, which considers the forces and torques required to achieve these movements while accounting for the physics of motion.