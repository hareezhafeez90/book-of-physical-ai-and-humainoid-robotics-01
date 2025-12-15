# Dynamics and Control: Motion Planning for Humanoid Systems

## Introduction: From Kinematics to Dynamics

While kinematics describes the geometry of motion, dynamics considers the forces and torques that cause motion. For humanoid robots, dynamics is crucial because these systems must interact with the environment, maintain balance under external forces, and execute movements while respecting physical constraints. This section explores the mathematical models and control strategies that enable robust humanoid motion.

### Why Dynamics Matter for Humanoid Robots

Humanoid robots operate in dynamic environments where:
- External forces affect robot stability
- Robot motion generates internal forces and torques
- Balance must be maintained during movement
- Energy efficiency is critical for sustained operation

## Mathematical Models of Robot Dynamics

### The Euler-Lagrange Equation

The dynamics of a robotic system can be described by the Euler-Lagrange equation:

```
M(q)q_ddot + C(q, q_dot)q_dot + g(q) = τ
```

Where:
- `M(q)` is the mass/inertia matrix
- `C(q, q_dot)` contains Coriolis and centrifugal terms
- `g(q)` represents gravitational forces
- `τ` is the vector of joint torques
- `q`, `q_dot`, `q_ddot` are joint positions, velocities, and accelerations

### The Mass Matrix

The mass matrix `M(q)` represents the inertial properties of the robot as a function of configuration. It is symmetric and positive definite, meaning energy is always positive. For a humanoid robot, the mass matrix accounts for the distribution of mass in all links and how they interact through the kinematic structure.

### Coriolis and Centrifugal Terms

The term `C(q, q_dot)q_dot` represents forces due to velocity coupling between joints. These forces arise because the motion of one link affects the motion of other links through the kinematic chain.

### Gravity Vector

The term `g(q)` represents the gravitational forces acting on the robot in its current configuration. This includes both the weight of each link and the gravitational coupling between links.

## Control Strategies for Humanoid Systems

### Computed Torque Control (Inverse Dynamics Control)

Computed torque control uses the dynamic model to linearize the system:

```
τ = M(q)q_ddot_des + C(q, q_dot)q_dot + g(q) + Kp(q_des - q) + Kd(q_dot_des - q_dot)
```

This control law:
1. Computes the required torques for desired acceleration
2. Compensates for gravity and Coriolis effects
3. Adds PD feedback for error correction

### Operational Space Control

Operational space control focuses on task-space variables rather than joint-space variables:

```
F = Λ(x)(x_ddot_des - x_ddot) + Kp(x_des - x) + Kd(x_dot_des - x_dot)
τ = J^T F
```

Where:
- `Λ(x)` is the task-space inertia matrix
- `x` represents task-space variables (end-effector position, orientation)
- `J` is the Jacobian matrix

This approach is particularly useful for humanoid robots performing tasks in Cartesian space while maintaining balance.

### Python Implementation: Computed Torque Control

```python
import numpy as np

class HumanoidDynamics:
    def __init__(self, mass_matrix, coriolis_matrix, gravity_vector):
        self.M = mass_matrix
        self.C = coriolis_matrix
        self.g = gravity_vector
        self.Kp = np.diag([100, 100, 100, 50, 50, 50])  # Position gains
        self.Kd = np.diag([20, 20, 20, 10, 10, 10])    # Velocity gains

    def mass_matrix(self, q):
        """Calculate mass matrix M(q) - simplified for example"""
        # In practice, this would be calculated from robot URDF/URDF
        n = len(q)
        M = np.eye(n) * 2.0  # Simplified constant mass matrix
        # Add coupling terms based on kinematic structure
        for i in range(n-1):
            M[i, i+1] = 0.1
            M[i+1, i] = 0.1
        return M

    def coriolis_vector(self, q, q_dot):
        """Calculate Coriolis and centrifugal terms C(q, q_dot)q_dot"""
        # Simplified model - in practice this involves complex calculations
        Cq_dot = np.zeros_like(q_dot)
        for i in range(len(q)):
            # Simplified Coriolis terms
            if i < len(q) - 1:
                Cq_dot[i] = 0.5 * q_dot[i] * q_dot[i+1]
        return Cq_dot

    def gravity_vector(self, q):
        """Calculate gravity terms g(q)"""
        # Simplified gravity vector
        g = np.zeros_like(q)
        g[0] = 9.81 * 0.1  # Gravity effect on first joint
        return g

    def computed_torque_control(self, q, q_dot, q_des, q_dot_des, q_ddot_des):
        """Computed torque control law"""
        M = self.mass_matrix(q)
        Cq_dot = self.coriolis_vector(q, q_dot)
        g = self.gravity_vector(q)

        # Feedback terms
        pos_error = q_des - q
        vel_error = q_dot_des - q_dot

        # Control law
        tau = (M @ q_ddot_des +
               Cq_dot +
               g +
               self.Kp @ pos_error +
               self.Kd @ vel_error)

        return tau

    def operational_space_control(self, q, q_dot, x_des, x_dot_des, x_ddot_des, J, J_dot):
        """Operational space control"""
        # Calculate Jacobian transpose
        J_T = J.T

        # Calculate mass matrix in operational space
        M_inv = np.linalg.inv(self.mass_matrix(q))
        Lambda_inv = J @ M_inv @ J_T
        Lambda = np.linalg.inv(Lambda_inv)

        # Calculate bias forces in operational space
        Cq_dot = self.coriolis_vector(q, q_dot)
        g = self.gravity_vector(q)
        h_op = J @ M_inv @ (Cq_dot + g) - Lambda @ (J_dot @ q_dot)

        # Calculate operational space error
        x = self.forward_kinematics(q)  # Simplified
        x_error = x_des - x
        x_dot_error = x_dot_des - J @ q_dot

        # Control law in operational space
        F = (Lambda @ x_ddot_des +
             h_op +
             self.Kp[:len(x_error)] @ x_error +
             self.Kd[:len(x_dot_error)] @ x_dot_error)

        # Convert to joint torques
        tau = J_T @ F

        return tau
```

## Balance and Stability Control

### Zero-Moment Point (ZMP) Control

The Zero-Moment Point is a crucial concept for humanoid balance:

```
ZMP_x = x_com - (g/(z_com - z_ref)) * (x_com_ddot)
ZMP_y = y_com - (g/(z_com - z_ref)) * (y_com_ddot)
```

For stable walking, the ZMP must remain within the support polygon (typically the foot area).

### Center of Mass (CoM) Control

CoM control strategies maintain balance by regulating the position and acceleration of the robot's center of mass:

```python
def zmp_controller(current_com, current_com_vel, current_com_acc,
                   desired_zmp, kp=1.0, kd=1.0):
    """ZMP-based balance controller"""
    # Calculate desired CoM acceleration to achieve desired ZMP
    z_ref = 0.8  # Height reference (m)
    g = 9.81

    # ZMP error
    zmp_current = current_com - (z_ref / g) * current_com_acc
    zmp_error = desired_zmp - zmp_current

    # Control law for CoM acceleration
    com_acc_des = (g / z_ref) * (current_com - desired_zmp) + kp * zmp_error + kd * (0 - current_com_vel)

    return com_acc_des
```

### Preview Control

Preview control uses future reference trajectories to improve balance:

```python
def preview_control(zmp_reference, preview_horizon=20, dt=0.01):
    """Preview control for ZMP tracking"""
    # Calculate preview control gains (simplified)
    A = np.array([[0, 1], [g/1.0, 0]])  # Simplified for 1D
    B = np.array([0, g/1.0])

    # Solve Riccati equation for LQR
    # In practice, this would involve more complex calculations
    K = np.array([1.0, 1.0])  # Feedback gains

    # Calculate preview gains
    preview_gains = []
    for i in range(preview_horizon):
        # Calculate future influence
        gain = np.exp(-i * dt)  # Simplified preview gain
        preview_gains.append(gain)

    return preview_gains
```

## Walking Pattern Generation

### Inverted Pendulum Model

The linear inverted pendulum model (LIPM) is commonly used for humanoid walking:

```
x_ddot = ω²(x - x_zmp)
```

Where `ω² = g/h` and `h` is the CoM height.

### Footstep Planning

Walking requires careful footstep planning to maintain stability:

```python
def generate_walk_pattern(step_length=0.3, step_width=0.2, step_height=0.1,
                         step_time=0.8, n_steps=10):
    """Generate walking pattern parameters"""
    footsteps = []

    for i in range(n_steps):
        # Alternate between left and right foot
        side_offset = step_width/2 if i % 2 == 0 else -step_width/2

        # Calculate foot position
        x_pos = (i + 1) * step_length
        y_pos = side_offset
        z_pos = 0  # Ground level

        # Add timing information
        step_timing = i * step_time

        footsteps.append({
            'position': [x_pos, y_pos, z_pos],
            'timing': step_timing,
            'step_height': step_height,
            'step_time': step_time
        })

    return footsteps
```

## Advanced Control Techniques

### Model Predictive Control (MPC)

MPC optimizes control inputs over a finite horizon while considering constraints:

```python
def mpc_controller(state, reference_trajectory, prediction_horizon=10,
                   control_horizon=5, dt=0.01):
    """Model Predictive Control for humanoid systems"""

    # Define cost function
    Q = np.eye(len(state)) * 1.0    # State tracking cost
    R = np.eye(3) * 0.1            # Control effort cost
    P = np.eye(len(state)) * 10.0   # Terminal cost

    # Linearize system around current state
    A, B = linearize_dynamics(state)

    # Set up optimization problem
    # Minimize: sum(x_k^T Q x_k + u_k^T R u_k) + x_N^T P x_N
    # Subject to: x_k+1 = A x_k + B u_k

    # In practice, this would use a QP solver
    # For this example, we'll return a simplified control
    state_error = reference_trajectory[0] - state
    control_input = -np.linalg.inv(R) @ B.T @ state_error

    return control_input
```

### Adaptive Control

Adaptive control adjusts parameters based on system behavior:

```python
class AdaptiveController:
    def __init__(self, initial_params):
        self.params = initial_params
        self.learning_rate = 0.01

    def update_params(self, tracking_error, regressor):
        """Update adaptive parameters"""
        # Parameter update law
        param_update = self.learning_rate * tracking_error * regressor
        self.params += param_update
        return self.params

    def control(self, state, desired_state):
        """Adaptive control law"""
        error = desired_state - state
        control_output = self.params @ error
        return control_output
```

## Real-Time Implementation Considerations

### Computational Complexity

Humanoid control systems must operate in real-time (typically 1-10ms control cycles). This requires:
- Efficient algorithms
- Simplified models where possible
- Parallel processing for complex calculations

### Sensor Integration

Real-time control requires integration of multiple sensor sources:
- Joint encoders for position feedback
- IMU for orientation and acceleration
- Force/torque sensors for contact detection
- Vision systems for environment perception

## Conclusion

Dynamics and control form the foundation of robust humanoid robot operation. The mathematical models of robot dynamics enable precise control of movement while accounting for physical constraints and environmental interactions. Control strategies like computed torque control, operational space control, and balance-aware algorithms allow humanoid robots to perform complex tasks while maintaining stability.

The next section will focus specifically on balance and stability control, which is critical for humanoid robots that must maintain upright posture during various activities.