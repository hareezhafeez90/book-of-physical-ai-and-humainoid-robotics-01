# Practical Examples and Exercises: Humanoid Control Implementation

## Introduction: From Theory to Practice

This section provides hands-on examples and exercises that demonstrate the implementation of humanoid control concepts using Python and ROS 2. Through these practical examples, students will gain experience implementing the theoretical concepts covered in previous sections, from basic joint control to complex whole-body behaviors.

## Exercise 1: Simple Joint Control with PID

Let's start with a basic PID joint controller implementation:

```python
import numpy as np
import time
import matplotlib.pyplot as plt

class SimpleJointController:
    def __init__(self, kp=50, ki=10, kd=5):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain

        self.integral = 0
        self.previous_error = 0
        self.max_integral = 10  # Anti-windup limit
        self.max_torque = 50    # Torque limit (Nm)

    def update(self, desired_pos, current_pos, dt):
        """Update controller and return torque command"""
        # Calculate error
        error = desired_pos - current_pos

        # Update integral with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)

        # Calculate derivative
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error

        # Calculate control output
        output = (self.kp * error +
                 self.ki * self.integral +
                 self.kd * derivative)

        # Apply torque limits
        output = np.clip(output, -self.max_torque, self.max_torque)

        return output

# Simulation of a simple joint
class SimpleJoint:
    def __init__(self, inertia=1.0, damping=0.1):
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.inertia = inertia
        self.damping = damping

    def update(self, torque, dt):
        """Update joint dynamics"""
        # Calculate acceleration: T = I*alpha => alpha = T/I
        self.acceleration = (torque - self.damping * self.velocity) / self.inertia

        # Integrate to get velocity and position
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt

        return self.position, self.velocity

# Run simulation
def run_joint_control_simulation():
    controller = SimpleJointController(kp=100, ki=20, kd=10)
    joint = SimpleJoint(inertia=0.5, damping=0.05)

    dt = 0.001  # 1ms control loop
    simulation_time = 5.0  # 5 seconds
    steps = int(simulation_time / dt)

    # Desired trajectory: step input
    desired_positions = [0.0 if t < 1.0 else 1.0 for t in np.arange(0, simulation_time, dt)]

    # Store results for plotting
    times = []
    actual_positions = []
    desired_pos_log = []
    torques = []

    for i, t in enumerate(np.arange(0, simulation_time, dt)):
        # Get desired position
        desired_pos = desired_positions[i]

        # Get current position from joint
        current_pos = joint.position

        # Calculate control command
        torque = controller.update(desired_pos, current_pos, dt)

        # Apply torque to joint
        joint.update(torque, dt)

        # Log data
        times.append(t)
        actual_positions.append(joint.position)
        desired_pos_log.append(desired_pos)
        torques.append(torque)

    # Plot results
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(times, desired_pos_log, 'r--', label='Desired Position')
    plt.plot(times, actual_positions, 'b-', label='Actual Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.title('PID Joint Control Simulation')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(times, torques, 'g-', label='Applied Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (Nm)')
    plt.title('Control Torque')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    print(f"Final position error: {abs(desired_positions[-1] - actual_positions[-1]):.3f} rad")

# Run the simulation
run_joint_control_simulation()
```

## Exercise 2: Forward and Inverse Kinematics for a Simple Arm

Now let's implement kinematics for a simple 2-DOF planar arm:

```python
import numpy as np
import matplotlib.pyplot as plt

class PlanarArm:
    def __init__(self, link_lengths=[1.0, 1.0]):
        self.l1, self.l2 = link_lengths

    def forward_kinematics(self, theta1, theta2):
        """Calculate end-effector position from joint angles"""
        # Calculate end-effector position
        x = self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2)
        y = self.l1 * np.sin(theta1) + self.l2 * np.sin(theta1 + theta2)
        return x, y

    def jacobian(self, theta1, theta2):
        """Calculate the Jacobian matrix"""
        # Partial derivatives of end-effector position w.r.t. joint angles
        J = np.array([
            [-self.l1*np.sin(theta1) - self.l2*np.sin(theta1+theta2),
             -self.l2*np.sin(theta1+theta2)],
            [self.l1*np.cos(theta1) + self.l2*np.cos(theta1+theta2),
             self.l2*np.cos(theta1+theta2)]
        ])
        return J

    def inverse_kinematics_jacobian(self, target_x, target_y,
                                   initial_theta1=0, initial_theta2=0,
                                   max_iterations=100, tolerance=1e-4):
        """Solve inverse kinematics using Jacobian transpose method"""
        theta1, theta2 = initial_theta1, initial_theta2

        for i in range(max_iterations):
            # Calculate current position
            curr_x, curr_y = self.forward_kinematics(theta1, theta2)

            # Calculate error
            error_x = target_x - curr_x
            error_y = target_y - curr_y
            error = np.array([error_x, error_y])

            # Check convergence
            if np.linalg.norm(error) < tolerance:
                print(f"Converged after {i+1} iterations")
                break

            # Calculate Jacobian
            J = self.jacobian(theta1, theta2)

            # Update joint angles using Jacobian transpose
            # dq = J^T * dx (simple transpose method)
            joint_delta = 0.01 * J.T @ error
            theta1 += joint_delta[0]
            theta2 += joint_delta[1]
        else:
            print(f"Did not converge after {max_iterations} iterations")

        return theta1, theta2

# Demonstrate forward kinematics
arm = PlanarArm([0.7, 0.5])  # 70cm and 50cm links

# Test different joint angles
angles = [(0, 0), (np.pi/4, np.pi/4), (np.pi/2, -np.pi/4)]
print("Forward Kinematics Results:")
for theta1, theta2 in angles:
    x, y = arm.forward_kinematics(theta1, theta2)
    print(f"Angles: ({theta1:.2f}, {theta2:.2f}) -> Position: ({x:.2f}, {y:.2f})")

# Test inverse kinematics
print("\nInverse Kinematics Test:")
target_x, target_y = 0.8, 0.6
theta1, theta2 = arm.inverse_kinematics_jacobian(target_x, target_y)

print(f"Target: ({target_x}, {target_y})")
print(f"Solution: ({theta1:.3f}, {theta2:.3f})")

# Verify solution
x_verify, y_verify = arm.forward_kinematics(theta1, theta2)
print(f"Verification: ({x_verify:.3f}, {y_verify:.3f})")
print(f"Error: {np.sqrt((target_x-x_verify)**2 + (target_y-y_verify)**2):.4f}")
```

## Exercise 3: Balance Control with Center of Mass Tracking

Let's implement a simple balance controller using CoM tracking:

```python
import numpy as np
import matplotlib.pyplot as plt

class SimpleBalanceController:
    def __init__(self, com_height=0.8, control_gain=10.0):
        self.com_height = com_height  # Height of CoM (m)
        self.g = 9.81  # Gravity (m/s^2)
        self.omega = np.sqrt(self.g / self.com_height)  # Natural frequency
        self.K = control_gain  # Control gain

    def compute_zmp_from_com(self, com_pos, com_vel):
        """Compute ZMP from CoM position and velocity"""
        # ZMP = CoM - [0; 0; h] * CoM_ddot / g
        # For LIPM: CoM_ddot = omega^2 * (CoM - ZMP)
        # Solving for ZMP: ZMP = CoM - CoM_ddot / omega^2
        # But we need to estimate CoM_ddot from position and velocity
        # For now, we'll use a simplified approach

        zmp_x = com_pos[0] - com_vel[0] / self.omega
        zmp_y = com_pos[1] - com_vel[1] / self.omega

        return np.array([zmp_x, zmp_y])

    def compute_com_acceleration(self, desired_zmp, current_com_pos, current_com_vel):
        """Compute CoM acceleration to achieve desired ZMP"""
        # LIPM dynamics: x_ddot = omega^2 * (x - zmp)
        com_acc_x = self.omega**2 * (current_com_pos[0] - desired_zmp[0])
        com_acc_y = self.omega**2 * (current_com_pos[1] - desired_zmp[1])

        return np.array([com_acc_x, com_acc_y])

# Simulate balance control
def simulate_balance_control():
    controller = SimpleBalanceController(com_height=0.85)

    dt = 0.01  # 100Hz control
    simulation_time = 10.0
    steps = int(simulation_time / dt)

    # Initialize state
    com_pos = np.array([0.0, 0.0])  # Initial CoM position
    com_vel = np.array([0.0, 0.0])  # Initial CoM velocity
    com_acc = np.array([0.0, 0.0])  # Initial CoM acceleration

    # Desired ZMP trajectory (start at origin, move to offset at t=3s)
    desired_zmp = []
    for t in np.arange(0, simulation_time, dt):
        if t < 3.0:
            zmp = np.array([0.0, 0.0])
        elif t < 6.0:
            zmp = np.array([0.05, 0.0])  # Small offset in x
        else:
            zmp = np.array([0.0, 0.05])  # Small offset in y
        desired_zmp.append(zmp)

    # Store simulation results
    times = []
    com_positions = []
    zmp_positions = []
    desired_zmp_log = []

    for i, t in enumerate(np.arange(0, simulation_time, dt)):
        # Get desired ZMP
        des_zmp = desired_zmp[i]

        # Compute CoM acceleration to achieve desired ZMP
        com_acc = controller.compute_com_acceleration(des_zmp, com_pos, com_vel)

        # Integrate to get velocity and position
        com_vel += com_acc * dt
        com_pos += com_vel * dt

        # Compute actual ZMP from current CoM state
        actual_zmp = controller.compute_zmp_from_com(com_pos, com_vel)

        # Log data
        times.append(t)
        com_positions.append(com_pos.copy())
        zmp_positions.append(actual_zmp)
        desired_zmp_log.append(des_zmp)

    # Convert to numpy arrays for plotting
    com_positions = np.array(com_positions)
    zmp_positions = np.array(zmp_positions)
    desired_zmp_log = np.array(desired_zmp_log)

    # Plot results
    plt.figure(figsize=(15, 5))

    plt.subplot(1, 3, 1)
    plt.plot(times, com_positions[:, 0], 'b-', label='CoM X')
    plt.plot(times, desired_zmp_log[:, 0], 'r--', label='Desired ZMP X')
    plt.plot(times, zmp_positions[:, 0], 'g:', label='Actual ZMP X')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('X-Axis Balance Control')
    plt.legend()
    plt.grid(True)

    plt.subplot(1, 3, 2)
    plt.plot(times, com_positions[:, 1], 'b-', label='CoM Y')
    plt.plot(times, desired_zmp_log[:, 1], 'r--', label='Desired ZMP Y')
    plt.plot(times, zmp_positions[:, 1], 'g:', label='Actual ZMP Y')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Y-Axis Balance Control')
    plt.legend()
    plt.grid(True)

    plt.subplot(1, 3, 3)
    plt.plot(com_positions[:, 0], com_positions[:, 1], 'b-', label='CoM Trajectory')
    plt.plot(desired_zmp_log[:, 0], desired_zmp_log[:, 1], 'ro', label='Desired ZMP')
    plt.plot(zmp_positions[:, 0], zmp_positions[:, 1], 'g+', label='Actual ZMP')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Balance Control in XY Plane')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

    plt.tight_layout()
    plt.show()

simulate_balance_control()
```

## Exercise 4: ROS 2 Integration Example

Now let's create a practical ROS 2 example that demonstrates integration with the control system:

```python
# Save this as a separate Python file: balance_controller_ros2.py
"""
ROS 2 Node for Humanoid Balance Control
This example demonstrates how to integrate balance control with ROS 2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64MultiArray
import numpy as np

class BalanceControllerROS2(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Declare parameters
        self.declare_parameter('com_height', 0.85)
        self.declare_parameter('control_frequency', 100)

        self.com_height = self.get_parameter('com_height').value
        self.g = 9.81
        self.omega = np.sqrt(self.g / self.com_height)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.desired_zmp_sub = self.create_subscription(
            Point,
            '/desired_zmp',
            self.desired_zmp_callback,
            10
        )

        # Publishers
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        self.com_state_pub = self.create_publisher(
            Point,
            '/com_state',
            10
        )

        # Timer for control loop
        control_period = 1.0 / self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(control_period, self.control_loop)

        # Internal state
        self.current_joint_positions = []
        self.current_joint_velocities = []
        self.desired_zmp = np.array([0.0, 0.0])

        self.get_logger().info('Balance Controller initialized')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.current_joint_positions = list(msg.position)
        if msg.velocity:
            self.current_joint_velocities = list(msg.velocity)

        # For this example, we'll assume we can estimate CoM from joint positions
        # In a real system, this would use forward kinematics and mass distribution
        self.estimated_com = self.estimate_com_from_joints(msg.position)

    def desired_zmp_callback(self, msg):
        """Callback for desired ZMP"""
        self.desired_zmp = np.array([msg.x, msg.y])

    def estimate_com_from_joints(self, joint_positions):
        """Estimate CoM position from joint positions (simplified model)"""
        # This is a simplified estimation - in reality, this would use
        # forward kinematics and a detailed mass distribution model
        # For this example, we'll return a placeholder
        return np.array([0.0, 0.0, self.com_height])

    def compute_balance_control(self):
        """Compute balance control commands"""
        # This would implement the actual balance control algorithm
        # For this example, we'll return placeholder joint commands
        # that would help achieve the desired ZMP

        # Simplified approach: adjust joint positions based on ZMP error
        com_pos = self.estimated_com[:2]  # X, Y only
        zmp_error = self.desired_zmp - com_pos

        # Generate joint commands to correct ZMP error
        # This would involve inverse kinematics and whole-body control
        joint_commands = [0.0] * len(self.current_joint_positions)

        # Add simple proportional correction based on ZMP error
        if len(joint_commands) >= 6:  # At least 6 joints
            # Adjust hip joints for balance (simplified)
            joint_commands[0] = 0.1 * zmp_error[0]  # Left hip roll
            joint_commands[1] = 0.1 * zmp_error[1]  # Left hip pitch
            joint_commands[3] = 0.1 * zmp_error[0]  # Right hip roll
            joint_commands[4] = 0.1 * zmp_error[1]  # Right hip pitch

        return joint_commands

    def control_loop(self):
        """Main control loop"""
        if not self.current_joint_positions:
            return  # Wait for first joint state message

        # Compute balance control commands
        joint_commands = self.compute_balance_control()

        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_commands
        self.joint_command_pub.publish(cmd_msg)

        # Publish CoM state
        com_msg = Point()
        com_msg.x = float(self.estimated_com[0])
        com_msg.y = float(self.estimated_com[1])
        com_msg.z = float(self.estimated_com[2])
        self.com_state_pub.publish(com_msg)

def main(args=None):
    rclpy.init(args=args)

    balance_controller = BalanceControllerROS2()

    try:
        rclpy.spin(balance_controller)
    except KeyboardInterrupt:
        pass
    finally:
        balance_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 5: Whole-Body Control Simulation

Let's create a more comprehensive example that combines multiple control concepts:

```python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class WholeBodyController:
    def __init__(self):
        # Robot parameters (simplified humanoid model)
        self.left_foot = np.array([0.1, -0.1])  # Position relative to body
        self.right_foot = np.array([0.1, 0.1])
        self.com_height = 0.85  # Fixed CoM height for simplicity

        # Control parameters
        self.zmp_gain = 10.0
        self.com_gain = 5.0

    def compute_support_polygon(self):
        """Compute the support polygon for balance"""
        # For this example, assume feet positions are fixed
        # In reality, this would come from forward kinematics
        support_points = np.array([
            self.left_foot,
            [self.left_foot[0], self.left_foot[1] - 0.1],  # Foot width
            [self.right_foot[0], self.right_foot[1] + 0.1],
            self.right_foot
        ])
        return support_points

    def is_stable(self, zmp):
        """Check if ZMP is within support polygon"""
        support_poly = self.compute_support_polygon()

        # Simple check: ZMP between feet in y-direction,
        # and within foot length in x-direction
        y_min = min(self.left_foot[1], self.right_foot[1]) - 0.05
        y_max = max(self.left_foot[1], self.right_foot[1]) + 0.05
        x_min = min(self.left_foot[0], self.right_foot[0]) - 0.1
        x_max = max(self.left_foot[0], self.right_foot[0]) + 0.1

        return (x_min <= zmp[0] <= x_max and y_min <= zmp[1] <= y_max)

    def compute_balance_correction(self, desired_com, current_com, current_zmp):
        """Compute balance correction based on CoM and ZMP errors"""
        # CoM tracking error
        com_error = desired_com[:2] - current_com[:2]

        # ZMP tracking error
        zmp_error = desired_com[:2] - current_zmp  # Drive ZMP to desired CoM

        # Combined balance correction
        balance_correction = self.com_gain * com_error + self.zmp_gain * zmp_error

        return balance_correction

# Create an animation to visualize the control
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.3, 0.3)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title('Whole-Body Balance Control Simulation')

# Initialize robot state
controller = WholeBodyController()
time_step = 0.02
current_com = np.array([0.0, 0.0, controller.com_height])
current_com_vel = np.array([0.0, 0.0, 0.0])
desired_com_trajectory = []

# Generate a desired CoM trajectory (circle motion)
t_values = np.linspace(0, 4*np.pi, 400)
for t in t_values:
    x = 0.1 * np.cos(t * 0.5)
    y = 0.05 * np.sin(t * 0.5)
    desired_com_trajectory.append(np.array([x, y, controller.com_height]))

# Plot elements
com_point, = ax.plot([], [], 'bo', markersize=10, label='CoM')
zmp_point, = ax.plot([], [], 'ro', markersize=8, label='ZMP')
foot_left, = ax.plot([], [], 'ks', markersize=12, label='Left Foot')
foot_right, = ax.plot([], [], 'gs', markersize=12, label='Right Foot')
support_polygon = plt.Polygon(controller.compute_support_polygon(),
                             alpha=0.2, color='gray', label='Support Polygon')
ax.add_patch(support_polygon)

ax.legend()

def animate(frame):
    global current_com, current_com_vel

    if frame >= len(desired_com_trajectory):
        return com_point, zmp_point, foot_left, foot_right, support_polygon

    # Get desired CoM
    desired_com = desired_com_trajectory[frame]

    # Compute ZMP from current CoM (simplified)
    # ZMP = CoM - CoM_vel / omega (for LIPM)
    omega = np.sqrt(9.81 / controller.com_height)
    current_zmp = current_com[:2] - current_com_vel[:2] / omega if omega > 0 else current_com[:2]

    # Compute balance correction
    balance_correction = controller.compute_balance_correction(
        desired_com, current_com, current_zmp
    )

    # Simple dynamics update (in reality, this would involve full robot dynamics)
    # Apply balance correction to CoM velocity
    current_com_vel[:2] += balance_correction * time_step
    current_com[:2] += current_com_vel[:2] * time_step

    # Update plot
    com_point.set_data([current_com[0]], [current_com[1]])
    zmp_point.set_data([current_zmp[0]], [current_zmp[1]])
    foot_left.set_data([controller.left_foot[0]], [controller.left_foot[1]])
    foot_right.set_data([controller.right_foot[0]], [controller.right_foot[1]])

    # Update support polygon
    support_points = controller.compute_support_polygon()
    support_polygon.set_xy(support_points)

    # Color code based on stability
    is_stable = controller.is_stable(current_zmp)
    com_point.set_color('blue' if is_stable else 'red')

    ax.set_title(f'Whole-Body Balance Control - Stable: {is_stable}')

    return com_point, zmp_point, foot_left, foot_right, support_polygon

# Create animation
anim = FuncAnimation(fig, animate, frames=len(desired_com_trajectory),
                     interval=time_step*1000, blit=True, repeat=True)

plt.show()

print("Whole-body balance control simulation completed.")
print("The animation shows:")
print("- Blue dot: Center of Mass (CoM)")
print("- Red dot: Zero-Moment Point (ZMP)")
print("- Black/Green squares: Feet positions")
print("- Gray polygon: Support polygon")
print("- Color changes to red when robot is unstable (ZMP outside support polygon)")
```

## Exercise 6: Practical Implementation Tips

Here are some practical considerations for implementing humanoid control systems:

### Safety Considerations
```python
class SafetyMonitor:
    def __init__(self):
        self.emergency_stop = False
        self.max_joint_velocity = 5.0  # rad/s
        self.max_joint_torque = 100.0  # Nm
        self.torque_threshold = 80.0   # Threshold for safety checks

    def check_safety(self, joint_positions, joint_velocities, joint_torques):
        """Check if robot state is safe"""
        # Check joint limits
        for vel in joint_velocities:
            if abs(vel) > self.max_joint_velocity:
                print(f"WARNING: High joint velocity detected: {vel}")
                return False

        # Check torque limits
        for torque in joint_torques:
            if abs(torque) > self.max_joint_torque:
                print(f"WARNING: High joint torque detected: {torque}")
                return False

        # Check for sudden torque changes (indicating collision)
        for torque in joint_torques:
            if abs(torque) > self.torque_threshold:
                print(f"WARNING: High torque detected: {torque}")
                return False

        return True

    def emergency_stop_procedure(self):
        """Execute emergency stop"""
        print("EMERGENCY STOP ACTIVATED")
        # Set all joint torques to zero
        # Log the incident
        # Wait for manual reset
        self.emergency_stop = True
```

### Performance Optimization
```python
import time
from functools import wraps

def timing_decorator(func):
    """Decorator to measure function execution time"""
    @wraps(func)
    def wrapper(*args, **kwargs):
        start = time.perf_counter()
        result = func(*args, **kwargs)
        end = time.perf_counter()
        print(f"{func.__name__} executed in {(end-start)*1000:.2f} ms")
        return result
    return wrapper

class OptimizedController:
    def __init__(self):
        self.previous_values = {}  # Cache for expensive calculations

    @timing_decorator
    def compute_inverse_kinematics(self, target_pos, current_joints):
        """Optimized inverse kinematics with caching"""
        # Simple caching mechanism
        target_key = tuple(np.round(target_pos, decimals=3))

        if target_key in self.previous_values:
            # Return cached result if target is similar
            cached_joints, cached_time = self.previous_values[target_key]
            if time.time() - cached_time < 0.1:  # Cache for 100ms
                return cached_joints

        # Perform expensive IK calculation
        result = self._expensive_ik_calculation(target_pos, current_joints)

        # Cache the result
        self.previous_values[target_key] = (result, time.time())

        return result

    def _expensive_ik_calculation(self, target_pos, current_joints):
        """Placeholder for actual IK algorithm"""
        # In a real implementation, this would contain the actual
        # inverse kinematics calculation
        return current_joints  # Placeholder
```

## Conclusion

These practical examples demonstrate the implementation of key humanoid control concepts:

1. **Basic Control**: PID controllers form the foundation of joint control
2. **Kinematics**: Forward and inverse kinematics enable spatial control
3. **Balance**: ZMP-based control maintains stability
4. **Integration**: ROS 2 enables system integration
5. **Whole-Body**: Coordinated control of multiple subsystems
6. **Safety**: Essential for real-world deployment

Students should experiment with these examples, modify parameters, and observe the effects on system behavior. Understanding both the theoretical concepts and their practical implementation is crucial for developing effective humanoid control systems.

The next module will explore perception and sensing systems that provide the sensory input necessary for intelligent physical behavior.