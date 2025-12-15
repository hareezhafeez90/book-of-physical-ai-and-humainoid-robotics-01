# Balance and Stability: Maintaining Upright Posture

## Introduction: The Challenge of Bipedal Balance

Maintaining balance is one of the most fundamental challenges in humanoid robotics. Unlike wheeled or tracked robots that have continuous ground contact, bipedal robots must maintain stability with only two points of contact (during single support) or a small support polygon (during double support). This section explores the principles and control strategies that enable humanoid robots to maintain upright posture and recover from disturbances.

### Why Balance is Critical for Humanoid Robots

Balance is fundamental to humanoid robot operation because:
- **Safety**: Unstable robots can fall and damage themselves or surroundings
- **Functionality**: Many tasks require stable posture to execute properly
- **Efficiency**: Maintaining balance reduces energy consumption
- **Human-like Operation**: Stable balance enables human-like movement patterns

## The Physics of Balance

### Center of Mass (CoM) and Center of Pressure (CoP)

The relationship between CoM and CoP determines stability:
- **CoM**: The weighted average position of all mass in the system
- **CoP**: The point where the ground reaction force acts

For static balance, the CoM projection must remain within the support polygon defined by ground contact points.

### Support Polygon

The support polygon is the convex hull of all ground contact points:
- **Double Support**: Polygon defined by both feet
- **Single Support**: Polygon defined by stance foot
- **Multi-Contact**: Polygon defined by all contact points (e.g., hands and feet)

### Zero-Moment Point (ZMP) Theory

The ZMP is a crucial concept in dynamic balance:
- **Definition**: Point on the ground where the moment of the ground reaction force equals the moment of the gravitational and inertial forces
- **Stability Condition**: ZMP must remain within the support polygon for stable motion
- **Mathematical Expression**:
  ```
  px = x_com - (z_com - z_ref) / g * x_com_ddot
  py = y_com - (z_com - z_ref) / g * y_com_ddot
  ```

## Balance Control Strategies

### Feedback Control Approaches

#### PID-Based Balance Control

Proportional-Integral-Derivative controllers form the foundation of many balance systems:

```python
class BalanceController:
    def __init__(self, kp=100.0, ki=10.0, kd=20.0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.integral_error = 0
        self.previous_error = 0

    def compute_balance_torque(self, current_angle, desired_angle, dt):
        """Compute balance torque using PID control"""
        error = desired_angle - current_angle

        # Integral of error
        self.integral_error += error * dt

        # Derivative of error
        derivative_error = (error - self.previous_error) / dt

        # PID control law
        torque = (self.kp * error +
                 self.ki * self.integral_error +
                 self.kd * derivative_error)

        self.previous_error = error
        return torque
```

#### Linear Quadratic Regulator (LQR)

LQR provides optimal control for linearized balance models:

```python
import numpy as np
from scipy.linalg import solve_continuous_are

def lqr_balance_control(A, B, Q, R):
    """Design LQR controller for balance system"""
    # Solve algebraic Riccati equation
    P = solve_continuous_are(A, B, Q, R)

    # Optimal feedback gain
    K = np.linalg.inv(R) @ B.T @ P

    return K

# Example: Simple inverted pendulum model
# State: [angle, angular_velocity]
A = np.array([[0, 1], [g/l, 0]])  # Linearized inverted pendulum
B = np.array([[0], [1/(m*l**2)]])  # Torque input matrix
Q = np.array([[10, 0], [0, 1]])    # State cost matrix
R = np.array([[0.1]])              # Control cost matrix

K = lqr_balance_control(A, B, Q, R)
```

### Advanced Balance Control Methods

#### Capture Point Theory

The capture point is the location where a robot must step to come to a complete stop:

```python
def compute_capture_point(com_pos, com_vel, com_height):
    """Compute capture point for balance recovery"""
    g = 9.81
    omega = np.sqrt(g / com_height)
    capture_point = com_pos + com_vel / omega
    return capture_point

def balance_recovery_step(capture_point, current_foot_pos, max_step_size=0.3):
    """Determine step location for balance recovery"""
    step_vector = capture_point - current_foot_pos
    step_distance = np.linalg.norm(step_vector)

    if step_distance > max_step_size:
        # Scale to maximum step size
        step_vector = step_vector / step_distance * max_step_size

    return current_foot_pos + step_vector
```

#### Linear Inverted Pendulum Mode (LIPM)

The LIPM simplifies balance control by assuming constant CoM height:

```python
class LIPMController:
    def __init__(self, com_height=0.8, dt=0.01):
        self.com_height = com_height
        self.omega = np.sqrt(9.81 / com_height)
        self.dt = dt
        self.zmp_ref = np.array([0.0, 0.0])

    def update_zmp_reference(self, desired_com_pos, current_com_pos):
        """Update ZMP reference based on desired CoM position"""
        # ZMP planning using LIPM dynamics
        com_error = desired_com_pos - current_com_pos
        self.zmp_ref = desired_com_pos - com_error / (self.omega**2)
        return self.zmp_ref

    def compute_com_trajectory(self, zmp_trajectory):
        """Compute CoM trajectory from ZMP trajectory"""
        # Solve LIPM differential equation
        # x_ddot = omega^2 * (x - zmp)
        # This would typically use numerical integration
        pass
```

## Sensor-Based Balance Control

### Inertial Measurement Units (IMU)

IMUs provide crucial feedback for balance control:

```python
class IMUBasedBalance:
    def __init__(self):
        self.orientation = np.array([0, 0, 0, 1])  # Quaternion
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        self.com_estimator = ComEstimator()

    def process_imu_data(self, gyro_data, accel_data, dt):
        """Process IMU data for balance control"""
        # Integrate gyro data for orientation
        self.angular_velocity = gyro_data
        self.update_orientation(dt)

        # Use accelerometer for gravity reference
        self.linear_acceleration = accel_data

        # Estimate CoM state
        self.com_state = self.com_estimator.estimate(
            self.orientation,
            self.linear_acceleration,
            dt
        )

    def update_orientation(self, dt):
        """Update orientation using gyro integration"""
        # Convert angular velocity to quaternion derivative
        omega_quat = np.array([0, *self.angular_velocity])
        quat_dot = 0.5 * self.quaternion_multiply(
            self.orientation,
            omega_quat
        )

        # Integrate
        self.orientation += quat_dot * dt
        # Normalize quaternion
        self.orientation = self.orientation / np.linalg.norm(self.orientation)
```

### Force/Torque Sensors

Force/torque sensors in feet provide ground contact information:

```python
class ForceBasedBalance:
    def __init__(self):
        self.left_foot_force = np.zeros(6)  # Fx, Fy, Fz, Mx, My, Mz
        self.right_foot_force = np.zeros(6)
        self.support_foot = "left"  # or "right" or "both"

    def compute_zmp_from_forces(self):
        """Compute ZMP from foot force sensors"""
        total_force = self.left_foot_force + self.right_foot_force
        total_moment = self.compute_total_moment()

        if total_force[2] > 1.0:  # Check for sufficient normal force
            zmp_x = -total_moment[1] / total_force[2]  # My / Fz
            zmp_y = total_moment[0] / total_force[2]   # Mx / Fz
            return np.array([zmp_x, zmp_y])
        else:
            return np.array([0.0, 0.0])  # No contact

    def detect_support_switch(self):
        """Detect when robot switches support foot"""
        left_force_z = self.left_foot_force[2]
        right_force_z = self.right_foot_force[2]

        # Determine support foot based on force distribution
        if left_force_z > 200 and right_force_z < 50:
            self.support_foot = "left"
        elif right_force_z > 200 and left_force_z < 50:
            self.support_foot = "right"
        else:
            self.support_foot = "both"
```

## Walking Balance Control

### Phase-Based Balance Control

Walking involves distinct phases that require different balance strategies:

```python
class WalkingBalanceController:
    def __init__(self):
        self.phase = "double_support"  # double_support, single_support_left, single_support_right
        self.balance_controller = BalanceController()
        self.zmp_planner = ZMPPlanner()

    def update_balance_control(self, current_phase, time_in_phase):
        """Update balance control based on walking phase"""
        if current_phase != self.phase:
            self.phase = current_phase
            self.adapt_balance_strategy()

        # Compute balance torques based on current phase
        if self.phase == "double_support":
            # More conservative balance control
            self.balance_controller.set_gains(80, 8, 15)
        elif self.phase == "single_support_left":
            # Focus on left foot stability
            self.balance_controller.set_gains(100, 10, 20)
        elif self.phase == "single_support_right":
            # Focus on right foot stability
            self.balance_controller.set_gains(100, 10, 20)

    def adapt_balance_strategy(self):
        """Adapt balance strategy when phase changes"""
        if self.phase == "double_support":
            # Plan ZMP trajectory for double support
            self.zmp_planner.plan_double_support_trajectory()
        else:
            # Plan ZMP trajectory for single support
            self.zmp_planner.plan_single_support_trajectory()
```

### Disturbance Recovery

Robots must be able to recover from external disturbances:

```python
class DisturbanceRecovery:
    def __init__(self):
        self.disturbance_threshold = 0.5  # Threshold for disturbance detection
        self.recovery_active = False

    def detect_disturbance(self, com_acceleration, imu_data):
        """Detect external disturbances"""
        # Check for high acceleration or unexpected IMU readings
        linear_acc_magnitude = np.linalg.norm(com_acceleration)
        angular_vel_magnitude = np.linalg.norm(imu_data.angular_velocity)

        if (linear_acc_magnitude > self.disturbance_threshold or
            angular_vel_magnitude > self.disturbance_threshold):
            return True
        return False

    def initiate_recovery(self, current_state):
        """Initiate balance recovery sequence"""
        if not self.recovery_active:
            self.recovery_active = True

            # Compute recovery strategy
            capture_point = self.compute_capture_point(
                current_state.com_pos,
                current_state.com_vel,
                current_state.com_height
            )

            # Determine recovery step location
            recovery_step = self.plan_recovery_step(capture_point)

            # Execute recovery
            self.execute_recovery_step(recovery_step)

    def execute_recovery_step(self, step_location):
        """Execute a recovery step"""
        # Move swing foot to recovery location
        # Adjust CoM trajectory
        # Monitor for successful recovery
        pass
```

## Multi-Contact Balance

### Using Arms for Balance

Humanoid robots can use their arms to assist with balance:

```python
def arm_balance_assist(com_state, desired_com_state, current_arm_pos):
    """Use arm movement for balance assistance"""
    # Calculate required arm position for balance
    com_error = desired_com_state[:2] - com_state[:2]  # X, Y position error

    # Map CoM error to arm movement
    arm_correction = -0.3 * com_error  # Proportional correction
    desired_arm_pos = current_arm_pos + arm_correction

    return desired_arm_pos
```

### Whole-Body Balance Control

Coordinating all degrees of freedom for optimal balance:

```python
class WholeBodyBalanceController:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.kinematic_solver = KinematicSolver()
        self.dynamics_model = DynamicsModel()

    def compute_balance_posture(self, desired_com, desired_zmp, support_polygons):
        """Compute whole-body posture for balance"""
        # Formulate optimization problem
        # Minimize: ||q - q_nominal||^2 + ||com - desired_com||^2
        # Subject to: zmp within support polygon, joint limits, balance constraints

        # This would typically use a QP solver
        # For this example, we'll outline the approach:

        # 1. Define task hierarchy
        #    - Primary: ZMP tracking
        #    - Secondary: CoM tracking
        #    - Tertiary: Posture optimization

        # 2. Use null-space projection for task prioritization
        # 3. Apply joint limits and balance constraints
        # 4. Solve for joint angles

        return self.solve_balance_optimization(
            desired_com, desired_zmp, support_polygons
        )
```

## Practical Implementation Considerations

### Real-Time Performance

Balance controllers must operate in real-time:

```python
def real_time_balance_loop(sensor_data, dt):
    """Real-time balance control loop"""
    # 1. Sensor data processing (1-2ms)
    processed_data = process_sensors(sensor_data)

    # 2. State estimation (1-2ms)
    robot_state = estimate_state(processed_data)

    # 3. Balance control computation (2-5ms)
    control_commands = compute_balance_control(robot_state)

    # 4. Command execution (1ms)
    send_commands(control_commands)

    # Total: < 10ms for real-time performance
```

### Robustness to Modeling Errors

Balance controllers must handle model inaccuracies:

```python
def robust_balance_control(measured_state, model_state, control_input):
    """Robust balance control with model error compensation"""
    # Estimate model error
    state_error = measured_state - model_state

    # Adapt control gains based on error magnitude
    if np.linalg.norm(state_error) > 0.1:  # High error threshold
        # Increase robustness, decrease performance
        adaptive_gains = decrease_gains(control_input.gains)
    else:
        # Use nominal gains
        adaptive_gains = control_input.gains

    return adaptive_gains
```

## Conclusion

Balance and stability control is fundamental to humanoid robot operation. The complex interplay between center of mass dynamics, support polygon constraints, and environmental interactions requires sophisticated control strategies. From basic PID controllers to advanced optimization-based approaches, balance systems must operate in real-time while handling modeling uncertainties and external disturbances.

The next section will explore motor control systems that translate high-level balance and movement commands into precise actuator commands, completing the control hierarchy from high-level goals to physical execution.