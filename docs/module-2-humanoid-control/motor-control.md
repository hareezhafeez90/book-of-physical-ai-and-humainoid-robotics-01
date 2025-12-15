# Motor Control Systems: From High-Level Commands to Actuator Commands

## Introduction: The Robotic Muscular System

Motor control systems form the "muscular system" of humanoid robots, translating high-level movement commands into precise actuator torques that generate physical motion. This section explores the control architectures, algorithms, and implementation strategies that enable precise and robust actuator control while maintaining the stability and safety required for humanoid operation.

### The Control Hierarchy in Humanoid Robots

Humanoid control operates on multiple levels:
- **High-Level**: Task planning, motion planning, and trajectory generation
- **Mid-Level**: Balance control, whole-body control, and coordination
- **Low-Level**: Joint-level servo control and actuator management

Motor control systems operate at the lowest level, receiving desired joint positions, velocities, and torques from higher-level controllers and generating the appropriate electrical signals to drive actuators.

## Actuator Technologies for Humanoid Robots

### Types of Actuators

#### Servo Motors
Servo motors are the most common actuators in humanoid robots:
- **Precision**: High-resolution position control
- **Speed**: Fast response times
- **Torque**: Adequate torque for humanoid applications
- **Integration**: Built-in position/velocity/torque sensing

#### Series Elastic Actuators (SEA)
SEAs incorporate a spring in series with the motor:
- **Compliance**: Built-in compliance for safe interaction
- **Force Control**: Direct force control capability
- **Backdrivability**: Easy manual manipulation when powered off
- **Energy Storage**: Spring can store and release energy

#### Variable Stiffness Actuators (VSA)
VSAs allow adjustment of joint stiffness:
- **Adaptability**: Stiffness can be adjusted for different tasks
- **Energy Efficiency**: Lower stiffness for compliant tasks
- **Safety**: Reduced impact forces during collisions

### Actuator Characteristics

Each actuator type has specific characteristics that affect control:

```python
class ActuatorModel:
    def __init__(self, actuator_type="servo"):
        self.type = actuator_type
        self.max_torque = 0  # Nm
        self.max_velocity = 0  # rad/s
        self.gear_ratio = 0
        self.torque_constant = 0  # Nm/A
        self.backlash = 0  # rad
        self.friction_params = [0, 0, 0]  # Static, Coulomb, Viscous

    def compute_torque_limit(self, velocity):
        """Compute maximum available torque based on speed"""
        # Torque-velocity curve
        max_torque = self.max_torque
        if abs(velocity) > self.max_velocity * 0.8:
            # Torque decreases at high speeds
            max_torque *= (1 - abs(velocity) / self.max_velocity)
        return max_torque

    def model_friction(self, velocity):
        """Model friction effects"""
        if abs(velocity) < 0.01:  # Static friction region
            friction_torque = self.friction_params[0] * np.sign(velocity)
        else:  # Dynamic friction
            friction_torque = (self.friction_params[1] * np.sign(velocity) +
                             self.friction_params[2] * velocity)
        return friction_torque
```

## Joint-Level Control

### PID Control for Joint Servos

Proportional-Integral-Derivative (PID) controllers form the foundation of joint-level control:

```python
class JointController:
    def __init__(self, kp=100, ki=10, kd=20):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.integral_error = 0
        self.previous_error = 0
        self.max_integral = 10  # Anti-windup limit
        self.max_torque = 100   # Torque limit (Nm)

    def compute_command(self, desired_pos, current_pos, desired_vel=0, current_vel=0, dt=0.001):
        """Compute joint control command"""
        # Position error
        pos_error = desired_pos - current_pos

        # Integral of error (with anti-windup)
        self.integral_error += pos_error * dt
        self.integral_error = np.clip(self.integral_error,
                                    -self.max_integral, self.max_integral)

        # Derivative of error
        derivative_error = (pos_error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = pos_error

        # PID control law
        torque = (self.kp * pos_error +
                 self.ki * self.integral_error +
                 self.kd * (desired_vel - current_vel))

        # Apply torque limits
        torque = np.clip(torque, -self.max_torque, self.max_torque)

        return torque
```

### Advanced Joint Control Techniques

#### Feedforward Control

Feedforward control improves tracking performance by anticipating required torques:

```python
class FeedforwardController:
    def __init__(self):
        self.inverse_dynamics = InverseDynamicsModel()

    def compute_feedforward_torque(self, desired_pos, desired_vel, desired_acc, robot_state):
        """Compute feedforward torque based on desired trajectory"""
        # Gravity compensation
        gravity_torque = self.inverse_dynamics.gravity_compensation(robot_state.q)

        # Coriolis and centrifugal compensation
        coriolis_torque = self.inverse_dynamics.coriolis_compensation(
            robot_state.q, robot_state.q_dot
        )

        # Inertia shaping
        inertia_torque = self.inverse_dynamics.inertia_matrix(robot_state.q) @ desired_acc

        # Total feedforward torque
        ff_torque = inertia_torque + coriolis_torque + gravity_torque

        return ff_torque

    def combined_control(self, desired_pos, desired_vel, desired_acc, current_pos, current_vel):
        """Combine feedback and feedforward control"""
        # Feedback control (from PID controller)
        feedback_torque = self.feedback_controller.compute_command(
            desired_pos, current_pos, desired_vel, current_vel
        )

        # Feedforward control
        feedforward_torque = self.compute_feedforward_torque(
            desired_pos, desired_vel, desired_acc,
            type('RobotState', (), {'q': current_pos, 'q_dot': current_vel})()
        )

        # Combined command
        total_torque = feedback_torque + feedforward_torque

        return total_torque
```

#### Impedance Control

Impedance control makes joints behave like springs and dampers:

```python
class ImpedanceController:
    def __init__(self, stiffness=100, damping=20):
        self.stiffness = stiffness  # N/m or Nm/rad
        self.damping = damping      # N*s/m or Nm*s/rad

    def compute_impedance_force(self, pos_error, vel_error):
        """Compute impedance control force"""
        force = self.stiffness * pos_error + self.damping * vel_error
        return force

    def set_impedance_parameters(self, stiffness, damping):
        """Adjust impedance parameters for different tasks"""
        self.stiffness = stiffness
        self.damping = damping
```

## Torque Control and Force Control

### Direct Torque Control

Direct torque control provides precise force regulation:

```python
class TorqueController:
    def __init__(self, kp=10, ki=1, kd=2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_error = 0
        self.previous_error = 0

    def compute_current_command(self, desired_torque, actual_torque, dt):
        """Convert torque error to current command"""
        torque_error = desired_torque - actual_torque

        self.integral_error += torque_error * dt
        derivative_error = (torque_error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = torque_error

        # PID for torque control
        current_command = (self.kp * torque_error +
                          self.ki * self.integral_error +
                          self.kd * derivative_error)

        return current_command
```

### Admittance Control

Admittance control relates force inputs to motion outputs:

```python
class AdmittanceController:
    def __init__(self, mass=1.0, damping=10, stiffness=100):
        self.mass = mass      # Apparent mass
        self.damping = damping  # Apparent damping
        self.stiffness = stiffness  # Apparent stiffness

    def update_motion(self, applied_force, current_pos, current_vel, dt):
        """Update motion based on applied force"""
        # Admittance: motion = admittance * force
        # M*x_ddot + B*x_dot + K*x = F
        acceleration = (applied_force - self.damping * current_vel -
                       self.stiffness * current_pos) / self.mass

        # Integrate to get new state
        new_vel = current_vel + acceleration * dt
        new_pos = current_pos + new_vel * dt

        return new_pos, new_vel
```

## Safety and Limit Management

### Joint Limit Handling

Safety systems prevent damage from joint limit violations:

```python
class JointLimitSafety:
    def __init__(self, joint_limits):
        self.position_limits = joint_limits['position']
        self.velocity_limits = joint_limits['velocity']
        self.torque_limits = joint_limits['torque']
        self.safety_margin = 0.1  # 10% safety margin

    def check_limits(self, position, velocity, torque):
        """Check if joint commands are within safe limits"""
        pos_ok = (self.position_limits[0] + self.safety_margin <= position <=
                 self.position_limits[1] - self.safety_margin)
        vel_ok = abs(velocity) <= self.velocity_limits[1]
        torque_ok = abs(torque) <= self.torque_limits[1]

        return pos_ok and vel_ok and torque_ok

    def enforce_limits(self, command):
        """Enforce joint limits on commands"""
        command.position = np.clip(
            command.position,
            self.position_limits[0] + self.safety_margin,
            self.position_limits[1] - self.safety_margin
        )
        command.velocity = np.clip(
            command.velocity,
            -self.velocity_limits[1],
            self.velocity_limits[1]
        )
        command.torque = np.clip(
            command.torque,
            -self.torque_limits[1],
            self.torque_limits[1]
        )
        return command
```

### Collision Detection and Response

Detecting and responding to collisions is crucial for safety:

```python
class CollisionDetector:
    def __init__(self, threshold=50):  # Nm or N
        self.threshold = threshold
        self.torque_history = []
        self.max_history = 100

    def detect_collision(self, current_torque, current_force=None):
        """Detect collision based on torque/force measurements"""
        # Add current measurement to history
        self.torque_history.append(abs(current_torque))
        if len(self.torque_history) > self.max_history:
            self.torque_history.pop(0)

        # Check for sudden torque increase
        if len(self.torque_history) > 10:
            avg_torque = np.mean(self.torque_history[-10:])
            current_torque_mag = abs(current_torque)

            if current_torque_mag > self.threshold:
                return True, "High force detected"
            elif current_torque_mag > 2 * avg_torque and current_torque_mag > 0.5 * self.threshold:
                return True, "Sudden force increase detected"

        return False, "No collision"

    def collision_response(self, collision_type):
        """Execute collision response"""
        if collision_type == "High force detected":
            # Reduce motor gains to be more compliant
            self.reduce_control_gains()
        elif collision_type == "Sudden force increase detected":
            # Check if intentional contact or actual collision
            self.assess_contact_intent()
```

## Real-Time Control Implementation

### Real-Time Operating Systems

Real-time control requires deterministic timing:

```python
import threading
import time

class RealTimeController:
    def __init__(self, control_frequency=1000):  # 1kHz control
        self.control_frequency = control_frequency
        self.control_period = 1.0 / control_frequency
        self.controllers = {}
        self.running = False
        self.thread = None

    def add_joint_controller(self, joint_name, controller):
        """Add a joint controller to the real-time system"""
        self.controllers[joint_name] = controller

    def control_loop(self):
        """Real-time control loop"""
        while self.running:
            start_time = time.time()

            # Read sensor data
            sensor_data = self.read_sensors()

            # Update all controllers
            for joint_name, controller in self.controllers.items():
                command = controller.compute_command(
                    sensor_data[joint_name].desired_pos,
                    sensor_data[joint_name].current_pos,
                    sensor_data[joint_name].desired_vel,
                    sensor_data[joint_name].current_vel,
                    self.control_period
                )
                self.send_command(joint_name, command)

            # Maintain timing
            elapsed = time.time() - start_time
            sleep_time = self.control_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                print(f"Control deadline missed by {abs(sleep_time)*1000:.1f}ms")

    def start(self):
        """Start the real-time control loop"""
        self.running = True
        self.thread = threading.Thread(target=self.control_loop)
        self.thread.start()

    def stop(self):
        """Stop the real-time control loop"""
        self.running = False
        if self.thread:
            self.thread.join()
```

### Communication Protocols

Efficient communication between controllers and actuators:

```python
class CANBusInterface:
    """CAN bus interface for motor communication"""
    def __init__(self, can_channel="can0"):
        self.can_channel = can_channel
        self.bus = None

    def send_torque_command(self, motor_id, torque):
        """Send torque command to specific motor"""
        # Pack torque into CAN message
        torque_int = int(torque * 1000)  # Convert to milli-Nm
        data = torque_int.to_bytes(4, byteorder='little', signed=True)

        # Create CAN message
        msg = can.Message(
            arbitration_id=0x140 + motor_id,  # Standard torque command ID
            data=data,
            is_extended_id=True
        )

        # Send message
        self.bus.send(msg)

    def read_feedback(self, motor_id):
        """Read position, velocity, torque feedback"""
        # Request feedback
        request_msg = can.Message(
            arbitration_id=0x180 + motor_id,
            data=[0x01],  # Request position
            is_extended_id=True
        )
        self.bus.send(request_msg)

        # Wait for response
        response = self.bus.recv(timeout=0.01)  # 10ms timeout
        if response:
            pos_raw = int.from_bytes(response.data[0:4], byteorder='little', signed=True)
            vel_raw = int.from_bytes(response.data[4:8], byteorder='little', signed=True)

            position = pos_raw / 10000.0  # Convert to radians
            velocity = vel_raw / 1000.0   # Convert to rad/s

            return position, velocity
        return None, None
```

## Integration with Higher-Level Control

### ROS 2 Integration

Connecting motor control with the ROS 2 communication framework:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Publishers and subscribers
        self.joint_command_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.001, self.control_loop)  # 1kHz

        # Initialize motor controllers
        self.motor_controllers = self.initialize_controllers()

    def joint_command_callback(self, msg):
        """Handle incoming joint commands"""
        self.desired_positions = msg.data

    def control_loop(self):
        """Main control loop"""
        # Read current joint states
        current_states = self.read_joint_states()

        # Compute control commands
        commands = []
        for i, (desired_pos, current_state) in enumerate(zip(self.desired_positions, current_states)):
            command = self.motor_controllers[i].compute_command(
                desired_pos, current_state.position, 0, current_state.velocity, 0.001
            )
            commands.append(command)

        # Send commands to motors
        self.send_motor_commands(commands)

        # Publish joint states
        self.publish_joint_states(current_states)

    def read_joint_states(self):
        """Read current joint states from hardware"""
        # Implementation depends on hardware interface
        pass

    def send_motor_commands(self, commands):
        """Send computed commands to motor hardware"""
        # Implementation depends on communication protocol
        pass
```

## Performance Optimization

### Model-Based Feedforward

Using dynamic models to improve tracking performance:

```python
class ModelBasedController:
    def __init__(self, robot_model):
        self.model = robot_model
        self.inverse_dynamics = InverseDynamicsCalculator(robot_model)

    def compute_model_based_command(self, desired_trajectory, current_state):
        """Compute control command using model-based feedforward"""
        # Inverse dynamics to compute required torques
        feedforward_torque = self.inverse_dynamics.calculate(
            desired_trajectory.positions,
            desired_trajectory.velocities,
            desired_trajectory.accelerations
        )

        # Feedback control for error correction
        feedback_torque = self.feedback_controller.compute_command(
            desired_trajectory.positions,
            current_state.positions,
            desired_trajectory.velocities,
            current_state.velocities
        )

        # Combined command
        total_torque = feedforward_torque + feedback_torque

        return total_torque
```

### Adaptive Control

Adjusting control parameters based on system behavior:

```python
class AdaptiveController:
    def __init__(self, initial_params):
        self.params = initial_params
        self.param_adaptation_rate = 0.01

    def update_parameters(self, tracking_error, regressor_vector):
        """Update controller parameters based on tracking error"""
        # Parameter update law
        param_update = (self.param_adaptation_rate *
                       tracking_error * regressor_vector)
        self.params += param_update

        # Apply parameter constraints
        self.params = np.clip(self.params, self.min_params, self.max_params)

        return self.params

    def compute_adaptive_command(self, state_error, state_derivative_error):
        """Compute command with adaptive parameters"""
        # Use adapted parameters in control law
        command = (self.params['kp'] * state_error +
                  self.params['kd'] * state_derivative_error)

        return command
```

## Conclusion

Motor control systems form the critical interface between high-level planning and physical execution in humanoid robots. From basic PID control to advanced model-based and adaptive techniques, these systems must operate in real-time while ensuring safety, precision, and robustness. The integration of motor control with the broader ROS 2 communication framework enables coordinated whole-body behavior while maintaining the low-level precision required for stable humanoid operation.

The next section will provide practical examples and exercises to reinforce the concepts learned in this module, allowing students to implement and experiment with the control strategies discussed.