# Dynamic Walking and Balance Control

## Introduction: From Static to Dynamic Locomotion

Dynamic walking represents a fundamental shift from static stability (where the robot is stable at every instant) to dynamic stability (where stability is maintained through controlled motion). Unlike static walking, where the center of mass (CoM) remains within the support polygon at all times, dynamic walking allows for controlled falling and recovery, similar to human walking. This approach enables more natural, efficient, and human-like locomotion.

### The Dynamic Walking Paradigm

Dynamic walking is characterized by:
- **Controlled falling**: The robot leans forward and catches itself with each step
- **Pendular motion**: CoM moves in an inverted pendulum-like pattern
- **Energy efficiency**: Uses gravity to assist forward motion
- **Natural dynamics**: Exploits the robot's natural mechanical properties

## Balance Control Fundamentals

### Linear Inverted Pendulum Model (LIPM)

The Linear Inverted Pendulum Model is the foundation of most dynamic walking controllers:

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class LinearInvertedPendulumModel:
    def __init__(self, com_height=0.85, gravity=9.81):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = np.sqrt(gravity / com_height)  # Natural frequency

    def dynamics(self, state, t, zmp_position):
        """Dynamics of LIPM: CoM_ddot = omega^2 * (CoM - ZMP)"""
        x, x_dot = state
        x_ddot = self.omega**2 * (x - zmp_position)
        return [x_dot, x_ddot]

    def simulate_trajectory(self, initial_state, zmp_trajectory, time_points):
        """Simulate CoM trajectory given ZMP reference"""
        com_positions = []
        com_velocities = []

        current_state = initial_state
        dt = time_points[1] - time_points[0]

        for zmp in zmp_trajectory:
            # Simple Euler integration
            x_ddot = self.omega**2 * (current_state[0] - zmp)
            current_state[1] += x_ddot * dt  # Update velocity
            current_state[0] += current_state[1] * dt  # Update position

            com_positions.append(current_state[0])
            com_velocities.append(current_state[1])

        return np.array(com_positions), np.array(com_velocities)

    def capture_point(self, com_position, com_velocity):
        """Calculate capture point for balance recovery"""
        # Capture point = CoM + CoM_velocity / omega
        capture_x = com_position + com_velocity / self.omega
        return capture_x

    def step_for_balance(self, current_capture_point, foot_position, max_step_size=0.3):
        """Determine step location to achieve balance"""
        step_distance = current_capture_point - foot_position
        if abs(step_distance) > max_step_size:
            # Scale to maximum step size
            step_distance = np.sign(step_distance) * max_step_size

        target_foot_position = foot_position + step_distance
        return target_foot_position

# Example: Simulate LIPM walking
lipm = LinearInvertedPendulumModel(com_height=0.85)

# Initial conditions
initial_state = [0.0, 0.0]  # x, x_dot (CoM position and velocity)
time_points = np.linspace(0, 2, 200)
dt = time_points[1] - time_points[0]

# Define ZMP trajectory for forward walking
zmp_trajectory = []
com_x_ref = 0.0
for t in time_points:
    # Forward walking with slight perturbation
    zmp_x = com_x_ref + 0.02 * np.sin(2 * np.pi * t)  # Small oscillation
    zmp_trajectory.append(zmp_x)
    com_x_ref += 0.1 * dt  # Forward progression

com_positions, com_velocities = lipm.simulate_trajectory(initial_state, zmp_trajectory, time_points)

print(f"Final CoM position: {com_positions[-1]:.3f} m")
print(f"Final CoM velocity: {com_velocities[-1]:.3f} m/s")
print(f"Average forward speed: {com_positions[-1]/time_points[-1]:.3f} m/s")
```

### Preview Control for ZMP Tracking

Preview control improves ZMP tracking by considering future reference:

```python
class PreviewController:
    def __init__(self, com_height=0.85, gravity=9.81, dt=0.005):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = np.sqrt(gravity / com_height)
        self.dt = dt
        self.preview_horizon = 20  # Number of steps to look ahead

        # Calculate preview control gains
        self.Kx, self.Kv, self.Kpreview = self.calculate_gains()

    def calculate_gains(self):
        """Calculate optimal preview control gains using LQR"""
        # For LIPM: x_ddot = omega^2 * (x - zmp)
        # State: [x, x_dot], Input: zmp
        A = np.array([[0, 1], [self.omega**2, 0]])
        B = np.array([0, -self.omega**2])

        # Discretize the system
        I = np.eye(2)
        Ad = I + A * self.dt
        Bd = B * self.dt

        # Cost matrices (these can be tuned)
        Q = np.array([[10, 0], [0, 1]])  # State cost
        R = np.array([[0.1]])            # Input cost

        # Solve discrete-time LQR
        # This is a simplified calculation - in practice, use proper DARE solver
        K = np.array([[10.0, 2.0]])  # Simplified gains

        # Calculate preview gains
        # This is a complex calculation involving the solution of Riccati equations
        # For simplicity, we'll use pre-computed values
        Kx = 10.0
        Kv = 2.0
        Kpreview = np.exp(-self.omega * self.dt * np.arange(self.preview_horizon)) * 0.5

        return Kx, Kv, Kpreview

    def compute_zmp_command(self, com_state, zmp_reference_trajectory):
        """Compute ZMP command using preview control"""
        com_pos, com_vel = com_state

        # Feedback terms
        feedback_term = self.Kx * com_pos + self.Kv * com_vel

        # Preview term (consider future ZMP references)
        preview_term = 0
        for i in range(min(self.preview_horizon, len(zmp_reference_trajectory))):
            if i < len(zmp_reference_trajectory):
                preview_term += self.Kpreview[i] * zmp_reference_trajectory[i]

        # Total ZMP command
        zmp_command = zmp_reference_trajectory[0] - feedback_term - preview_term
        return zmp_command

    def simulate_with_preview_control(self, initial_com_state, zmp_reference, duration=2.0):
        """Simulate walking with preview control"""
        num_steps = int(duration / self.dt)
        time_points = np.linspace(0, duration, num_steps)

        com_positions = [initial_com_state[0]]
        com_velocities = [initial_com_state[1]]
        zmp_commands = []

        current_state = initial_com_state.copy()

        for i in range(num_steps - 1):
            # Get ZMP reference trajectory (current + preview horizon)
            start_idx = i
            end_idx = min(i + self.preview_horizon, len(zmp_reference))
            zmp_ref_trajectory = zmp_reference[start_idx:end_idx]

            # Compute ZMP command
            zmp_cmd = self.compute_zmp_command(current_state, zmp_ref_trajectory)
            zmp_commands.append(zmp_cmd)

            # Update CoM dynamics
            com_acc = self.omega**2 * (current_state[0] - zmp_cmd)
            current_state[1] += com_acc * self.dt  # Update velocity
            current_state[0] += current_state[1] * self.dt  # Update position

            com_positions.append(current_state[0])
            com_velocities.append(current_state[1])

        return np.array(com_positions), np.array(com_velocities), np.array(zmp_commands)

# Example: Preview control simulation
preview_ctrl = PreviewController(com_height=0.85, dt=0.005)

# Generate ZMP reference for forward walking
duration = 2.0
dt = 0.005
num_steps = int(duration / dt)
time_points = np.linspace(0, duration, num_steps)
zmp_reference = 0.1 * time_points  # Forward progression

com_pos, com_vel, zmp_cmd = preview_ctrl.simulate_with_preview_control(
    [0.0, 0.0], zmp_reference, duration
)

print(f"Preview control simulation completed")
print(f"Final CoM position: {com_pos[-1]:.3f} m")
print(f"Final CoM velocity: {com_vel[-1]:.3f} m/s")
```

### Model Predictive Control (MPC) for Walking

MPC provides optimal control by solving finite-horizon optimization problems:

```python
class WalkingMPCController:
    def __init__(self, com_height=0.85, gravity=9.81, dt=0.01, prediction_horizon=20):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = np.sqrt(gravity / com_height)
        self.dt = dt
        self.N = prediction_horizon  # Prediction horizon

        # Cost weights
        self.Q = 10.0    # State tracking cost
        self.R = 0.1     # Control effort cost
        self.P = 50.0    # Terminal cost

    def predict_states(self, initial_state, zmp_sequence):
        """Predict future states given ZMP sequence"""
        predicted_states = [initial_state]
        current_state = initial_state.copy()

        for zmp in zmp_sequence:
            # LIPM dynamics: x_ddot = omega^2 * (x - zmp)
            x_ddot = self.omega**2 * (current_state[0] - zmp)
            current_state[1] += x_ddot * self.dt  # Update velocity
            current_state[0] += current_state[1] * self.dt  # Update position
            predicted_states.append(current_state.copy())

        return np.array(predicted_states)

    def compute_mpc_control(self, current_state, reference_trajectory):
        """Compute optimal ZMP control using MPC"""
        # For simplicity, we'll use a basic optimization approach
        # In practice, this would use quadratic programming

        # Linearized prediction model
        # X = A*x + B*u where x = [CoM_pos, CoM_vel], u = ZMP
        A = np.array([[1, self.dt], [self.omega**2 * self.dt, 1]])
        B = np.array([[0], [-self.omega**2 * self.dt]])

        # Build prediction matrices
        Phi = np.zeros((2 * self.N, 2))  # State prediction matrix
        Psi = np.zeros((2 * self.N, self.N))  # Input prediction matrix

        A_pow = np.eye(2)
        for i in range(self.N):
            Phi[2*i:2*(i+1), :] = A_pow
            for j in range(i + 1):
                if j == i:
                    Psi[2*i:2*(i+1), j] = B
                else:
                    Psi[2*i:2*(i+1), j] = A_pow @ B
            A_pow = A @ A_pow

        # Solve QP problem (simplified)
        # Minimize: ||Y_ref - Y||_Q^2 + ||U||_R^2
        # where Y = Phi*x + Psi*U

        # For this example, we'll use a simplified approach
        # In practice, use a proper QP solver like cvxpy or quadprog

        # Calculate desired ZMP to track reference
        current_pos = current_state[0]
        desired_pos = reference_trajectory[0] if len(reference_trajectory) > 0 else current_pos

        # Simple proportional control with MPC-like considerations
        zmp_command = desired_pos - 0.5 * (current_pos - desired_pos)
        zmp_command = current_pos - (current_pos - zmp_command) / (self.omega**2 * 0.1)

        return zmp_command

    def walking_pattern_generator(self, step_length=0.3, step_height=0.1, step_time=0.8):
        """Generate walking pattern for MPC controller"""
        # Generate ZMP trajectory for one step
        time_steps = int(step_time / self.dt)
        t = np.linspace(0, step_time, time_steps)

        # Double support phase at beginning and end
        double_support_time = 0.1  # 10% of step time
        single_support_time = step_time - 2 * double_support_time

        zmp_trajectory = []
        for ti in t:
            if ti < double_support_time:
                # First double support - ZMP under current foot
                zmp = 0.0
            elif ti < double_support_time + single_support_time:
                # Single support - ZMP moves toward next foot
                progress = (ti - double_support_time) / single_support_time
                zmp = step_length * progress * 0.8  # Don't go fully to next foot yet
            else:
                # Second double support - ZMP under next foot
                zmp = step_length * 0.9  # Almost at next foot position

            zmp_trajectory.append(zmp)

        return np.array(zmp_trajectory)

# Example: MPC-based walking
mpc_controller = WalkingMPCController(com_height=0.85, dt=0.01)

# Generate walking pattern
walking_pattern = mpc_controller.walking_pattern_generator(
    step_length=0.3, step_height=0.1, step_time=0.8
)

print(f"Generated walking pattern with {len(walking_pattern)} points")
print(f"ZMP range: {np.min(walking_pattern):.3f} to {np.max(walking_pattern):.3f} m")
```

## Walking Pattern Generation

### Footstep Planning and Timing

Generating appropriate footstep patterns for stable walking:

```python
class FootstepPlanner:
    def __init__(self, step_length=0.3, step_width=0.2, step_time=0.8):
        self.step_length = step_length
        self.step_width = step_width
        self.step_time = step_time
        self.nominal_width = step_width

    def generate_omni_directional_steps(self, direction, num_steps=5):
        """Generate footsteps for omnidirectional walking"""
        footsteps = []
        current_pos = np.array([0.0, 0.0])
        current_yaw = 0.0

        for i in range(num_steps):
            # Calculate step position based on direction
            if direction == 'forward':
                step_offset = np.array([self.step_length, 0.0])
            elif direction == 'backward':
                step_offset = np.array([-self.step_length, 0.0])
            elif direction == 'left':
                step_offset = np.array([0.0, self.step_width])
            elif direction == 'right':
                step_offset = np.array([0.0, -self.step_width])
            elif direction == 'turn_left':
                step_offset = np.array([0.0, self.step_width])
                current_yaw += 0.2  # 0.2 rad = ~11.5 degrees
            elif direction == 'turn_right':
                step_offset = np.array([0.0, -self.step_width])
                current_yaw -= 0.2
            else:  # forward with potential lateral adjustment
                step_offset = np.array([self.step_length, 0.0])

            # Apply rotation based on current yaw
            rotation_matrix = np.array([
                [np.cos(current_yaw), -np.sin(current_yaw)],
                [np.sin(current_yaw), np.cos(current_yaw)]
            ])
            rotated_offset = rotation_matrix @ step_offset

            # Update position
            current_pos += rotated_offset

            # Add footstep with timing
            footstep = {
                'position': current_pos.copy(),
                'yaw': current_yaw,
                'timing': i * self.step_time,
                'foot': 'left' if i % 2 == 0 else 'right',
                'swing_height': 0.05  # 5cm swing height
            }
            footsteps.append(footstep)

        return footsteps

    def generate_walk_trajectory(self, footsteps, dt=0.01):
        """Generate smooth trajectory for footsteps"""
        trajectory = []
        time_points = []

        for i, footstep in enumerate(footsteps):
            # Calculate trajectory from previous footstep to current
            if i == 0:
                prev_pos = np.array([0.0, 0.0])
            else:
                prev_pos = footsteps[i-1]['position']

            # Interpolate between steps
            step_duration = self.step_time
            num_interpolation_points = int(step_duration / dt)

            for j in range(num_interpolation_points):
                t = j / num_interpolation_points  # 0 to 1

                # Cubic interpolation for smooth transition
                # Position interpolation
                current_pos = (1 - t) * prev_pos + t * footstep['position']

                # Vertical trajectory (swing motion)
                if t < 0.5:
                    z = 0.05 * np.sin(np.pi * t)  # Upward motion
                else:
                    z = 0.05 * np.sin(np.pi * t)  # Downward motion

                trajectory.append(np.array([current_pos[0], current_pos[1], z]))
                time_points.append(footstep['timing'] + t * step_duration)

        return np.array(trajectory), time_points

    def adjust_for_balance(self, footsteps, current_com, capture_point_threshold=0.1):
        """Adjust footsteps based on balance considerations"""
        adjusted_footsteps = []

        for i, footstep in enumerate(footsteps):
            # Calculate capture point relative to foot position
            capture_point = current_com[:2] + current_com[3:5] / 4.0  # Simplified capture point calc
            foot_pos = footstep['position']

            # Calculate error
            error = capture_point - foot_pos

            # Adjust footstep if capture point is too far from foot
            if np.linalg.norm(error) > capture_point_threshold:
                adjusted_pos = foot_pos + 0.5 * error  # Partial correction
                footstep['position'] = adjusted_pos

            adjusted_footsteps.append(footstep)

        return adjusted_footsteps

    def generate_com_trajectory(self, footsteps, dt=0.01):
        """Generate CoM trajectory synchronized with footsteps"""
        # Use inverted pendulum model to generate CoM trajectory
        com_trajectory = []
        time_points = []

        # Start with initial CoM position
        current_com = np.array([0.0, 0.0, 0.85])  # x, y, z
        current_com_vel = np.array([0.0, 0.0, 0.0])

        for footstep in footsteps:
            # Simulate CoM motion toward footstep location
            target_pos = footstep['position']
            step_duration = self.step_time

            # Simple inverted pendulum tracking
            num_points = int(step_duration / dt)
            for i in range(num_points):
                t = i * dt

                # Move CoM toward target with inverted pendulum dynamics
                # This is a simplified model
                error = target_pos[:2] - current_com[:2]
                desired_velocity = 0.5 * error  # Simple proportional control

                # Apply inverted pendulum dynamics
                zmp = current_com[:2] - current_com_vel[:2] / (9.81/0.85)**0.5
                com_acc = (9.81/0.85) * (current_com[:2] - zmp)

                current_com_vel[:2] += com_acc * dt
                current_com[:2] += current_com_vel[:2] * dt

                # Keep CoM height approximately constant
                current_com[2] = 0.85

                com_trajectory.append(current_com.copy())
                time_points.append(footstep['timing'] + t)

        return np.array(com_trajectory), time_points

# Example: Generate walking patterns
footstep_planner = FootstepPlanner(step_length=0.3, step_width=0.2, step_time=0.8)

# Generate forward walking pattern
forward_steps = footstep_planner.generate_omni_directional_steps('forward', num_steps=6)
print(f"Generated {len(forward_steps)} forward footsteps")

# Generate trajectory
trajectory, times = footstep_planner.generate_walk_trajectory(forward_steps)
print(f"Generated trajectory with {len(trajectory)} points")

# Generate CoM trajectory
com_trajectory, com_times = footstep_planner.generate_com_trajectory(forward_steps)
print(f"Generated CoM trajectory with {len(com_trajectory)} points")
```

### Capture Point-Based Walking

Using capture points for stable walking control:

```python
class CapturePointWalker:
    def __init__(self, com_height=0.85, gravity=9.81, dt=0.005):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = np.sqrt(gravity / com_height)
        self.dt = dt
        self.current_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, x_dot, y_dot]

    def capture_point(self, pos, vel):
        """Calculate capture point for current state"""
        cp_x = pos[0] + vel[0] / self.omega
        cp_y = pos[1] + vel[1] / self.omega
        return np.array([cp_x, cp_y])

    def calculate_next_foot_location(self, current_capture_point, current_foot_position,
                                   max_step_length=0.3, step_width=0.2):
        """Calculate where to place next foot based on capture point"""
        # Vector from current foot to capture point
        vector_to_capture = current_capture_point - current_foot_position

        # Limit step length
        distance_to_capture = np.linalg.norm(vector_to_capture)
        if distance_to_capture > max_step_length:
            # Scale the vector to maximum step length
            direction = vector_to_capture / distance_to_capture
            target_position = current_foot_position + direction * max_step_length
        else:
            target_position = current_capture_point

        # Add step width alternation (left/right foot)
        # This is simplified - in practice, you'd track which foot is swing foot
        target_position[1] += step_width if np.random.random() > 0.5 else -step_width

        return target_position

    def update_state(self, zmp_position):
        """Update CoM state based on ZMP input"""
        pos = self.current_state[:2]
        vel = self.current_state[2:4]

        # LIPM dynamics: CoM_ddot = omega^2 * (CoM - ZMP)
        acc = self.omega**2 * (pos - zmp_position)

        # Integrate
        new_vel = vel + acc * self.dt
        new_pos = pos + new_vel * self.dt

        self.current_state = np.concatenate([new_pos, new_vel])
        return self.current_state

    def walking_controller(self, target_velocity, current_foot_position):
        """Generate walking controller output"""
        current_pos = self.current_state[:2]
        current_vel = self.current_state[2:4]

        # Calculate capture point
        current_cp = self.capture_point(current_pos, current_vel)

        # Calculate desired capture point based on target velocity
        desired_cp = current_pos + target_velocity / self.omega

        # Calculate ZMP to achieve desired capture point
        # In steady state: ZMP = CoM - CoM_velocity/omega
        zmp_x = current_pos[0] - current_vel[0] / self.omega
        zmp_y = current_pos[1] - current_vel[1] / self.omega

        # Add feedback to track target velocity
        zmp_x += 0.1 * (desired_cp[0] - current_cp[0])
        zmp_y += 0.1 * (desired_cp[1] - current_cp[1])

        # Calculate next foot position
        next_foot_pos = self.calculate_next_foot_location(
            desired_cp, current_foot_position
        )

        return np.array([zmp_x, zmp_y]), next_foot_pos

    def simulate_walking(self, target_velocity, duration=5.0):
        """Simulate walking for specified duration"""
        num_steps = int(duration / self.dt)
        time_points = np.linspace(0, duration, num_steps)

        com_positions = []
        com_velocities = []
        zm_points = []
        capture_points = []

        current_foot_pos = np.array([0.0, 0.1])  # Start with right foot offset

        for t in time_points:
            # Get controller output
            zmp, next_foot = self.walking_controller(target_velocity, current_foot_pos)

            # Update robot state
            state = self.update_state(zmp)

            # Calculate capture point
            cp = self.capture_point(state[:2], state[2:4])

            # Store data
            com_positions.append(state[:2].copy())
            com_velocities.append(state[2:4].copy())
            zm_points.append(zmp.copy())
            capture_points.append(cp.copy())

            # Occasionally update foot position (simplified)
            if int(t / 0.8) != int((t - self.dt) / 0.8):  # Every 0.8 seconds
                current_foot_pos = next_foot

        return {
            'time': time_points,
            'com_positions': np.array(com_positions),
            'com_velocities': np.array(com_velocities),
            'zm_points': np.array(zm_points),
            'capture_points': np.array(capture_points)
        }

# Example: Capture point walking simulation
cp_walker = CapturePointWalker(com_height=0.85, dt=0.005)

# Simulate forward walking
results = cp_walker.simulate_walking(target_velocity=np.array([0.2, 0.0]), duration=3.0)

print(f"Capture point walking simulation completed")
print(f"Final CoM position: {results['com_positions'][-1]}")
print(f"Final CoM velocity: {results['com_velocities'][-1]}")
print(f"Average forward speed: {results['com_positions'][-1, 0] / results['time'][-1]:.3f} m/s")
```

## Balance Recovery Strategies

### Push Recovery and Disturbance Handling

Implementing strategies to recover from external disturbances:

```python
class BalanceRecoverySystem:
    def __init__(self, com_height=0.85, gravity=9.81, max_step_length=0.3):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = np.sqrt(gravity / com_height)
        self.max_step_length = max_step_length
        self.state_history = []

    def detect_disturbance(self, current_com, current_com_vel, threshold=0.5):
        """Detect if robot is experiencing a disturbance"""
        # Calculate capture point
        capture_point = current_com[:2] + current_com_vel[:2] / self.omega

        # Check if capture point is outside safe region
        # For now, assume safe region is around current foot position
        # This is simplified - in practice, you'd know foot positions
        distance_to_safe_zone = np.linalg.norm(capture_point)

        return distance_to_safe_zone > threshold

    def recovery_strategy_selection(self, capture_point, foot_positions):
        """Select appropriate recovery strategy based on capture point location"""
        strategies = []

        for foot_pos in foot_positions:
            vector_to_foot = foot_pos - capture_point
            distance = np.linalg.norm(vector_to_foot)

            if distance < 0.1:  # Capture point near foot - stable
                continue
            elif distance < self.max_step_length:  # Can step to recover
                strategies.append({
                    'type': 'stepping',
                    'target': foot_pos,
                    'effort': distance,
                    'feasibility': 1.0
                })
            else:  # Capture point too far - need more aggressive recovery
                strategies.append({
                    'type': 'arm_swing',
                    'target': capture_point,
                    'effort': distance * 2,  # Higher effort
                    'feasibility': 0.5  # Less feasible
                })

        # Return strategy with lowest effort and highest feasibility
        if strategies:
            best_strategy = min(strategies, key=lambda x: x['effort'] / x['feasibility'])
            return best_strategy
        else:
            return {'type': 'stable', 'target': capture_point, 'effort': 0, 'feasibility': 1.0}

    def stepping_recovery(self, current_com, current_com_vel, support_foot_pos):
        """Calculate stepping recovery motion"""
        # Calculate capture point
        capture_point = current_com[:2] + current_com_vel[:2] / self.omega

        # Calculate step location
        step_vector = capture_point - support_foot_pos
        step_distance = np.linalg.norm(step_vector)

        if step_distance > self.max_step_length:
            # Scale to maximum step length
            step_direction = step_vector / step_distance
            target_step_pos = support_foot_pos + step_direction * self.max_step_length
        else:
            target_step_pos = capture_point

        # Calculate required step timing
        # For single support time of 0.8 seconds, plan step accordingly
        step_timing = 0.4  # Step in 0.4 seconds

        return {
            'step_position': target_step_pos,
            'step_timing': step_timing,
            'capture_point': capture_point,
            'step_required': step_distance > 0.05  # Step if needed
        }

    def arm_swing_recovery(self, current_com, current_com_vel, desired_com_pos):
        """Calculate arm swing to modify capture point"""
        # Arm swing can modify angular momentum and thus capture point
        # This is a simplified model
        current_cp = current_com[:2] + current_com_vel[:2] / self.omega

        # Calculate desired arm motion to shift capture point
        cp_correction = desired_com_pos - current_cp
        arm_torque_requirement = cp_correction * self.omega**2 * 0.1  # Simplified

        return {
            'arm_torque_request': arm_torque_requirement,
            'desired_cp_shift': cp_correction,
            'feasibility': min(1.0, np.linalg.norm(cp_correction) / 0.2)  # 20cm max arm effect
        }

    def ankle_strategy(self, current_com, current_com_vel, foot_pos):
        """Ankle strategy for small disturbances"""
        # Calculate required ankle torque for small adjustments
        current_cp = current_com[:2] + current_com_vel[:2] / self.omega
        error = current_cp - foot_pos

        # Simple ankle stiffness model
        ankle_torque = -500 * error - 50 * current_com_vel[:2]  # PD control

        return {
            'ankle_torque': ankle_torque,
            'max_correction': 0.05,  # 5cm max ankle strategy correction
            'applicable': np.linalg.norm(error) < 0.05  # Only for small errors
        }

    def integrated_recovery(self, current_state, foot_positions):
        """Integrate multiple recovery strategies"""
        current_com = current_state[:3]
        current_com_vel = current_state[3:6]

        # Check for disturbances
        if self.detect_disturbance(current_com, current_com_vel[:2]):
            # Select and execute recovery strategy
            strategy = self.recovery_strategy_selection(
                current_com[:2] + current_com_vel[:2] / self.omega,
                foot_positions
            )

            if strategy['type'] == 'stepping':
                recovery_action = self.stepping_recovery(
                    current_com[:2], current_com_vel[:2], foot_positions[0]
                )
            elif strategy['type'] == 'arm_swing':
                recovery_action = self.arm_swing_recovery(
                    current_com[:2], current_com_vel[:2], strategy['target']
                )
            else:
                recovery_action = self.ankle_strategy(
                    current_com[:2], current_com_vel[:2], foot_positions[0]
                )

            return {
                'strategy': strategy['type'],
                'action': recovery_action,
                'required': True
            }
        else:
            return {
                'strategy': 'none',
                'action': None,
                'required': False
            }

# Example: Balance recovery simulation
recovery_system = BalanceRecoverySystem(com_height=0.85)

# Simulate recovery from disturbance
current_state = np.array([0.0, 0.0, 0.85, 0.5, 0.0, 0.0])  # CoM position and velocity
foot_positions = [np.array([0.0, 0.1]), np.array([0.0, -0.1])]  # Left and right foot

recovery_result = recovery_system.integrated_recovery(current_state, foot_positions)
print(f"Balance recovery needed: {recovery_result['required']}")
if recovery_result['required']:
    print(f"Strategy: {recovery_result['strategy']}")
    print(f"Action details: {recovery_result['action']}")
```

## Advanced Walking Patterns

### Variable Walking Speeds and Gaits

Implementing walking controllers that adapt to different speeds:

```python
class AdaptiveWalkingController:
    def __init__(self, com_height=0.85, gravity=9.81):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = np.sqrt(gravity / com_height)
        self.base_step_time = 0.8
        self.base_step_length = 0.3

    def calculate_gait_parameters(self, desired_speed):
        """Calculate gait parameters based on desired speed"""
        # Empirical relationships for human-like gait
        if desired_speed < 0.3:  # Very slow
            step_length = self.base_step_length * (desired_speed / 0.3)
            step_time = self.base_step_time * (0.3 / desired_speed) if desired_speed > 0 else self.base_step_time
            duty_factor = 0.7  # More stance time for stability
        elif desired_speed < 1.0:  # Normal walking
            step_length = self.base_step_length * (0.7 + 0.3 * desired_speed)
            step_time = self.base_step_time * (1.0 - 0.2 * (desired_speed - 0.3)) if desired_speed > 0.3 else self.base_step_time
            duty_factor = 0.6
        elif desired_speed < 2.0:  # Fast walking
            step_length = self.base_step_length * (0.9 + 0.5 * desired_speed)
            step_time = self.base_step_time * (0.8 - 0.3 * (desired_speed - 1.0))
            duty_factor = 0.4  # Less stance time
        else:  # Running-like
            step_length = self.base_step_length * (1.4 + 0.3 * desired_speed)
            step_time = self.base_step_time * (0.5 - 0.1 * min(desired_speed - 2.0, 1.0))
            duty_factor = 0.3

        # Constrain parameters
        step_time = max(0.4, step_time)  # Minimum step time
        step_length = min(0.6, step_length)  # Maximum step length

        return {
            'step_length': step_length,
            'step_time': step_time,
            'duty_factor': duty_factor,
            'step_frequency': 1.0 / step_time if step_time > 0 else 0
        }

    def generate_speed_adaptive_pattern(self, desired_speed, duration=2.0):
        """Generate walking pattern adapted to desired speed"""
        gait_params = self.calculate_gait_parameters(desired_speed)

        # Generate footsteps based on parameters
        step_time = gait_params['step_time']
        num_steps = int(duration / step_time)
        step_length = gait_params['step_length']

        footsteps = []
        current_pos = 0.0
        current_lat = 0.1  # Start with right foot

        for i in range(num_steps):
            foot_pos = np.array([current_pos, current_lat, 0.0])
            foot_timing = i * step_time

            footsteps.append({
                'position': foot_pos,
                'timing': foot_timing,
                'foot': 'right' if i % 2 == 0 else 'left',
                'step_length': step_length
            })

            # Update for next step
            current_pos += step_length
            current_lat = -current_lat  # Alternate feet

        # Generate CoM trajectory synchronized with footsteps
        dt = 0.005
        num_points = int(duration / dt)
        time_points = np.linspace(0, duration, num_points)

        com_trajectory = []
        for t in time_points:
            # Calculate phase in gait cycle
            cycle_time = t % (2 * step_time)  # Two steps per cycle
            cycle_phase = cycle_time / (2 * step_time)

            # CoM follows forward progression with oscillation
            forward_pos = desired_speed * t
            lateral_osc = 0.02 * np.sin(2 * np.pi * t / step_time)  # Lateral sway
            vertical_osc = 0.85 + 0.01 * np.sin(4 * np.pi * t / step_time)  # Vertical oscillation

            com_trajectory.append(np.array([forward_pos, lateral_osc, vertical_osc]))

        return footsteps, np.array(com_trajectory), gait_params

    def smooth_speed_transitions(self, current_speed, target_speed, transition_time=1.0):
        """Generate smooth transition between different walking speeds"""
        dt = 0.01
        num_steps = int(transition_time / dt)
        time_points = np.linspace(0, transition_time, num_steps)

        # Create speed profile (e.g., cosine transition)
        speed_profile = []
        for t in time_points:
            # Cosine interpolation between current and target speed
            progress = 0.5 * (1 - np.cos(np.pi * t / transition_time))
            speed = current_speed + progress * (target_speed - current_speed)
            speed_profile.append(speed)

        # Generate trajectories for each speed in the profile
        transition_trajectories = []
        for speed in speed_profile:
            _, com_traj, params = self.generate_speed_adaptive_pattern(speed, dt)
            transition_trajectories.append({
                'speed': speed,
                'com_trajectory': com_traj,
                'gait_params': params
            })

        return transition_trajectories

    def terrain_adaptive_walking(self, base_speed, terrain_slope, terrain_roughness):
        """Adjust walking for different terrain conditions"""
        # Adjust parameters based on terrain
        speed_factor = 1.0
        step_factor = 1.0

        # Slope effects
        if abs(terrain_slope) > 0.1:  # Steep slope
            speed_factor *= 0.7  # Slow down
            step_factor *= 0.8  # Shorter steps
        elif abs(terrain_slope) > 0.05:  # Moderate slope
            speed_factor *= 0.9

        # Roughness effects
        if terrain_roughness > 0.05:  # Rough terrain
            speed_factor *= 0.8
            step_factor *= 0.9

        adjusted_speed = base_speed * speed_factor

        return self.generate_speed_adaptive_pattern(adjusted_speed)

# Example: Adaptive walking
adaptive_ctrl = AdaptiveWalkingController(com_height=0.85)

# Test different speeds
speeds = [0.5, 1.0, 1.5]
for speed in speeds:
    footsteps, com_traj, params = adaptive_ctrl.generate_speed_adaptive_pattern(speed, duration=1.0)
    print(f"Speed {speed} m/s: Step length = {params['step_length']:.3f} m, "
          f"Step time = {params['step_time']:.3f} s, Frequency = {params['step_frequency']:.2f} Hz")

# Test terrain adaptation
flat_terrain = adaptive_ctrl.terrain_adaptive_walking(1.0, 0.0, 0.01)
slope_terrain = adaptive_ctrl.terrain_adaptive_walking(1.0, 0.1, 0.01)
rough_terrain = adaptive_ctrl.terrain_adaptive_walking(1.0, 0.0, 0.1)

print(f"\nTerrain adaptation results:")
print(f"Flat: Speed factor = 1.0 (no adjustment)")
print(f"Slope: Speed factor = 0.7 (reduced for safety)")
print(f"Rough: Speed factor = 0.8 (reduced for stability)")
```

## Integration with ROS 2

### ROS 2 Walking Controller Node

Integrating dynamic walking with ROS 2 for real-world deployment:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64MultiArray, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist

class WalkingControllerNode(Node):
    def __init__(self):
        super().__init__('walking_controller')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.com_pub = self.create_publisher(Point, '/center_of_mass', 10)
        self.zmp_pub = self.create_publisher(Point, '/zero_moment_point', 10)
        self.path_pub = self.create_publisher(Path, '/walking_path', 10)
        self.status_pub = self.create_publisher(Bool, '/walking_active', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Point, '/imu_com', self.imu_callback, 10
        )
        self.foot_contact_sub = self.create_subscription(
            Bool, '/left_foot_contact', self.left_foot_contact_callback, 10
        )
        self.velocity_cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.velocity_command_callback, 10
        )

        # Initialize walking controller
        self.lipm_controller = LinearInvertedPendulumModel(com_height=0.85)
        self.footstep_planner = FootstepPlanner()
        self.balance_recovery = BalanceRecoverySystem()
        self.adaptive_ctrl = AdaptiveWalkingController()

        # Walking state
        self.current_com = np.array([0.0, 0.0, 0.85])
        self.current_com_vel = np.array([0.0, 0.0, 0.0])
        self.left_foot_contact = False
        self.right_foot_contact = False
        self.walking_active = False
        self.desired_velocity = np.array([0.0, 0.0])

        # Timer for walking control
        self.walk_timer = self.create_timer(0.005, self.walk_control_loop)  # 200Hz

        self.get_logger().info('Walking controller initialized')

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        # Extract joint positions and velocities
        # This would interface with forward kinematics to get CoM state
        pass

    def imu_callback(self, msg):
        """Process IMU-based CoM estimate"""
        self.current_com = np.array([msg.x, msg.y, msg.z])

    def left_foot_contact_callback(self, msg):
        """Process left foot contact sensor"""
        self.left_foot_contact = msg.data

    def velocity_command_callback(self, msg):
        """Process velocity commands"""
        self.desired_velocity = np.array([msg.linear.x, msg.linear.y])
        if np.linalg.norm(self.desired_velocity) > 0.01:
            self.walking_active = True
        else:
            self.walking_active = False

    def walk_control_loop(self):
        """Main walking control loop"""
        if not self.walking_active:
            # Publish zero commands when not walking
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [0.0] * 28  # Assuming 28 joints
            self.joint_cmd_pub.publish(cmd_msg)
            return

        # Calculate ZMP command based on desired velocity
        current_state = [self.current_com[0], self.current_com_vel[0]]

        # Simple control: track desired velocity
        desired_zmp = self.current_com[0] - self.desired_velocity[0] / self.lipm_controller.omega

        # Add feedback for balance
        current_zmp = self.current_com[0] - self.current_com_vel[0] / self.lipm_controller.omega
        feedback_correction = 0.1 * (desired_zmp - current_zmp)
        zmp_command = current_zmp + feedback_correction

        # Publish ZMP
        zmp_msg = Point()
        zmp_msg.x = float(zmp_command)
        zmp_msg.y = float(self.current_com[1] - self.current_com_vel[1] / self.lipm_controller.omega)
        zmp_msg.z = 0.0
        self.zmp_pub.publish(zmp_msg)

        # Plan footsteps based on walking direction
        footsteps = self.footstep_planner.generate_omni_directional_steps(
            'forward', num_steps=3
        )

        # Generate joint commands based on walking pattern
        # This would involve inverse kinematics and whole-body control
        joint_commands = self.generate_joint_commands(footsteps)

        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_commands
        self.joint_cmd_pub.publish(cmd_msg)

        # Publish CoM
        com_msg = Point()
        com_msg.x = float(self.current_com[0])
        com_msg.y = float(self.current_com[1])
        com_msg.z = float(self.current_com[2])
        self.com_pub.publish(com_msg)

        # Publish walking status
        status_msg = Bool()
        status_msg.data = self.walking_active
        self.status_pub.publish(status_msg)

    def generate_joint_commands(self, footsteps):
        """Generate joint commands for walking"""
        # This would implement inverse kinematics for walking
        # For now, return placeholder values
        num_joints = 28  # Example for humanoid robot
        commands = [0.0] * num_joints

        # Add walking-specific joint angles
        # This is a simplified example - real implementation would be much more complex
        phase = self.get_clock().now().nanoseconds / 1e9  # Use time as phase reference
        hip_angle = 0.1 * np.sin(phase * 2 * np.pi / 0.8)  # 0.8s step period
        knee_angle = 0.05 * np.cos(phase * 2 * np.pi / 0.8)

        # Assign to appropriate joints (indices would depend on robot)
        commands[0] = hip_angle  # Left hip
        commands[1] = knee_angle  # Left knee
        commands[14] = -hip_angle  # Right hip (opposite phase)
        commands[15] = -knee_angle  # Right knee

        return commands

    def calculate_foot_placement(self):
        """Calculate optimal foot placement for balance"""
        # Calculate capture point
        capture_point = self.current_com[:2] + self.current_com_vel[:2] / self.lipm_controller.omega

        # Determine if step is needed for balance
        if self.left_foot_contact and self.right_foot_contact:
            # Double support - no stepping needed
            return None

        # Calculate step location based on capture point
        support_foot_pos = self.get_support_foot_position()
        step_location = self.balance_recovery.calculate_next_foot_location(
            capture_point, support_foot_pos
        )

        return step_location

    def get_support_foot_position(self):
        """Get current support foot position"""
        # This would use forward kinematics
        # For now, return a placeholder
        if self.left_foot_contact and not self.right_foot_contact:
            return np.array([0.0, 0.1])  # Left foot position
        elif self.right_foot_contact and not self.left_foot_contact:
            return np.array([0.0, -0.1])  # Right foot position
        else:
            return np.array([0.0, 0.0])  # Center position

def main(args=None):
    rclpy.init(args=args)
    walking_node = WalkingControllerNode()

    try:
        rclpy.spin(walking_node)
    except KeyboardInterrupt:
        pass
    finally:
        walking_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

Dynamic walking and balance control represent the cutting edge of humanoid locomotion research. The combination of inverted pendulum models, capture point theory, and advanced control strategies enables robots to walk with human-like stability and efficiency. The Linear Inverted Pendulum Model provides a computationally efficient framework for balance control, while preview control and model predictive control offer sophisticated approaches to handle complex walking patterns.

Balance recovery strategies ensure that robots can respond appropriately to disturbances, maintaining stability through stepping, arm swinging, or ankle strategies. The integration of these control systems with ROS 2 enables deployment in real-world applications, where robots must adapt to varying terrains and unexpected perturbations.

The next section will explore gait generation and pattern formation, building on these dynamic control foundations to create natural, efficient walking patterns.