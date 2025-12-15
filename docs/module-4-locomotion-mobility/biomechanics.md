# Biomechanics of Humanoid Locomotion

## Introduction: Understanding Human Movement

The biomechanics of locomotion form the foundation for understanding how humanoid robots can move efficiently and stably. By studying human locomotion patterns, researchers can develop control strategies that mimic the natural efficiency and adaptability of human movement. This section explores the fundamental principles of bipedal locomotion, including the mechanics of walking, running, and balance maintenance.

### The Human Locomotor System

Human locomotion is a complex interplay of:
- **Skeletal System**: Provides structural support and levers for movement
- **Muscular System**: Generates forces for movement and stability
- **Nervous System**: Controls movement patterns and maintains balance
- **Sensory Systems**: Provides feedback for control and adaptation

## Key Biomechanical Concepts

### Center of Mass (CoM) and Stability

The Center of Mass (CoM) is crucial for understanding balance and locomotion:

```python
import numpy as np
import matplotlib.pyplot as plt

class CenterOfMassAnalyzer:
    def __init__(self, robot_mass=70.0, com_height=0.85):
        self.mass = robot_mass  # kg
        self.height = com_height  # m
        self.position = np.array([0.0, 0.0, com_height])  # x, y, z
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])

    def update_com_position(self, joint_positions, joint_masses):
        """Calculate CoM position from joint positions and masses"""
        total_mass = sum(joint_masses)
        weighted_sum = np.zeros(3)

        for pos, mass in zip(joint_positions, joint_masses):
            weighted_sum += np.array(pos) * mass

        self.position = weighted_sum / total_mass if total_mass > 0 else np.array([0, 0, self.height])
        return self.position

    def calculate_com_velocity(self, new_position, dt):
        """Calculate CoM velocity from position changes"""
        if dt > 0:
            self.velocity = (new_position - self.position) / dt
        return self.velocity

    def calculate_com_acceleration(self, new_velocity, dt):
        """Calculate CoM acceleration from velocity changes"""
        if dt > 0:
            self.acceleration = (new_velocity - self.velocity) / dt
        return self.acceleration

    def is_stable(self, support_polygon):
        """Check if CoM is within support polygon for static stability"""
        if len(support_polygon) < 3:
            return False

        # Simple check for 2D projection (x, y)
        com_2d = self.position[:2]
        return self.point_in_polygon(com_2d, support_polygon)

    def point_in_polygon(self, point, polygon):
        """Check if point is inside polygon using ray casting"""
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def calculate_angular_momentum(self, joint_positions, joint_velocities, joint_masses):
        """Calculate angular momentum about CoM"""
        angular_momentum = np.zeros(3)

        for pos, vel, mass in zip(joint_positions, joint_velocities, joint_masses):
            r = np.array(pos) - self.position  # Position relative to CoM
            v = np.array(vel)
            angular_momentum += mass * np.cross(r, v)

        return angular_momentum

# Example: Analyze CoM during walking
analyzer = CenterOfMassAnalyzer()

# Simulate CoM positions during walking cycle
t = np.linspace(0, 2, 100)  # 2 seconds of walking
com_x = 0.5 * t  # Forward progression
com_y = 0.05 * np.sin(2 * np.pi * t)  # Lateral sway
com_z = 0.85 + 0.02 * np.sin(4 * np.pi * t)  # Vertical oscillation

com_positions = np.column_stack([com_x, com_y, com_z])
com_velocities = np.gradient(com_positions, axis=0) / (2/100)  # dt = 2/100 seconds
com_accelerations = np.gradient(com_velocities, axis=0) / (2/100)

print(f"CoM oscillation range: Z = {np.min(com_z):.3f} to {np.max(com_z):.3f} m")
print(f"Average CoM velocity: {np.mean(np.linalg.norm(com_velocities, axis=1)):.3f} m/s")
```

### Zero-Moment Point (ZMP) Theory

The Zero-Moment Point is fundamental to dynamic balance in bipedal locomotion:

```python
class ZMPCalculator:
    def __init__(self, com_height=0.85, gravity=9.81):
        self.com_height = com_height  # CoM height above ground
        self.gravity = gravity

    def calculate_zmp_simple(self, com_position, com_acceleration):
        """Calculate ZMP using simple inverted pendulum model"""
        # ZMP = CoM - (CoM_height / gravity) * CoM_acceleration
        zmp_x = com_position[0] - (self.com_height / self.gravity) * com_acceleration[0]
        zmp_y = com_position[1] - (self.com_height / self.gravity) * com_acceleration[1]

        return np.array([zmp_x, zmp_y, 0.0])  # ZMP is on ground plane (z=0)

    def calculate_zmp_from_forces(self, contact_forces, contact_positions):
        """Calculate ZMP from ground reaction forces and positions"""
        # ZMP = sum(Fi * pi) / sum(Fi) where Fi are vertical forces
        total_force = 0
        moment_x = 0
        moment_y = 0

        for force, position in zip(contact_forces, contact_positions):
            fz = force[2]  # Vertical force component
            total_force += fz
            moment_x += fz * position[0]  # Moment about y-axis
            moment_y += fz * position[1]  # Moment about x-axis

        if total_force != 0:
            zmp_x = moment_x / total_force
            zmp_y = moment_y / total_force
            return np.array([zmp_x, zmp_y, 0.0])
        else:
            return np.array([0.0, 0.0, 0.0])

    def is_stable_zmp(self, zmp, support_polygon, margin=0.05):
        """Check if ZMP is within support polygon with safety margin"""
        # Expand support polygon by margin
        expanded_polygon = self.expand_polygon(support_polygon, margin)
        return self.point_in_polygon_2d(zmp[:2], expanded_polygon)

    def expand_polygon(self, polygon, margin):
        """Expand polygon by margin distance"""
        # Simple expansion by moving each vertex outward
        expanded = []
        for i, vertex in enumerate(polygon):
            # Calculate average direction from centroid
            centroid = np.mean(polygon, axis=0)
            direction = np.array(vertex) - centroid
            direction = direction / np.linalg.norm(direction) if np.linalg.norm(direction) > 0 else np.array([1, 0])
            expanded_vertex = np.array(vertex) + direction * margin
            expanded.append(expanded_vertex)
        return expanded

    def point_in_polygon_2d(self, point, polygon):
        """Check if 2D point is in polygon"""
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

# Example: Calculate ZMP during walking
zmp_calc = ZMPCalculator(com_height=0.85)

# Simulate walking with CoM data
walking_time = np.linspace(0, 2, 100)
com_pos_x = 0.5 * walking_time  # Forward motion
com_pos_y = 0.05 * np.sin(2 * np.pi * walking_time)  # Lateral sway
com_pos_z = 0.85 + 0.02 * np.sin(4 * np.pi * walking_time)  # Vertical oscillation

com_positions = np.column_stack([com_pos_x, com_pos_y, com_pos_z])
com_accelerations = np.gradient(np.gradient(com_positions, axis=0), axis=0) / (2/100)**2

# Calculate ZMP for each time step
zmp_positions = []
for pos, acc in zip(com_positions, com_accelerations):
    zmp = zmp_calc.calculate_zmp_simple(pos, acc)
    zmp_positions.append(zmp)

zmp_positions = np.array(zmp_positions)

print(f"ZMP range: X = {np.min(zmp_positions[:, 0]):.3f} to {np.max(zmp_positions[:, 0]):.3f} m")
print(f"ZMP range: Y = {np.min(zmp_positions[:, 1]):.3f} to {np.max(zmp_positions[:, 1]):.3f} m")
```

### Inverted Pendulum Models

Inverted pendulum models are fundamental to understanding balance:

```python
class InvertedPendulum:
    def __init__(self, length=0.85, mass=70.0, gravity=9.81):
        self.length = length  # Length from pivot to CoM (m)
        self.mass = mass      # Mass (kg)
        self.gravity = gravity
        self.angle = 0.0      # Angle from vertical (rad)
        self.angular_velocity = 0.0

    def dynamics(self, angle, angular_velocity, control_torque=0):
        """Equations of motion for inverted pendulum"""
        # Nonlinear dynamics: theta_ddot = (g/l)*sin(theta) + torque/(m*l^2)
        angular_acceleration = (self.gravity / self.length) * np.sin(angle) + \
                              control_torque / (self.mass * self.length**2)
        return angular_acceleration

    def linearized_dynamics(self, angle, angular_velocity, control_torque=0):
        """Linearized equations of motion (valid for small angles)"""
        # For small angles: sin(theta) ≈ theta
        # theta_ddot = (g/l)*theta + torque/(m*l^2)
        angular_acceleration = (self.gravity / self.length) * angle + \
                              control_torque / (self.mass * self.length**2)
        return angular_acceleration

    def integrate(self, dt, control_torque=0):
        """Integrate the dynamics forward by dt"""
        angular_acceleration = self.dynamics(self.angle, self.angular_velocity, control_torque)

        # Update state using Euler integration
        self.angular_velocity += angular_acceleration * dt
        self.angle += self.angular_velocity * dt

        return self.angle, self.angular_velocity

    def energy(self):
        """Calculate total energy of the pendulum"""
        # Kinetic energy: KE = 0.5 * I * omega^2
        # Potential energy: PE = m * g * h
        # For pendulum: I = m * l^2, h = l * (1 - cos(theta))
        kinetic_energy = 0.5 * self.mass * self.length**2 * self.angular_velocity**2
        potential_energy = self.mass * self.gravity * self.length * (1 - np.cos(self.angle))
        return kinetic_energy + potential_energy

class LinearInvertedPendulum:
    def __init__(self, com_height=0.85, gravity=9.81):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = np.sqrt(gravity / com_height)  # Natural frequency

    def zmp_dynamics(self, zmp, com_pos, com_vel):
        """Dynamics: CoM acceleration based on ZMP position"""
        # LIPM: CoM_ddot = omega^2 * (CoM - ZMP)
        com_acc_x = self.omega**2 * (com_pos[0] - zmp[0])
        com_acc_y = self.omega**2 * (com_pos[1] - zmp[1])
        return np.array([com_acc_x, com_acc_y])

    def com_trajectory_from_zmp(self, zmp_trajectory, initial_com, initial_vel, dt):
        """Compute CoM trajectory from ZMP trajectory"""
        com_positions = [initial_com]
        com_velocities = [initial_vel]

        for zmp in zmp_trajectory:
            # Get current state
            current_com = com_positions[-1]
            current_vel = com_velocities[-1]

            # Calculate acceleration
            acc = self.zmp_dynamics(zmp, current_com, current_vel)

            # Integrate velocity
            new_vel = current_vel + acc * dt

            # Integrate position
            new_pos = current_com + new_vel * dt

            com_positions.append(new_pos)
            com_velocities.append(new_vel)

        return np.array(com_positions), np.array(com_velocities)

# Example: Simulate inverted pendulum for balance
pendulum = InvertedPendulum(length=0.85, mass=70.0)

# Simulate with small disturbance
time_points = np.linspace(0, 5, 500)
dt = time_points[1] - time_points[0]

# Initial conditions: small angle
pendulum.angle = 0.1  # 0.1 rad = ~5.7 degrees
pendulum.angular_velocity = 0.0

angles = []
angular_velocities = []
energies = []

for t in time_points:
    angle, ang_vel = pendulum.integrate(dt)
    energy = pendulum.energy()

    angles.append(angle)
    angular_velocities.append(ang_vel)
    energies.append(energy)

print(f"Initial energy: {energies[0]:.3f} J")
print(f"Final energy: {energies[-1]:.3f} J")
print(f"Max angle: {np.max(np.abs(angles)):.3f} rad ({np.max(np.abs(angles))*180/np.pi:.1f}°)")
```

## Human Walking Biomechanics

### Gait Cycle Analysis

The human gait cycle has distinct phases that inform robotic walking:

```python
class GaitCycleAnalyzer:
    def __init__(self):
        self.stance_phase = 0.6  # 60% of gait cycle is stance
        self.swing_phase = 0.4   # 40% of gait cycle is swing
        self.step_length = 0.7   # Average step length (m)
        self.step_width = 0.1    # Distance between feet (m)
        self.stride_time = 1.0   # Time for one complete step cycle (s)

    def gait_phase(self, time, leg='right'):
        """Determine gait phase based on time"""
        # Normalize time to gait cycle
        cycle_time = time % (2 * self.stride_time)  # Two steps per cycle (right-left)

        if leg == 'right':
            if cycle_time < self.stride_time:
                # Right leg stance phase
                if cycle_time / self.stride_time < self.stance_phase:
                    return 'stance'
                else:
                    return 'swing'
            else:
                # Right leg swing phase (left leg is in stance)
                return 'swing'
        else:  # left leg
            if cycle_time < self.stride_time:
                # Left leg swing phase (right leg is in stance)
                return 'swing'
            else:
                # Left leg stance phase
                if (cycle_time - self.stride_time) / self.stride_time < self.stance_phase:
                    return 'stance'
                else:
                    return 'swing'

    def foot_position(self, time, leg='right'):
        """Calculate foot position during gait cycle"""
        # This is a simplified model
        cycle_time = time % (2 * self.stride_time)

        if leg == 'right':
            if cycle_time < self.stride_time:
                # Right foot in stance, left foot in swing
                # Right foot position (relative to body center)
                x = -self.step_length / 2
                y = -self.step_width / 2 if cycle_time < self.stride_time / 2 else self.step_width / 2
            else:
                # Left foot in stance, right foot in swing
                # Right foot position (moving forward)
                swing_progress = (cycle_time - self.stride_time) / (self.stride_time * self.swing_phase)
                x = -self.step_length / 2 + self.step_length * swing_progress
                y = self.step_width / 2
        else:  # left leg
            if cycle_time < self.stride_time:
                # Left foot in swing, right foot in stance
                # Left foot position (moving forward)
                swing_progress = cycle_time / (self.stride_time * self.swing_phase)
                x = -self.step_length / 2 + self.step_length * swing_progress
                y = -self.step_width / 2
            else:
                # Left foot in stance
                x = -self.step_length / 2
                y = self.step_width / 2 if cycle_time < 1.5 * self.stride_time else -self.step_width / 2

        return np.array([x, y, 0.0])

    def calculate_gait_parameters(self, walking_speed):
        """Calculate gait parameters based on walking speed"""
        # Empirical relationships for human gait
        stride_length = 0.45 + 0.4 * walking_speed  # Approximate relationship
        stride_time = stride_length / walking_speed if walking_speed > 0 else 1.0
        step_frequency = 1 / stride_time

        # Duty factor (stance time / stride time)
        duty_factor = 0.6 if walking_speed < 1.0 else 0.4  # Humans change duty factor with speed

        return {
            'stride_length': stride_length,
            'stride_time': stride_time,
            'step_frequency': step_frequency,
            'duty_factor': duty_factor,
            'step_length': stride_length / 2
        }

    def generate_footprint_pattern(self, duration, walking_speed=1.0):
        """Generate footprint pattern over time"""
        gait_params = self.calculate_gait_parameters(walking_speed)
        time_steps = np.arange(0, duration, 0.01)

        right_footprints = []
        left_footprints = []

        for t in time_steps:
            # Determine if foot should be placed
            right_phase = self.gait_phase(t, 'right')
            left_phase = self.gait_phase(t, 'left')

            # Simplified footprint placement logic
            if t > 0 and int(t / gait_params['stride_time']) != int((t-0.01) / gait_params['stride_time']):
                # New stride cycle started
                cycle_num = int(t / gait_params['stride_time'])
                forward_offset = cycle_num * gait_params['stride_length'] / 2

                if cycle_num % 2 == 0:  # Right foot contact
                    right_footprints.append([forward_offset, -self.step_width/2, t])
                else:  # Left foot contact
                    left_footprints.append([forward_offset, self.step_width/2, t])

        return np.array(right_footprints), np.array(left_footprints)

# Example: Analyze gait patterns
gait_analyzer = GaitCycleAnalyzer()

# Calculate gait parameters for different speeds
speeds = [0.5, 1.0, 1.5]
for speed in speeds:
    params = gait_analyzer.calculate_gait_parameters(speed)
    print(f"Speed {speed} m/s: Stride length = {params['stride_length']:.2f} m, "
          f"Frequency = {params['step_frequency']:.2f} Hz")

# Generate footprint pattern
right_fp, left_fp = gait_analyzer.generate_footprint_pattern(4.0, walking_speed=1.0)
print(f"Generated {len(right_fp)} right footprints and {len(left_fp)} left footprints")
```

### Joint Kinematics During Walking

Understanding joint movements during the gait cycle:

```python
class JointKinematicsAnalyzer:
    def __init__(self):
        # Typical ranges of motion during walking (in degrees)
        self.hip_rom = {'flexion': 30, 'extension': 10, 'abduction': 10, 'adduction': 5}
        self.knee_rom = {'flexion': 65, 'extension': 5}
        self.ankle_rom = {'dorsiflexion': 15, 'plantarflexion': 20}

    def hip_angle_trajectory(self, phase, leg='right'):
        """Generate hip angle trajectory during gait cycle"""
        # Simplified hip angle patterns during walking
        if phase == 'stance':
            # Hip flexion/extension during stance phase
            # Maximum flexion at initial contact, extension during push-off
            hip_flexion = 10 * np.sin(2 * np.pi * phase) if hasattr(phase, '__len__') else 10  # Simplified
        else:  # swing
            # Hip flexion increases during swing to clear the foot
            hip_flexion = 20  # Average swing phase flexion

        return np.radians(hip_flexion)

    def knee_angle_trajectory(self, gait_phase, swing_progress=0.0):
        """Generate knee angle trajectory"""
        if gait_phase == 'stance':
            # Knee starts slightly flexed, extends mid-stance, flexes for push-off
            knee_angle = 10  # Degrees flexion during mid-stance
        else:  # swing
            # Knee flexes during early swing for foot clearance, extends for landing
            if swing_progress < 0.3:  # Early swing
                knee_angle = 60  # Maximum flexion
            elif swing_progress < 0.7:  # Mid swing
                knee_angle = 40
            else:  # Late swing
                knee_angle = 10  # Prepare for landing

        return np.radians(knee_angle)

    def ankle_angle_trajectory(self, gait_phase, stance_progress=0.0):
        """Generate ankle angle trajectory"""
        if gait_phase == 'stance':
            # Ankle motion during stance: dorsiflexion after heel strike, plantarflexion at push-off
            if stance_progress < 0.2:  # After heel strike
                ankle_angle = -10  # Dorsiflexion
            elif stance_progress < 0.6:  # Mid-stance
                ankle_angle = 0   # Neutral
            else:  # Push-off
                ankle_angle = 15  # Plantarflexion
        else:  # swing
            # Ankle remains slightly dorsiflexed during swing for foot clearance
            ankle_angle = -5

        return np.radians(ankle_angle)

    def generate_full_leg_trajectory(self, time_points):
        """Generate complete leg kinematics for walking"""
        trajectories = {
            'right_hip_flexion': [],
            'right_knee_flexion': [],
            'right_ankle_angle': [],
            'left_hip_flexion': [],
            'left_knee_flexion': [],
            'left_ankle_angle': []
        }

        gait_analyzer = GaitCycleAnalyzer()

        for t in time_points:
            # Right leg
            right_phase = gait_analyzer.gait_phase(t, 'right')
            right_foot_pos = gait_analyzer.foot_position(t, 'right')

            # Left leg
            left_phase = gait_analyzer.gait_phase(t, 'left')
            left_foot_pos = gait_analyzer.foot_position(t, 'left')

            # Calculate joint angles based on phase
            # This is a simplified representation
            trajectories['right_hip_flexion'].append(self.hip_angle_trajectory(right_phase))
            trajectories['right_knee_flexion'].append(self.knee_angle_trajectory(right_phase))
            trajectories['right_ankle_angle'].append(self.ankle_angle_trajectory(right_phase))

            trajectories['left_hip_flexion'].append(self.hip_angle_trajectory(left_phase))
            trajectories['left_knee_flexion'].append(self.knee_angle_trajectory(left_phase))
            trajectories['left_ankle_angle'].append(self.ankle_angle_trajectory(left_phase))

        return trajectories

# Example: Generate joint trajectories
kin_analyzer = JointKinematicsAnalyzer()
time_points = np.linspace(0, 4, 400)  # 4 seconds of walking
joint_trajectories = kin_analyzer.generate_full_leg_trajectory(time_points)

print(f"Generated trajectories for {len(time_points)} time steps")
print(f"Right hip flexion range: {np.degrees(min(joint_trajectories['right_hip_flexion'])):.1f}° "
      f"to {np.degrees(max(joint_trajectories['right_hip_flexion'])):.1f}°")
```

## Anthropomorphic Design Principles

### Proportional Relationships

Human-like proportions are important for natural movement:

```python
class AnthropomorphicDesign:
    def __init__(self, height=1.7):
        self.height = height
        self.mass = 70.0  # Default mass in kg
        self.segment_ratios = self.calculate_segment_ratios()
        self.joint_constraints = self.define_joint_constraints()

    def calculate_segment_ratios(self):
        """Calculate body segment ratios based on anthropometric data"""
        # Standard anthropometric ratios (fraction of total height)
        ratios = {
            'head': 0.152,  # Head height
            'trunk': 0.520,  # Trunk height
            'upper_leg': 0.245,  # Upper leg (thigh)
            'lower_leg': 0.240,  # Lower leg (shank)
            'foot': 0.152,  # Foot length
            'upper_arm': 0.186,  # Upper arm
            'lower_arm': 0.146   # Lower arm
        }

        # Convert to actual lengths
        lengths = {key: value * self.height for key, value in ratios.items()}
        return lengths

    def define_joint_constraints(self):
        """Define physiologically realistic joint limits"""
        constraints = {
            'hip': {
                'flexion': np.radians(120),
                'extension': np.radians(-30),
                'abduction': np.radians(45),
                'adduction': np.radians(-30),
                'internal_rotation': np.radians(45),
                'external_rotation': np.radians(-45)
            },
            'knee': {
                'flexion': np.radians(150),
                'extension': np.radians(0),
                'rotation': np.radians(10)  # Limited in extended position
            },
            'ankle': {
                'dorsiflexion': np.radians(20),
                'plantarflexion': np.radians(50),
                'inversion': np.radians(35),
                'eversion': np.radians(-35)
            },
            'shoulder': {
                'flexion': np.radians(180),
                'extension': np.radians(-60),
                'abduction': np.radians(180),
                'adduction': np.radians(-45),
                'internal_rotation': np.radians(90),
                'external_rotation': np.radians(-90)
            }
        }
        return constraints

    def calculate_mass_distribution(self):
        """Calculate mass distribution based on anthropometric data"""
        # Standard anthropometric mass distribution percentages
        mass_percentages = {
            'head': 0.073,
            'trunk': 0.507,
            'thigh': 0.142,  # Each leg
            'shank': 0.043,  # Each leg
            'foot': 0.014,   # Each foot
            'upper_arm': 0.028,  # Each arm
            'forearm': 0.016,    # Each forearm
            'hand': 0.007      # Each hand
        }

        # Calculate actual masses
        masses = {key: value * self.mass for key, value in mass_percentages.items()}
        return masses

    def calculate_com_position(self, joint_angles, joint_positions):
        """Calculate whole-body CoM based on joint configuration"""
        # This would require full kinematic model and mass distribution
        # For now, provide a simplified calculation
        segment_masses = self.calculate_mass_distribution()
        segment_lengths = self.segment_ratios

        # Simplified CoM calculation (would need full forward kinematics in practice)
        total_mass = sum(segment_masses.values())
        com = np.zeros(3)

        # This is a placeholder - full implementation would require
        # forward kinematics for each body segment
        return com

    def get_recommended_actuator_specs(self):
        """Get recommended actuator specifications based on anthropomorphic design"""
        # Based on human muscle forces and joint torques
        actuator_specs = {
            'hip': {
                'max_torque': 300,  # Nm (approximate human hip torque)
                'max_speed': 5.0,   # rad/s
                'power': 1500       # W
            },
            'knee': {
                'max_torque': 150,  # Nm
                'max_speed': 6.0,   # rad/s
                'power': 900        # W
            },
            'ankle': {
                'max_torque': 100,  # Nm
                'max_speed': 8.0,   # rad/s
                'power': 800        # W
            },
            'shoulder': {
                'max_torque': 80,   # Nm
                'max_speed': 10.0,  # rad/s
                'power': 800        # W
            }
        }
        return actuator_specs

# Example: Design anthropomorphic robot
anthro_design = AnthropomorphicDesign(height=1.7)

print(f"Segment lengths for {anthro_design.height}m tall robot:")
for segment, length in anthro_design.segment_ratios.items():
    print(f"  {segment}: {length:.3f} m")

print(f"\nMass distribution:")
segment_masses = anthro_design.calculate_mass_distribution()
for segment, mass in segment_masses.items():
    print(f"  {segment}: {mass:.2f} kg")

print(f"\nRecommended actuator specifications:")
actuator_specs = anthro_design.get_recommended_actuator_specs()
for joint, specs in actuator_specs.items():
    print(f"  {joint}: {specs['max_torque']} Nm, {specs['max_speed']} rad/s, {specs['power']} W")
```

## Locomotion Energetics

### Mechanical Work and Energy

Understanding the energy requirements of locomotion:

```python
class LocomotionEnergetics:
    def __init__(self, robot_mass=70.0):
        self.mass = robot_mass
        self.gravity = 9.81

    def step_to_step_transition_energy(self, step_length, step_height):
        """Calculate energy for step-to-step transition (collisional losses)"""
        # Energy lost during step transition due to impulsive forces
        # This is an approximation based on compass gait model
        energy_loss = 0.5 * self.mass * self.gravity * step_height
        return energy_loss

    def center_of_mass_work(self, com_trajectory):
        """Calculate mechanical work done on CoM"""
        work = 0.0
        for i in range(1, len(com_trajectory)):
            # Calculate velocity change
            v1 = (com_trajectory[i] - com_trajectory[i-1]) / 0.01  # dt = 0.01s
            if i > 1:
                v0 = (com_trajectory[i-1] - com_trajectory[i-2]) / 0.01
                delta_v = v1 - v0
                # Work = F * d = m * a * d = m * (dv/dt) * d
                work += self.mass * np.linalg.norm(delta_v) * np.linalg.norm(com_trajectory[i] - com_trajectory[i-1])
        return work

    def calculate_metabolic_cost(self, walking_speed, step_frequency):
        """Calculate approximate metabolic cost of transport"""
        # Based on human locomotion data
        # Cost of transport (COT) = metabolic energy / (body_weight * distance)
        # For humans: COT ≈ 1.0 J/(kg*m) at optimal speed

        # Speed-dependent COT with optimal around 1.3 m/s
        optimal_speed = 1.3
        speed_factor = 1.0 + 0.5 * ((walking_speed - optimal_speed) / optimal_speed)**2

        # Calculate COT
        cot = 1.0 * speed_factor  # J/(kg*m)

        # Total metabolic energy for distance
        distance = walking_speed * 10  # Energy for 10 seconds of walking
        metabolic_energy = cot * self.mass * self.gravity * distance

        return metabolic_energy, cot

    def pendular_energy_exchange(self, com_height_trajectory):
        """Calculate pendular energy exchange during walking"""
        # During walking, potential and kinetic energy are exchanged
        # This reduces the mechanical work required

        potential_energy = self.mass * self.gravity * com_height_trajectory
        # Kinetic energy would require velocity information
        # This is a simplified analysis

        # Calculate energy fluctuations
        pe_max = np.max(potential_energy)
        pe_min = np.min(potential_energy)
        pe_fluctuation = pe_max - pe_min

        # Energy recovery ratio (how much energy is exchanged vs. replaced)
        # In human walking: ~60-70% energy recovery
        energy_recovery_ratio = 0.65  # Typical human value

        return pe_fluctuation, energy_recovery_ratio

    def compare_locomotion_modes(self, distance=100):
        """Compare energy costs of different locomotion modes"""
        modes = {
            'walking': {'speed': 1.3, 'cost_per_meter': 1.0},  # J/kg/m
            'running': {'speed': 3.0, 'cost_per_meter': 3.0},  # Higher cost
            'crawling': {'speed': 0.5, 'cost_per_meter': 5.0}, # Much higher cost
            'wheelchair': {'speed': 1.0, 'cost_per_meter': 0.5} # Lower cost, but not bipedal
        }

        results = {}
        for mode, params in modes.items():
            time = distance / params['speed']
            energy = params['cost_per_meter'] * self.mass * distance
            power = energy / time

            results[mode] = {
                'time': time,
                'energy': energy,
                'power': power,
                'speed': params['speed']
            }

        return results

# Example: Analyze locomotion energetics
energetics = LocomotionEnergetics(robot_mass=70.0)

# Calculate metabolic cost for different speeds
speeds = [0.5, 1.0, 1.3, 1.5, 2.0]
print("Metabolic cost analysis:")
for speed in speeds:
    energy, cot = energetics.calculate_metabolic_cost(speed, 1.0)
    print(f"Speed {speed} m/s: COT = {cot:.2f} J/(kg*m), Energy for 10s = {energy:.1f} J")

# Compare locomotion modes
mode_comparison = energetics.compare_locomotion_modes(distance=100)
print(f"\nEnergy comparison for 100m travel:")
for mode, data in mode_comparison.items():
    print(f"{mode:12s}: {data['energy']:6.0f} J, {data['time']:5.1f}s, {data['power']:5.1f}W")
```

## Conclusion

The biomechanics of humanoid locomotion provide the fundamental understanding needed to design and control bipedal robots. By studying human movement patterns, researchers can develop more natural, efficient, and stable walking algorithms. The key concepts of Center of Mass control, Zero-Moment Point theory, and inverted pendulum dynamics form the foundation for dynamic balance in humanoid robots.

The anthropomorphic design principles ensure that robots have proportions and capabilities similar to humans, enabling them to navigate human environments effectively. Understanding the energetics of locomotion helps in designing energy-efficient systems that can operate for extended periods.

The next section will explore dynamic walking and balance control strategies that implement these biomechanical principles in real robotic systems.