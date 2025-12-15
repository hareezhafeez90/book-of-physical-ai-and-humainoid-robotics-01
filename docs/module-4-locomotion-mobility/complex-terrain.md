# Stair Climbing and Complex Terrain Navigation

## Introduction: Beyond Flat Ground Locomotion

Complex terrain navigation represents one of the most challenging aspects of humanoid robotics, requiring robots to handle environments far beyond simple flat surfaces. This includes stairs, narrow passages, dynamic obstacles, slippery surfaces, and other challenging scenarios that require sophisticated perception, planning, and control capabilities. This section explores the specialized techniques and algorithms needed for navigating these complex environments.

### Challenges of Complex Terrain Navigation

Complex terrain navigation involves multiple interconnected challenges:
- **Multi-modal locomotion**: Switching between different locomotion modes
- **Dynamic obstacle avoidance**: Handling moving obstacles and people
- **Precision positioning**: Accurate foot placement on small targets
- **Stability maintenance**: Preserving balance in challenging conditions
- **Real-time adaptation**: Responding to unexpected terrain features

## Stair Climbing and Negotiation

### Stair Detection and Classification

Robust stair detection is fundamental for successful stair climbing:

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from sklearn.linear_model import RANSACRegressor

class StairDetector:
    def __init__(self):
        self.stair_params = {
            'typical_rise': 0.17,    # 17cm typical stair rise
            'typical_run': 0.3,     # 30cm typical stair run
            'max_rise': 0.25,       # Maximum safe rise
            'min_rise': 0.10,       # Minimum rise to be considered a step
            'max_run': 0.5,         # Maximum safe run
            'min_run': 0.2,         # Minimum safe run
            'tolerance': 0.02       # 2cm tolerance
        }
        self.detected_stairs = []
        self.stair_confidence = 0.0

    def detect_stairs_from_height_profile(self, x_coords, height_values):
        """Detect stairs from 1D height profile along walking direction"""
        # Find potential step locations using height changes
        height_diffs = np.diff(height_values)

        # Look for consistent height changes that indicate stairs
        potential_steps = []
        for i in range(len(height_diffs)):
            if abs(height_diffs[i]) > self.stair_params['min_rise']:
                potential_steps.append((i, height_diffs[i], height_values[i]))

        if len(potential_steps) < 2:
            return []

        # Group steps by consistent height (stair rise)
        step_groups = []
        current_group = []

        for i in range(len(potential_steps)):
            if i == 0:
                current_group.append(potential_steps[i])
                continue

            prev_height = potential_steps[i-1][2]
            curr_height = potential_steps[i][2]
            height_change = curr_height - prev_height

            # Check if this continues the stair pattern
            if (abs(height_change - self.stair_params['typical_rise']) <
                self.stair_params['tolerance']):
                current_group.append(potential_steps[i])
            else:
                if len(current_group) >= 2:  # Need at least 2 steps to be stairs
                    step_groups.append(current_group.copy())
                current_group = [potential_steps[i]]

        if len(current_group) >= 2:
            step_groups.append(current_group)

        # Validate and classify stair groups
        valid_stairs = []
        for group in step_groups:
            if self.validate_stair_group(group):
                stair_info = self.extract_stair_info(group, x_coords)
                valid_stairs.append(stair_info)

        self.detected_stairs = valid_stairs
        self.stair_confidence = len(valid_stairs) / max(1, len(step_groups))

        return valid_stairs

    def validate_stair_group(self, step_group):
        """Validate if a group of potential steps forms valid stairs"""
        if len(step_group) < 2:
            return False

        # Check for consistent rise
        rises = []
        for i in range(1, len(step_group)):
            rise = step_group[i][1]  # height_diff from the tuple
            rises.append(abs(rise))

        if not rises:
            return False

        avg_rise = np.mean(rises)
        std_rise = np.std(rises)

        # Stairs should have consistent rise
        if std_rise > 0.02:  # Too much variation
            return False

        # Rise should be within reasonable bounds
        if not (self.stair_params['min_rise'] <= avg_rise <= self.stair_params['max_rise']):
            return False

        # Check for consistent run (horizontal distance between steps)
        if len(step_group) > 1:
            runs = []
            for i in range(1, len(step_group)):
                run = step_group[i][0] - step_group[i-1][0]  # x coordinate difference
                runs.append(run)

            if runs:
                avg_run = np.mean(runs) * 0.05  # Convert from index to meters (assuming 5cm resolution)
                if not (self.stair_params['min_run'] <= avg_run <= self.stair_params['max_run']):
                    return False

        return True

    def extract_stair_info(self, step_group, x_coords):
        """Extract detailed stair information from validated group"""
        rises = [abs(step[1]) for step in step_group]
        avg_rise = np.mean(rises)

        # Calculate stair dimensions
        start_x = x_coords[step_group[0][0]]
        end_x = x_coords[step_group[-1][0]]
        num_steps = len(step_group)

        # Calculate run (horizontal distance per step)
        total_run = end_x - start_x
        avg_run = total_run / num_steps if num_steps > 1 else self.stair_params['typical_run']

        # Calculate heights of each step
        step_heights = []
        current_height = step_group[0][2]  # starting height
        for i, step in enumerate(step_group):
            height = current_height + sum(rises[:i+1])
            step_heights.append(height)

        stair_info = {
            'start_x': start_x,
            'end_x': end_x,
            'num_steps': num_steps,
            'avg_rise': avg_rise,
            'avg_run': avg_run,
            'step_heights': step_heights,
            'total_height': step_heights[-1] - step_heights[0],
            'type': 'ascending' if rises[0] > 0 else 'descending',
            'confidence': min(1.0, len(step_group) / 10.0)  # More steps = higher confidence
        }

        return stair_info

    def detect_stairs_from_3d_point_cloud(self, point_cloud):
        """Detect stairs from 3D point cloud data"""
        # Project 3D points to 2D profile along walking direction
        # For simplicity, assume walking along X-axis

        # Sort points by X coordinate
        sorted_indices = np.argsort(point_cloud[:, 0])
        sorted_points = point_cloud[sorted_indices]

        # Create height profile by sampling points along X
        x_bins = np.arange(sorted_points[0, 0], sorted_points[-1, 0], 0.05)  # 5cm bins
        height_profile = []

        for x_bin in x_bins:
            # Get points in this bin
            bin_mask = (sorted_points[:, 0] >= x_bin) & (sorted_points[:, 0] < x_bin + 0.05)
            bin_points = sorted_points[bin_mask]

            if len(bin_points) > 0:
                # Use median height to reduce noise
                avg_height = np.median(bin_points[:, 2])
                height_profile.append(avg_height)
            else:
                height_profile.append(np.nan)

        # Remove NaN values
        valid_indices = ~np.isnan(height_profile)
        valid_x = x_bins[valid_indices]
        valid_heights = np.array(height_profile)[valid_indices]

        if len(valid_heights) > 10:  # Need sufficient points
            stairs = self.detect_stairs_from_height_profile(valid_x, valid_heights)
            return stairs

        return []

    def classify_stair_type(self, stair_info):
        """Classify stair type based on dimensions"""
        if stair_info['avg_rise'] < 0.15:
            return 'low_step'
        elif stair_info['avg_rise'] < 0.20:
            return 'standard_stair'
        elif stair_info['avg_rise'] < 0.25:
            return 'high_step'
        else:
            return 'ramp'  # If rise is too high, might be a ramp

# Example: Stair detection
stair_detector = StairDetector()

# Create sample stair point cloud
stair_points = []
for step in range(6):  # 6 steps
    z_height = step * 0.17  # 17cm rise per step
    for x in np.arange(step * 0.3, step * 0.3 + 0.3, 0.02):  # 30cm run
        for y in np.arange(-0.15, 0.15, 0.02):  # 30cm width
            stair_points.append([x, y, z_height])

# Add some noise to simulate real sensor data
stair_points = np.array(stair_points)
stair_points[:, 2] += np.random.normal(0, 0.01, len(stair_points))  # 1cm height noise

detected_stairs = stair_detector.detect_stairs_from_3d_point_cloud(stair_points)
print(f"Detected {len(detected_stairs)} stair sequences")

for i, stair in enumerate(detected_stairs):
    print(f"Stair {i+1}: {stair['num_steps']} steps, "
          f"rise: {stair['avg_rise']:.3f}m, "
          f"run: {stair['avg_run']:.3f}m, "
          f"type: {stair_detector.classify_stair_type(stair)}")
```

### Stair Climbing Gait Generation

Specialized gait patterns for stair climbing:

```python
class StairClimbingGaitGenerator:
    def __init__(self):
        self.stair_gait_params = {
            'approach_distance': 0.5,      # Distance before stairs to prepare
            'step_up_height': 0.05,        # Additional lift for step up
            'step_down_height': 0.02,      # Additional clearance for step down
            'step_time': 1.0,              # Time per step (slower than normal walking)
            'com_height_offset': 0.02,     # Slightly raise CoM during stair climbing
            'ankle_compensation': 0.1      # Ankle angle compensation for level feet
        }

    def generate_stair_ascent_gait(self, stair_info, robot_com_height=0.85):
        """Generate gait pattern for stair ascent"""
        gait_pattern = {
            'left_foot': [],
            'right_foot': [],
            'com_trajectory': [],
            'timestamps': [],
            'phase_descriptions': []
        }

        num_steps = stair_info['num_steps']
        avg_run = stair_info['avg_run']
        avg_rise = stair_info['avg_rise']

        # Generate trajectory for each step
        for step_idx in range(num_steps):
            # Calculate positions for this step
            step_x = stair_info['start_x'] + step_idx * avg_run
            step_z = stair_info['step_heights'][step_idx]

            # Left foot placement (for ascending stairs)
            left_foot_x = step_x
            left_foot_y = 0.1  # Left foot offset
            left_foot_z = step_z

            # Right foot placement (on same step, but alternates)
            right_foot_x = step_x
            right_foot_y = -0.1  # Right foot offset
            right_foot_z = step_z

            # Add to trajectories
            gait_pattern['left_foot'].append([left_foot_x, left_foot_y, left_foot_z])
            gait_pattern['right_foot'].append([right_foot_x, right_foot_y, right_foot_z])

            # CoM trajectory - follow the rising stairs
            com_x = step_x + avg_run / 2  # CoM leads feet slightly
            com_y = 0.0  # Center between feet
            com_z = robot_com_height + stair_info['avg_rise'] * step_idx * 0.1  # Gradual rise
            gait_pattern['com_trajectory'].append([com_x, com_y, com_z])

            # Timestamps
            gait_pattern['timestamps'].append(step_idx * self.stair_gait_params['step_time'])

            # Phase descriptions
            gait_pattern['phase_descriptions'].append(f"Step {step_idx + 1} ascent")

        return gait_pattern

    def generate_stair_descent_gait(self, stair_info, robot_com_height=0.85):
        """Generate gait pattern for stair descent"""
        gait_pattern = {
            'left_foot': [],
            'right_foot': [],
            'com_trajectory': [],
            'timestamps': [],
            'phase_descriptions': []
        }

        num_steps = stair_info['num_steps']
        avg_run = stair_info['avg_run']
        avg_rise = stair_info['avg_rise']

        # For descent, we approach from the top and go down
        start_x = stair_info['end_x']
        start_z = stair_info['step_heights'][-1]

        for step_idx in range(num_steps):
            # Calculate positions for descending step
            step_x = start_x - step_idx * avg_run
            step_z = start_z - step_idx * avg_rise

            # Left foot placement
            left_foot_x = step_x
            left_foot_y = 0.1
            left_foot_z = step_z

            # Right foot placement
            right_foot_x = step_x
            right_foot_y = -0.1
            right_foot_z = step_z

            # Add to trajectories
            gait_pattern['left_foot'].append([left_foot_x, left_foot_y, left_foot_z])
            gait_pattern['right_foot'].append([right_foot_x, right_foot_y, right_foot_z])

            # CoM trajectory - follow the descending stairs
            com_x = step_x - avg_run / 2  # CoM follows feet
            com_y = 0.0
            com_z = robot_com_height - avg_rise * step_idx * 0.1  # Gradual descent
            gait_pattern['com_trajectory'].append([com_x, com_y, com_z])

            # Timestamps
            gait_pattern['timestamps'].append(step_idx * self.stair_gait_params['step_time'])

            # Phase descriptions
            gait_pattern['phase_descriptions'].append(f"Step {step_idx + 1} descent")

        return gait_pattern

    def generate_approach_pattern(self, stair_start_x, stair_start_z, approach_steps=3):
        """Generate approach pattern before reaching stairs"""
        approach_pattern = {
            'left_foot': [],
            'right_foot': [],
            'com_trajectory': [],
            'timestamps': [],
            'phase_descriptions': []
        }

        for i in range(approach_steps):
            # Normal walking pattern approaching stairs
            x_pos = stair_start_x - (approach_steps - i) * 0.3  # 30cm steps
            y_offset = 0.1 if i % 2 == 0 else -0.1  # Alternate feet

            approach_pattern['left_foot'].append([x_pos, 0.1, stair_start_z]) if i % 2 == 0 else None
            approach_pattern['right_foot'].append([x_pos, -0.1, stair_start_z]) if i % 2 == 1 else None

            # CoM trajectory
            approach_pattern['com_trajectory'].append([x_pos, 0.0, 0.85])

            # Timestamps and descriptions
            approach_pattern['timestamps'].append(i * 0.8)
            approach_pattern['phase_descriptions'].append(f"Approach step {i + 1}")

        return approach_pattern

    def generate_departure_pattern(self, stair_end_x, stair_end_z, departure_steps=3):
        """Generate departure pattern after leaving stairs"""
        departure_pattern = {
            'left_foot': [],
            'right_foot': [],
            'com_trajectory': [],
            'timestamps': [],
            'phase_descriptions': []
        }

        for i in range(departure_steps):
            # Normal walking pattern after stairs
            x_pos = stair_end_x + (i + 1) * 0.3
            y_offset = 0.1 if i % 2 == 0 else -0.1

            departure_pattern['left_foot'].append([x_pos, 0.1, stair_end_z]) if i % 2 == 0 else None
            departure_pattern['right_foot'].append([x_pos, -0.1, stair_end_z]) if i % 2 == 1 else None

            # CoM trajectory
            departure_pattern['com_trajectory'].append([x_pos, 0.0, 0.85])

            # Timestamps and descriptions
            departure_pattern['timestamps'].append(i * 0.8)
            departure_pattern['phase_descriptions'].append(f"Departure step {i + 1}")

        return departure_pattern

    def generate_complete_stair_trajectory(self, stair_info, robot_com_height=0.85):
        """Generate complete trajectory including approach, climbing, and departure"""
        complete_trajectory = {
            'approach': self.generate_approach_pattern(
                stair_info['start_x'], stair_info['step_heights'][0]
            ),
            'climbing': self.generate_stair_ascent_gait(stair_info, robot_com_height),
            'departure': self.generate_departure_pattern(
                stair_info['end_x'], stair_info['step_heights'][-1]
            )
        }

        return complete_trajectory

# Example: Stair climbing gait generation
stair_gait_gen = StairClimbingGaitGenerator()

# Use the stair info from previous detection
if detected_stairs:
    stair_info = detected_stairs[0]
    complete_trajectory = stair_gait_gen.generate_complete_stair_trajectory(stair_info)

    print(f"Generated complete stair trajectory:")
    print(f"  Approach: {len(complete_trajectory['approach']['timestamps'])} steps")
    print(f"  Climbing: {len(complete_trajectory['climbing']['timestamps'])} steps")
    print(f"  Departure: {len(complete_trajectory['departure']['timestamps'])} steps")

    # Display some key trajectory points
    climbing = complete_trajectory['climbing']
    if climbing['left_foot']:
        print(f"  First step left foot: {climbing['left_foot'][0]}")
        print(f"  Last step left foot: {climbing['left_foot'][-1]}")
        print(f"  CoM trajectory range: Z from {climbing['com_trajectory'][0][2]:.3f} to {climbing['com_trajectory'][-1][2]:.3f}")
```

### Stair Negotiation Control Strategies

Advanced control strategies for stable stair negotiation:

```python
class StairNegotiationController:
    def __init__(self):
        self.control_params = {
            'ankle_stiffness': 200,        # Nm/rad ankle stiffness
            'hip_compensation_gain': 50,   # Hip position compensation
            'com_damping': 10,             # CoM velocity damping
            'foot_lift_height': 0.05,      # Foot lift height for step clearance
            'contact_threshold': 5.0       # Force threshold for contact detection
        }
        self.stance_foot = 'left'  # Current stance foot
        self.swing_foot = 'right'  # Current swing foot

    def calculate_ankle_compensation(self, terrain_normal, foot_type='left'):
        """Calculate ankle compensation angles based on terrain normal"""
        # terrain_normal should be [nx, ny, nz] where nz is upward component
        if np.linalg.norm(terrain_normal) == 0:
            return 0.0, 0.0  # No slope

        # Normalize the normal vector
        normal = terrain_normal / np.linalg.norm(terrain_normal)

        # Calculate required roll and pitch angles
        # For a surface normal [nx, ny, nz], the required angles are:
        # roll ≈ arctan(ny/nz)
        # pitch ≈ arctan(-nx/nz)
        roll_angle = np.arctan2(normal[1], normal[2])
        pitch_angle = np.arctan2(-normal[0], normal[2])

        # Apply compensation gains
        max_compensation = 0.2  # Limit to ~11 degrees
        roll_angle = np.clip(roll_angle, -max_compensation, max_compensation)
        pitch_angle = np.clip(pitch_angle, -max_compensation, max_compensation)

        return roll_angle, pitch_angle

    def generate_foot_lift_trajectory(self, start_pos, target_pos, step_type='up'):
        """Generate smooth foot lift trajectory for step clearance"""
        # Create intermediate points for smooth lifting
        lift_height = (self.control_params['foot_lift_height'] if step_type == 'up'
                      else self.control_params['foot_lift_height'] / 2)  # Less lift for step down

        # Define trajectory points
        mid_x = (start_pos[0] + target_pos[0]) / 2
        mid_y = (start_pos[1] + target_pos[1]) / 2
        mid_z = max(start_pos[2], target_pos[2]) + lift_height

        # Simple 3-point trajectory: lift, move, place
        trajectory_points = [
            start_pos,  # Start position
            [mid_x, mid_y, mid_z],  # Lift point
            target_pos  # Target position
        ]

        # Interpolate for smoother motion
        smooth_trajectory = []
        for i in range(len(trajectory_points) - 1):
            start_point = trajectory_points[i]
            end_point = trajectory_points[i + 1]

            # Linear interpolation with smooth transition
            for t in np.linspace(0, 1, 10):  # 10 points per segment
                # Use cubic interpolation for smoother motion
                t_smooth = 3*t**2 - 2*t**3
                pos = [(1-t_smooth)*s + t_smooth*e for s, e in zip(start_point, end_point)]
                smooth_trajectory.append(pos)

        return smooth_trajectory

    def calculate_balance_compensation(self, com_error, zmp_error, step_phase):
        """Calculate balance compensation based on errors and step phase"""
        compensation = {
            'hip_offset': np.zeros(3),  # [x, y, z] offset
            'ankle_torque': np.zeros(2),  # [roll, pitch] torque
            'com_adjustment': np.zeros(3)  # CoM adjustment
        }

        # Hip compensation for CoM error
        compensation['hip_offset'][0] = -com_error[0] * self.control_params['hip_compensation_gain'] * 0.1
        compensation['hip_offset'][1] = -com_error[1] * self.control_params['hip_compensation_gain'] * 0.05

        # Ankle torque for ZMP error
        compensation['ankle_torque'][0] = -zmp_error[0] * self.control_params['ankle_stiffness'] * 0.1
        compensation['ankle_torque'][1] = -zmp_error[1] * self.control_params['ankle_stiffness'] * 0.1

        # CoM adjustment based on step phase (more adjustment during swing phase)
        phase_factor = 1.0 if 0.3 < step_phase < 0.7 else 0.5  # Peak adjustment during mid-swing
        compensation['com_adjustment'][0] = -com_error[0] * phase_factor * 0.3
        compensation['com_adjustment'][1] = -com_error[1] * phase_factor * 0.2

        return compensation

    def execute_stair_step(self, current_state, target_foot_pos, terrain_normal, step_type='up'):
        """Execute a single stair step with appropriate control"""
        # Generate foot trajectory
        foot_trajectory = self.generate_foot_lift_trajectory(
            current_state['current_foot_pos'],
            target_foot_pos,
            step_type
        )

        # Calculate ankle compensation
        ankle_roll, ankle_pitch = self.calculate_ankle_compensation(terrain_normal)

        # Calculate balance compensation
        com_error = current_state['com_error']
        zmp_error = current_state['zmp_error']
        step_phase = current_state['step_phase']

        balance_comp = self.calculate_balance_compensation(com_error, zmp_error, step_phase)

        # Combine all control commands
        control_commands = {
            'foot_trajectory': foot_trajectory,
            'ankle_compensation': (ankle_roll, ankle_pitch),
            'balance_compensation': balance_comp,
            'step_timing': self.calculate_step_timing(step_type)
        }

        return control_commands

    def calculate_step_timing(self, step_type):
        """Calculate appropriate step timing based on step type"""
        base_time = 1.0  # Base step time for stairs

        if step_type == 'up':
            # Going up requires more careful placement
            return base_time * 1.2
        elif step_type == 'down':
            # Going down requires careful control to prevent falling
            return base_time * 1.1
        else:
            # Level walking
            return base_time * 0.8

    def detect_stair_phase(self, robot_state, stair_info):
        """Detect current phase of stair negotiation"""
        robot_x = robot_state['position'][0]
        stair_start = stair_info['start_x']
        stair_end = stair_info['end_x']
        step_width = stair_info['avg_run']

        if robot_x < stair_start - 0.5:
            return 'approach'
        elif robot_x < stair_start:
            return 'pre_stance'  # Just before first step
        elif robot_x <= stair_end:
            # Determine which step we're on
            step_number = int((robot_x - stair_start) / step_width)
            return f'step_{step_number + 1}'
        elif robot_x <= stair_end + 0.5:
            return 'post_stance'  # Just after last step
        else:
            return 'departure'

    def adapt_control_for_terrain_uncertainty(self, terrain_estimate, confidence_level):
        """Adapt control parameters based on terrain estimation uncertainty"""
        # Reduce step size and increase caution with low confidence
        adaptation_factor = confidence_level

        # Adjust parameters based on confidence
        new_params = self.control_params.copy()
        new_params['foot_lift_height'] *= (0.7 + 0.3 * confidence_level)  # Higher lift with low confidence
        new_params['ankle_stiffness'] *= (0.8 + 0.2 * confidence_level)  # Softer with low confidence

        return new_params

# Example: Stair negotiation control
stair_controller = StairNegotiationController()

# Simulate a stair negotiation scenario
current_state = {
    'position': [0.5, 0.0, 0.85],
    'current_foot_pos': [0.5, 0.1, 0.0],  # Current foot position
    'com_error': [0.02, -0.01, 0.0],      # CoM position error
    'zmp_error': [0.01, 0.005, 0.0],      # ZMP error
    'step_phase': 0.4                      # Current step phase [0-1]
}

if detected_stairs:
    target_pos = [0.8, 0.1, 0.17]  # Next step position
    terrain_normal = [0.0, 0.0, 1.0]  # Flat surface normal
    step_type = 'up'

    control_commands = stair_controller.execute_stair_step(
        current_state, target_pos, terrain_normal, step_type
    )

    print(f"Stair negotiation control commands:")
    print(f"  Foot trajectory points: {len(control_commands['foot_trajectory'])}")
    print(f"  Ankle compensation: roll={control_commands['ankle_compensation'][0]:.3f}, "
          f"pitch={control_commands['ankle_compensation'][1]:.3f}")
    print(f"  Step timing: {control_commands['step_timing']:.2f}s")
```

## Narrow Passage Navigation

### Gap and Obstacle Navigation

Navigating through narrow passages and around obstacles:

```python
class NarrowPassageNavigator:
    def __init__(self, robot_width=0.4):
        self.robot_width = robot_width
        self.min_passage_width = robot_width * 1.2  # Need 20% clearance
        self.side_step_distance = 0.15  # 15cm side steps
        self.forward_step_distance = 0.2  # 20cm forward steps

    def detect_passage_width(self, point_cloud, robot_pos, search_radius=1.0):
        """Detect the width of passages in the environment"""
        passages = []

        # Define search directions perpendicular to robot's intended direction
        # For simplicity, assume robot moves along X-axis
        y_range = np.linspace(robot_pos[1] - search_radius, robot_pos[1] + search_radius, 20)

        for y in y_range:
            # Find minimum distance to obstacles on left and right
            left_clearance = self.find_clearance_to_obstacle(point_cloud, robot_pos[0], y, -1)
            right_clearance = self.find_clearance_to_obstacle(point_cloud, robot_pos[0], y, 1)

            total_width = left_clearance + right_clearance
            if total_width > self.min_passage_width:
                passages.append({
                    'x': robot_pos[0],
                    'y': y,
                    'width': total_width,
                    'left_clearance': left_clearance,
                    'right_clearance': right_clearance
                })

        return passages

    def find_clearance_to_obstacle(self, point_cloud, x, y, direction):
        """Find clearance to obstacle in given direction"""
        test_y = y
        step_size = 0.01  # 1cm steps
        max_distance = 2.0  # Search up to 2m

        for _ in range(int(max_distance / step_size)):
            test_y += direction * step_size

            # Check if there's an obstacle at this position
            distances = np.sqrt((point_cloud[:, 0] - x)**2 + (point_cloud[:, 1] - test_y)**2)
            min_distance = np.min(distances) if len(distances) > 0 else float('inf')

            if min_distance < 0.1:  # Obstacle detected
                return abs(test_y - y) - 0.1  # Subtract obstacle thickness

        return max_distance  # No obstacle found

    def plan_side_step_maneuver(self, passage_width, target_y, current_y):
        """Plan side-step maneuver for narrow passages"""
        maneuver = {
            'steps': [],
            'total_distance': 0.0,
            'estimated_time': 0.0
        }

        if passage_width < self.min_passage_width * 1.5:
            # Very narrow - plan careful side-stepping
            distance_to_target = abs(target_y - current_y)

            # Calculate number of side steps needed
            num_side_steps = int(distance_to_target / self.side_step_distance) + 1

            for i in range(num_side_steps):
                step_direction = 1 if target_y > current_y else -1
                step_y = current_y + step_direction * self.side_step_distance * (i + 1)
                step_y = min(max(step_y, target_y - 0.05), target_y + 0.05)  # Clamp to target

                maneuver['steps'].append({
                    'type': 'side_step',
                    'x': current_y,  # Will be updated as robot moves forward
                    'y': step_y,
                    'direction': step_direction
                })

        else:
            # Wide enough - direct path is possible
            maneuver['steps'].append({
                'type': 'direct',
                'x': current_y,
                'y': target_y,
                'direction': 0
            })

        return maneuver

    def generate_lateral_shuffling_pattern(self, target_offset, num_steps=4):
        """Generate lateral shuffling pattern for very narrow passages"""
        shuffling_pattern = []

        # Generate alternating lateral movements
        for i in range(num_steps):
            # Calculate lateral offset with alternating pattern
            lateral_offset = target_offset * (i % 2) * 0.5  # Alternate sides
            forward_progress = i * self.forward_step_distance / 2

            shuffling_pattern.append({
                'lateral_offset': lateral_offset,
                'forward_progress': forward_progress,
                'step_type': 'shuffle' if i % 2 == 0 else 'adjust'
            })

        return shuffling_pattern

    def plan_passage_navigation(self, start_pos, goal_pos, point_cloud):
        """Plan navigation through passages with obstacles"""
        navigation_plan = {
            'approach': [],
            'passage_traversal': [],
            'departure': [],
            'total_steps': 0
        }

        # Detect passages along the path
        passages = self.detect_passage_width(point_cloud, start_pos)

        if not passages:
            # No narrow passages detected, plan direct path
            navigation_plan['passage_traversal'] = self.plan_direct_path(start_pos, goal_pos)
        else:
            # Plan around narrow passages
            for passage in passages:
                if passage['width'] < self.min_passage_width:
                    # Need to navigate around this narrow section
                    maneuver = self.plan_side_step_maneuver(
                        passage['width'],
                        passage['y'],
                        start_pos[1]
                    )
                    navigation_plan['passage_traversal'].extend(maneuver['steps'])

        return navigation_plan

    def plan_direct_path(self, start_pos, goal_pos):
        """Plan direct path when no obstacles are present"""
        path = []
        direction = np.array(goal_pos) - np.array(start_pos)
        distance = np.linalg.norm(direction)
        direction = direction / distance if distance > 0 else np.array([1, 0, 0])

        num_steps = int(distance / self.forward_step_distance)
        step_vector = direction * self.forward_step_distance

        for i in range(num_steps):
            pos = start_pos + step_vector * i
            path.append({
                'position': pos,
                'type': 'forward_step'
            })

        return path

    def evaluate_passage_traversability(self, passage, robot_params=None):
        """Evaluate if passage is traversable by the robot"""
        if robot_params is None:
            robot_params = {'width': self.robot_width, 'height': 1.7, 'ground_clearance': 0.05}

        # Check width constraint
        width_constraint = passage['width'] >= robot_params['width'] + 0.1  # 10cm clearance

        # Check for overhead obstacles if needed
        overhead_clearance = True  # Simplified - in practice, check Z-dimension

        traversable = width_constraint and overhead_clearance
        safety_margin = passage['width'] - robot_params['width']

        return {
            'traversable': traversable,
            'safety_margin': safety_margin,
            'constraint_type': 'width' if not width_constraint else 'none'
        }

# Example: Narrow passage navigation
narrow_navigator = NarrowPassageNavigator(robot_width=0.4)

# Create sample environment with narrow passage
env_points = []
# Walls
for x in np.arange(-2, 2, 0.1):
    for z in [0, 0.1]:  # Ground and low obstacles
        env_points.append([x, -1.0, z])  # Left wall
        env_points.append([x, 1.0, z])   # Right wall

# Add some obstacles to create a narrow passage
for y in np.arange(-0.8, 0.8, 0.05):
    env_points.append([-0.5, y, 0.0])  # Obstacle creating narrow passage
    env_points.append([0.5, y, 0.0])   # Obstacle creating narrow passage

env_point_cloud = np.array(env_points)

# Detect passages
passages = narrow_navigator.detect_passage_width(env_point_cloud, [0, 0, 0.85])
print(f"Detected {len(passages)} potential passages")

for i, passage in enumerate(passages[:3]):  # Show first 3
    eval_result = narrow_navigator.evaluate_passage_traversability(passage)
    print(f"Passage {i+1}: width={passage['width']:.3f}m, "
          f"traversable={eval_result['traversable']}, "
          f"safety_margin={eval_result['safety_margin']:.3f}m")

# Plan navigation through narrow passage
navigation_plan = narrow_navigator.plan_passage_navigation(
    [0, 0, 0.85], [1, 0, 0.85], env_point_cloud
)
print(f"Navigation plan generated with {len(navigation_plan['passage_traversal'])} steps")
```

## Dynamic Obstacle Avoidance

### Real-time Obstacle Avoidance

Handling moving obstacles and people in real-time:

```python
class DynamicObstacleAvoider:
    def __init__(self):
        self.prediction_horizon = 2.0  # Predict 2 seconds ahead
        self.safety_margin = 0.3       # 30cm safety margin
        self.response_time = 0.5       # 0.5 second response time
        self.velocity_threshold = 0.1  # Stop if obstacle velocity < 0.1 m/s

    def predict_obstacle_trajectory(self, obstacle_state, time_horizon=None):
        """Predict obstacle trajectory based on current state"""
        if time_horizon is None:
            time_horizon = self.prediction_horizon

        predicted_trajectory = []
        current_pos = obstacle_state['position']
        current_vel = obstacle_state['velocity']

        dt = 0.1  # 100ms prediction steps
        for t in np.arange(0, time_horizon, dt):
            # Simple constant velocity prediction
            predicted_pos = current_pos + current_vel * t
            predicted_trajectory.append({
                'time': t,
                'position': predicted_pos,
                'velocity': current_vel
            })

        return predicted_trajectory

    def detect_collision_risk(self, robot_trajectory, obstacle_trajectories):
        """Detect potential collisions between robot and obstacles"""
        collision_risks = []

        for obs_traj in obstacle_trajectories:
            for i, robot_pos in enumerate(robot_trajectory):
                if i < len(obs_traj):
                    obs_pos = obs_traj[i]['position']
                    distance = np.linalg.norm(robot_pos[:2] - obs_pos[:2])

                    if distance < self.safety_margin:
                        collision_risks.append({
                            'time_to_collision': obs_traj[i]['time'],
                            'distance': distance,
                            'robot_position': robot_pos,
                            'obstacle_position': obs_pos,
                            'risk_level': self.calculate_risk_level(distance)
                        })

        return collision_risks

    def calculate_risk_level(self, distance):
        """Calculate risk level based on distance"""
        if distance < self.safety_margin * 0.5:
            return 'critical'
        elif distance < self.safety_margin:
            return 'high'
        elif distance < self.safety_margin * 1.5:
            return 'medium'
        else:
            return 'low'

    def generate_avoidance_trajectory(self, current_robot_pos, current_robot_vel,
                                    collision_risks, obstacle_trajectories):
        """Generate avoidance trajectory to prevent collisions"""
        if not collision_risks:
            return None  # No risks detected

        # Find the earliest and most critical collision
        critical_risk = min(collision_risks, key=lambda x: x['time_to_collision'])
        if critical_risk['risk_level'] == 'low':
            return None  # Low risk, no avoidance needed

        # Calculate avoidance direction (perpendicular to relative velocity)
        closest_obstacle_pos = critical_risk['obstacle_position'][:2]
        robot_to_obstacle = closest_obstacle_pos - current_robot_pos[:2]

        # Calculate perpendicular direction for avoidance
        avoidance_direction = np.array([-robot_to_obstacle[1], robot_to_obstacle[0]])
        avoidance_direction = avoidance_direction / np.linalg.norm(avoidance_direction)

        # Generate avoidance maneuver
        avoidance_trajectory = []
        avoidance_distance = max(0.5, self.safety_margin * 2)  # Avoid by at least 0.5m

        # Simple lateral avoidance
        for t in np.arange(0, 1.0, 0.1):  # 1 second avoidance maneuver
            lateral_offset = avoidance_direction * avoidance_distance * t
            new_pos = current_robot_pos.copy()
            new_pos[:2] += lateral_offset
            avoidance_trajectory.append(new_pos)

        return avoidance_trajectory

    def update_obstacle_state(self, obstacle_id, new_position, new_time, previous_states=None):
        """Update obstacle state with new measurement"""
        if previous_states is None:
            previous_states = []

        # Calculate velocity from position changes
        if previous_states:
            last_state = previous_states[-1]
            time_diff = new_time - last_state['timestamp']
            if time_diff > 0:
                velocity = (np.array(new_position) - np.array(last_state['position'])) / time_diff
            else:
                velocity = np.array([0, 0, 0])
        else:
            velocity = np.array([0, 0, 0])

        return {
            'id': obstacle_id,
            'position': np.array(new_position),
            'velocity': velocity,
            'timestamp': new_time,
            'predicted_trajectory': self.predict_obstacle_trajectory({
                'position': np.array(new_position),
                'velocity': velocity
            })
        }

    def plan_reactive_avoidance(self, robot_state, dynamic_obstacles):
        """Plan reactive avoidance based on current obstacle states"""
        avoidance_plan = {
            'needs_avoidance': False,
            'avoidance_trajectory': [],
            'new_goal': None,
            'wait_time': 0.0
        }

        # Check immediate collision risk
        immediate_risk = False
        for obs in dynamic_obstacles:
            distance = np.linalg.norm(robot_state['position'][:2] - obs['position'][:2])
            if distance < self.safety_margin:
                immediate_risk = True

        if immediate_risk:
            # Calculate immediate avoidance action
            avoidance_trajectory = self.calculate_immediate_avoidance(
                robot_state, dynamic_obstacles
            )
            if avoidance_trajectory:
                avoidance_plan['needs_avoidance'] = True
                avoidance_plan['avoidance_trajectory'] = avoidance_trajectory

        return avoidance_plan

    def calculate_immediate_avoidance(self, robot_state, obstacles):
        """Calculate immediate avoidance action"""
        # Find the closest obstacle
        closest_obstacle = min(obstacles, key=lambda obs:
                             np.linalg.norm(robot_state['position'][:2] - obs['position'][:2]))

        # Calculate avoidance direction
        robot_to_obstacle = closest_obstacle['position'][:2] - robot_state['position'][:2]
        perpendicular = np.array([-robot_to_obstacle[1], robot_to_obstacle[0]])
        perpendicular = perpendicular / np.linalg.norm(perpendicular)

        # Generate immediate avoidance step
        avoidance_step = robot_state['position'].copy()
        avoidance_step[:2] += perpendicular * 0.2  # 20cm lateral step

        return [avoidance_step]

    def evaluate_avoidance_feasibility(self, avoidance_trajectory, environment_map):
        """Evaluate if avoidance trajectory is feasible"""
        if not avoidance_trajectory:
            return True  # No avoidance needed

        for point in avoidance_trajectory:
            x, y = point[:2]
            # Check if this point is in free space
            # This would interface with the environment map
            # For now, return True (in a real system, check for obstacles)
            pass

        return True  # Simplified - assume feasible

# Example: Dynamic obstacle avoidance
avoider = DynamicObstacleAvoider()

# Simulate dynamic obstacles
obstacle1 = {
    'id': 'person1',
    'position': np.array([1.0, 0.5, 0.0]),
    'velocity': np.array([-0.5, 0.0, 0.0]),  # Moving left
    'timestamp': time.time()
}

obstacle2 = {
    'id': 'person2',
    'position': np.array([0.5, -1.0, 0.0]),
    'velocity': np.array([0.0, 0.3, 0.0]),   # Moving up
    'timestamp': time.time()
}

dynamic_obstacles = [obstacle1, obstacle2]

# Predict obstacle trajectories
for obs in dynamic_obstacles:
    obs['predicted_trajectory'] = avoider.predict_obstacle_trajectory(obs)

robot_state = {
    'position': np.array([0.0, 0.0, 0.85]),
    'velocity': np.array([0.2, 0.0, 0.0]),
    'goal': np.array([2.0, 0.0, 0.85])
}

# Plan avoidance
avoidance_plan = avoider.plan_reactive_avoidance(robot_state, dynamic_obstacles)
print(f"Avoidance plan: needs_avoidance = {avoidance_plan['needs_avoidance']}")
print(f"Number of obstacles: {len(dynamic_obstacles)}")

# Update obstacle state with new measurement
new_obs_pos = [0.8, 0.5, 0.0]  # Person moved
updated_obs = avoider.update_obstacle_state('person1', new_obs_pos, time.time() + 1.0)
print(f"Updated obstacle velocity: {updated_obs['velocity']}")
```

## Slippery Surface Navigation

### Adaptation for Low-Friction Surfaces

Handling surfaces with reduced friction:

```python
class SlipperySurfaceNavigator:
    def __init__(self):
        self.friction_coefficients = {
            'dry_concrete': 0.8,
            'wet_surface': 0.4,
            'ice': 0.1,
            'grass': 0.6,
            'sand': 0.5
        }
        self.slip_threshold = 0.1  # Threshold for slip detection
        self.adaptation_params = {
            'reduced_step_length': 0.15,    # Shorter steps on slippery surfaces
            'increased_step_height': 0.02,  # Higher foot clearance
            'slower_step_frequency': 0.6,   # Reduce step frequency by 40%
            'increased_com_height': 0.02    # Slightly raise CoM for stability
        }

    def estimate_surface_friction(self, sensor_data):
        """Estimate surface friction from sensor data"""
        # This would use various sensors: cameras for visual cues,
        # force/torque sensors for slip detection, etc.
        # For simulation, we'll use a simplified approach

        if 'visual_texture' in sensor_data:
            texture = sensor_data['visual_texture']
            if 'wet' in texture or 'ice' in texture:
                return self.friction_coefficients['wet_surface']
            elif 'grass' in texture:
                return self.friction_coefficients['grass']
            else:
                return self.friction_coefficients['dry_concrete']
        else:
            # Default to dry concrete if no visual data
            return self.friction_coefficients['dry_concrete']

    def detect_slip_conditions(self, force_data, acceleration_data):
        """Detect slip conditions from force and acceleration sensors"""
        slip_detected = False
        slip_magnitude = 0.0

        if 'tangential_force' in force_data and 'normal_force' in force_data:
            tangential_force = np.linalg.norm(force_data['tangential_force'])
            normal_force = force_data['normal_force']

            if normal_force > 0:
                friction_ratio = tangential_force / normal_force
                # If friction ratio approaches or exceeds friction coefficient, slip is occurring
                if friction_ratio > 0.8:  # 80% of maximum friction
                    slip_detected = True
                    slip_magnitude = friction_ratio

        # Also check for unexpected accelerations
        if 'foot_acceleration' in acceleration_data:
            foot_acc = acceleration_data['foot_acceleration']
            if np.linalg.norm(foot_acc) > 5.0:  # High acceleration may indicate slip
                slip_detected = True
                slip_magnitude = max(slip_magnitude, np.linalg.norm(foot_acc) / 10.0)

        return slip_detected, slip_magnitude

    def adapt_gait_for_low_friction(self, base_gait_params, friction_coeff):
        """Adapt gait parameters for low friction conditions"""
        adapted_params = base_gait_params.copy()

        # Calculate adaptation factor based on friction
        friction_factor = friction_coeff / self.friction_coefficients['dry_concrete']

        # Reduce step length proportionally to friction
        adapted_params['step_length'] *= max(0.3, friction_factor * 0.7)  # Min 30% of normal

        # Increase step height for better foot clearance
        adapted_params['swing_height'] *= (1 + (1 - friction_factor) * 0.5)

        # Reduce step frequency
        adapted_params['step_time'] /= max(0.4, friction_factor)  # Max 2.5x slower

        # Increase stance time for better contact
        adapted_params['stance_ratio'] = min(0.8, adapted_params.get('stance_ratio', 0.6) + (1 - friction_factor) * 0.2)

        # Adjust foot placement for stability
        adapted_params['step_width'] = max(0.15, adapted_params.get('step_width', 0.2) * (1 + (1 - friction_factor) * 0.3))

        return adapted_params

    def generate_stabilization_pattern(self, current_state, friction_coeff):
        """Generate stabilization pattern for slippery conditions"""
        stabilization_pattern = {
            'com_modulation': [],
            'ankle_adjustments': [],
            'step_timing': []
        }

        # Generate CoM modulation to maintain balance
        base_com_x = current_state['com_position'][0]
        base_com_y = current_state['com_position'][1]

        for i in range(10):  # 10 steps of stabilization pattern
            t = i * 0.1  # Time parameter

            # Reduce CoM lateral movement to maintain stability
            lateral_reduction = 1 - (1 - friction_coeff) * 0.5
            com_y_offset = 0.02 * lateral_reduction * np.sin(2 * np.pi * t)  # Reduced sway

            # Increase CoM height slightly for better stability
            com_z_offset = current_state['com_position'][2] + self.adaptation_params['increased_com_height'] * (1 - friction_coeff)

            stabilization_pattern['com_modulation'].append([
                base_com_x + 0.01 * t,  # Slow forward progression
                base_com_y + com_y_offset,
                com_z_offset
            ])

            # Ankle adjustments for better grip
            ankle_roll = (1 - friction_coeff) * 0.05 * np.sin(4 * np.pi * t)  # Small modulations
            ankle_pitch = (1 - friction_coeff) * 0.03 * np.cos(4 * np.pi * t)
            stabilization_pattern['ankle_adjustments'].append([ankle_roll, ankle_pitch])

            # Adjust step timing based on friction
            step_timing = 0.8 + (1 - friction_coeff) * 0.4  # Slower on slippery surfaces
            stabilization_pattern['step_timing'].append(step_timing)

        return stabilization_pattern

    def implement_ankle_control_for_slippery_surfaces(self, current_ankle_state, desired_ankle_state, friction_coeff):
        """Implement specialized ankle control for slippery surfaces"""
        # On slippery surfaces, reduce ankle stiffness to prevent sudden slips
        reduced_stiffness_factor = friction_coeff * 0.7  # Reduce stiffness proportionally

        # Calculate control command with reduced gains
        position_error = desired_ankle_state - current_ankle_state
        control_command = position_error * reduced_stiffness_factor

        # Add damping to reduce oscillations
        damping_factor = 0.3 + (1 - friction_coeff) * 0.4  # Higher damping on slippery surfaces
        control_command *= (1 - damping_factor)

        return control_command

    def evaluate_traction_maintenance(self, foot_force_data, friction_coeff):
        """Evaluate how well traction is being maintained"""
        traction_score = 0.0

        if 'ground_reaction_forces' in foot_force_data:
            forces = foot_force_data['ground_reaction_forces']
            for force in forces:
                tangential = np.linalg.norm(force[:2])  # X, Y forces
                normal = abs(force[2])  # Z force

                if normal > 0:
                    friction_utilization = tangential / normal
                    # Score is higher when friction utilization is lower (more margin)
                    margin = friction_coeff - friction_utilization
                    traction_score += max(0, margin) / friction_coeff

        return traction_score / len(forces) if forces else 1.0  # Perfect score if no data

# Example: Slippery surface navigation
slippery_navigator = SlipperySurfaceNavigator()

# Simulate different surface conditions
surfaces = ['dry_concrete', 'wet_surface', 'ice']
for surface in surfaces:
    friction = slippery_navigator.friction_coefficients[surface]
    print(f"{surface}: friction coefficient = {friction}")

    # Adapt gait for this surface
    base_params = {
        'step_length': 0.3,
        'step_time': 0.8,
        'swing_height': 0.05,
        'step_width': 0.2
    }

    adapted_params = slippery_navigator.adapt_gait_for_low_friction(base_params, friction)
    print(f"  Adapted step length: {adapted_params['step_length']:.3f}m "
          f"(reduction: {(1-adapted_params['step_length']/base_params['step_length'])*100:.1f}%)")

# Simulate slip detection
sensor_data = {
    'tangential_force': [20, 10, 5],  # High tangential force may indicate slip
    'normal_force': 400  # Normal force from foot
}

slip_detected, slip_mag = slippery_navigator.detect_slip_conditions(
    sensor_data, {'foot_acceleration': [1.0, 2.0, 9.0]}
)
print(f"\nSlip detection: {slip_detected}, magnitude: {slip_mag:.3f}")
```

## Integration with Whole-Body Control

### Coordinated Multi-System Navigation

Integrating all complex terrain capabilities into a unified system:

```python
class ComplexTerrainNavigator:
    def __init__(self):
        # Initialize all subsystems
        self.stair_detector = StairDetector()
        self.stair_gait_gen = StairClimbingGaitGenerator()
        self.stair_controller = StairNegotiationController()
        self.narrow_navigator = NarrowPassageNavigator()
        self.dynamic_avoider = DynamicObstacleAvoider()
        self.slippery_navigator = SlipperySurfaceNavigator()

        # Navigation state
        self.current_mode = 'walking'  # walking, stair_ascent, stair_descent, narrow_passage, avoidance
        self.navigation_context = {}
        self.safety_monitor = SafetyMonitor()

    def analyze_environment(self, sensor_data):
        """Analyze environment to determine navigation requirements"""
        environment_analysis = {
            'terrain_type': 'flat',
            'obstacles': [],
            'passages': [],
            'stairs': [],
            'surface_conditions': 'dry',
            'dynamic_objects': []
        }

        # Detect stairs
        if 'point_cloud' in sensor_data:
            stairs = self.stair_detector.detect_stairs_from_3d_point_cloud(
                sensor_data['point_cloud']
            )
            environment_analysis['stairs'] = stairs

        # Detect narrow passages
        if 'point_cloud' in sensor_data and 'robot_position' in sensor_data:
            passages = self.narrow_navigator.detect_passage_width(
                sensor_data['point_cloud'],
                sensor_data['robot_position']
            )
            environment_analysis['passages'] = passages

        # Detect dynamic obstacles
        if 'dynamic_objects' in sensor_data:
            environment_analysis['dynamic_objects'] = sensor_data['dynamic_objects']

        # Estimate surface conditions
        if 'force_data' in sensor_data:
            slip_detected, _ = self.slippery_navigator.detect_slip_conditions(
                sensor_data['force_data'], {}
            )
            if slip_detected:
                environment_analysis['surface_conditions'] = 'slippery'

        return environment_analysis

    def select_navigation_mode(self, environment_analysis, robot_state):
        """Select appropriate navigation mode based on environment"""
        # Priority order for mode selection
        if environment_analysis['dynamic_objects']:
            # Check if immediate collision avoidance is needed
            for obj in environment_analysis['dynamic_objects']:
                distance = np.linalg.norm(
                    robot_state['position'][:2] - obj['position'][:2]
                )
                if distance < 0.5:  # 50cm critical zone
                    return 'collision_avoidance'

        # Check for stairs
        if environment_analysis['stairs']:
            # Determine if we need to climb or descend
            robot_x = robot_state['position'][0]
            for stair in environment_analysis['stairs']:
                if abs(robot_x - stair['start_x']) < 1.0:  # Within approach distance
                    if stair['type'] == 'ascending':
                        return 'stair_ascent'
                    else:
                        return 'stair_descent'

        # Check for narrow passages
        if environment_analysis['passages']:
            for passage in environment_analysis['passages']:
                if passage['width'] < self.narrow_navigator.min_passage_width * 1.5:
                    return 'narrow_passage'

        # Surface conditions
        if environment_analysis['surface_conditions'] == 'slippery':
            return 'slippery_surface'

        # Default to normal walking
        return 'walking'

    def generate_navigation_plan(self, environment_analysis, robot_state, goal):
        """Generate navigation plan based on environment analysis"""
        navigation_plan = {
            'mode': self.select_navigation_mode(environment_analysis, robot_state),
            'waypoints': [],
            'control_parameters': {},
            'safety_checks': []
        }

        if navigation_plan['mode'] == 'stair_ascent':
            if environment_analysis['stairs']:
                stair_info = environment_analysis['stairs'][0]  # Use first detected stairs
                trajectory = self.stair_gait_gen.generate_complete_stair_trajectory(stair_info)
                navigation_plan['waypoints'] = self.extract_waypoints_from_trajectory(trajectory)
                navigation_plan['control_parameters'] = self.get_stair_control_params()

        elif navigation_plan['mode'] == 'narrow_passage':
            if environment_analysis['passages']:
                passage = environment_analysis['passages'][0]  # Use first narrow passage
                navigation_plan['waypoints'] = self.plan_narrow_passage_path(passage, robot_state, goal)

        elif navigation_plan['mode'] == 'collision_avoidance':
            if environment_analysis['dynamic_objects']:
                navigation_plan['waypoints'] = self.plan_avoidance_path(
                    robot_state, environment_analysis['dynamic_objects'], goal
                )

        elif navigation_plan['mode'] == 'slippery_surface':
            navigation_plan['control_parameters'] = self.get_slippery_surface_params(
                environment_analysis
            )

        else:  # normal walking
            navigation_plan['waypoints'] = self.plan_normal_path(robot_state, goal)

        return navigation_plan

    def extract_waypoints_from_trajectory(self, trajectory):
        """Extract waypoints from complex trajectory"""
        waypoints = []

        # Combine approach, climbing, and departure trajectories
        for phase in ['approach', 'climbing', 'departure']:
            if phase in trajectory and trajectory[phase]:
                phase_data = trajectory[phase]
                for i in range(len(phase_data.get('left_foot', []))):
                    # Average left and right foot positions for CoM guidance
                    if i < len(phase_data.get('left_foot', [])) and i < len(phase_data.get('right_foot', [])):
                        left_pos = phase_data['left_foot'][i]
                        right_pos = phase_data['right_foot'][i]
                        avg_pos = [(l + r) / 2 for l, r in zip(left_pos, right_pos)]
                        avg_pos[2] = (left_pos[2] + right_pos[2]) / 2  # Average height
                        waypoints.append(avg_pos)

        return waypoints

    def get_stair_control_params(self):
        """Get control parameters for stair climbing"""
        return {
            'ankle_stiffness': 250,
            'step_timing': 1.2,
            'com_height_offset': 0.03,
            'foot_lift_height': 0.08
        }

    def plan_narrow_passage_path(self, passage, robot_state, goal):
        """Plan path through narrow passage"""
        # Use narrow passage navigator to plan the path
        navigation_plan = self.narrow_navigator.plan_passage_navigation(
            robot_state['position'], goal, self.get_current_point_cloud()
        )
        return self.extract_waypoints_from_navigation_plan(navigation_plan)

    def plan_avoidance_path(self, robot_state, dynamic_objects, goal):
        """Plan path to avoid dynamic obstacles"""
        # Use dynamic avoider to plan around obstacles
        avoidance_plan = self.dynamic_avoider.plan_reactive_avoidance(
            robot_state, dynamic_objects
        )

        if avoidance_plan['needs_avoidance'] and avoidance_plan['avoidance_trajectory']:
            return avoidance_plan['avoidance_trajectory']
        else:
            return [goal]  # Direct path if no immediate danger

    def get_slippery_surface_params(self, environment_analysis):
        """Get parameters for slippery surface navigation"""
        # Estimate friction from environment
        friction_coeff = self.slippery_navigator.estimate_surface_friction(
            environment_analysis
        )
        return self.slippery_navigator.adapt_gait_for_low_friction(
            {'step_length': 0.3, 'step_time': 0.8, 'swing_height': 0.05},
            friction_coeff
        )

    def plan_normal_path(self, robot_state, goal):
        """Plan normal walking path"""
        # Simple straight-line path for normal walking
        direction = goal - robot_state['position']
        distance = np.linalg.norm(direction)
        direction = direction / distance if distance > 0 else np.array([1, 0, 0])

        waypoints = []
        step_size = 0.3  # Normal step size
        num_steps = int(distance / step_size)

        for i in range(num_steps + 1):
            pos = robot_state['position'] + direction * step_size * i
            waypoints.append(pos)

        return waypoints

    def extract_waypoints_from_navigation_plan(self, plan):
        """Extract waypoints from navigation plan"""
        waypoints = []
        for phase in ['approach', 'passage_traversal', 'departure']:
            if phase in plan:
                for step in plan[phase]:
                    if isinstance(step, dict) and 'position' in step:
                        waypoints.append(step['position'])
                    elif isinstance(step, dict) and 'type' in step:
                        # For narrow passage steps
                        waypoints.append([step.get('x', 0), step.get('y', 0), 0.85])
        return waypoints

    def get_current_point_cloud(self):
        """Get current point cloud (placeholder for actual sensor data)"""
        # In a real system, this would come from sensors
        return np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]])  # Placeholder

    def execute_navigation_step(self, robot_state, navigation_plan):
        """Execute one step of the navigation plan"""
        # This would interface with the robot's control system
        # For simulation, return the next planned position
        if navigation_plan['waypoints']:
            return navigation_plan['waypoints'][0]  # Return first waypoint
        else:
            return robot_state['position']  # Stay in place if no plan

    def monitor_safety(self, robot_state, environment_analysis):
        """Monitor safety during navigation"""
        safety_status = {
            'stable': True,
            'emergency_stop': False,
            'warnings': []
        }

        # Check for critical conditions
        if environment_analysis['surface_conditions'] == 'slippery':
            if 'force_data' in robot_state:
                slip_detected, slip_mag = self.slippery_navigator.detect_slip_conditions(
                    robot_state['force_data'], {}
                )
                if slip_detected and slip_mag > 0.2:
                    safety_status['warnings'].append('High slip risk detected')
                    if slip_mag > 0.3:
                        safety_status['emergency_stop'] = True

        # Check balance
        if 'com_position' in robot_state and 'zmp' in robot_state:
            com_pos = robot_state['com_position']
            zmp_pos = robot_state['zmp']
            stability_margin = np.linalg.norm(com_pos[:2] - zmp_pos[:2])
            if stability_margin > 0.3:  # Too far from ZMP
                safety_status['warnings'].append('Stability margin exceeded')
                if stability_margin > 0.5:
                    safety_status['emergency_stop'] = True

        return safety_status

class SafetyMonitor:
    def __init__(self):
        self.safety_thresholds = {
            'com_zmp_distance': 0.3,      # Max distance from ZMP
            'angular_velocity': 1.0,      # Max angular velocity (rad/s)
            'joint_limit_margin': 0.1,    # Min margin from joint limits
            'collision_distance': 0.2     # Min distance to obstacles
        }

    def check_safety(self, robot_state):
        """Check if current robot state is safe"""
        safety_issues = []

        # Check ZMP stability
        if 'com_position' in robot_state and 'zmp' in robot_state:
            com_zmp_dist = np.linalg.norm(
                robot_state['com_position'][:2] - robot_state['zmp'][:2]
            )
            if com_zmp_dist > self.safety_thresholds['com_zmp_distance']:
                safety_issues.append(f"ZMP stability exceeded: {com_zmp_dist:.3f}m")

        # Check angular velocity
        if 'angular_velocity' in robot_state:
            ang_vel_mag = np.linalg.norm(robot_state['angular_velocity'])
            if ang_vel_mag > self.safety_thresholds['angular_velocity']:
                safety_issues.append(f"High angular velocity: {ang_vel_mag:.3f} rad/s")

        return len(safety_issues) == 0, safety_issues

# Example: Complex terrain navigation
complex_navigator = ComplexTerrainNavigator()

# Simulate environment analysis
sensor_data = {
    'point_cloud': stair_points,  # Use the stair point cloud from earlier
    'robot_position': [0, 0, 0.85],
    'dynamic_objects': [],
    'force_data': {
        'tangential_force': [10, 5, 2],
        'normal_force': 600
    }
}

environment_analysis = complex_navigator.analyze_environment(sensor_data)
print(f"Environment analysis:")
print(f"  Terrain type: {environment_analysis['terrain_type']}")
print(f"  Detected stairs: {len(environment_analysis['stairs'])}")
print(f"  Surface condition: {environment_analysis['surface_conditions']}")

# Generate navigation plan
robot_state = {
    'position': [0, 0, 0.85],
    'com_position': [0, 0, 0.85],
    'zmp': [0, 0, 0],
    'angular_velocity': [0.1, 0.05, 0.02]
}
goal = [2, 0, 0.85]

navigation_plan = complex_navigator.generate_navigation_plan(
    environment_analysis, robot_state, goal
)

print(f"\nNavigation plan:")
print(f"  Mode: {navigation_plan['mode']}")
print(f"  Waypoints: {len(navigation_plan['waypoints'])}")
print(f"  Control parameters: {list(navigation_plan['control_parameters'].keys())}")

# Check safety
safety_ok, issues = complex_navigator.safety_monitor.check_safety(robot_state)
print(f"\nSafety check: {'OK' if safety_ok else 'ISSUES'}")
if issues:
    for issue in issues:
        print(f"  - {issue}")
```

## Performance Evaluation and Benchmarking

### Complex Terrain Navigation Metrics

Evaluating performance on complex terrain scenarios:

```python
class ComplexTerrainEvaluator:
    def __init__(self):
        self.metrics = {
            'success_rate': [],
            'navigation_time': [],
            'energy_efficiency': [],
            'stability_score': [],
            'adaptation_accuracy': [],
            'safety_incidents': []
        }

    def evaluate_stair_navigation(self, robot_trajectory, stair_reference, execution_time):
        """Evaluate stair navigation performance"""
        success = self.check_stair_navigation_success(robot_trajectory, stair_reference)
        time_efficiency = self.evaluate_time_efficiency(execution_time, stair_reference['num_steps'])
        stability = self.evaluate_stability(robot_trajectory)

        return {
            'success': success,
            'time_efficiency': time_efficiency,
            'stability': stability,
            'overall_score': (success + time_efficiency + stability) / 3
        }

    def check_stair_navigation_success(self, trajectory, stair_reference):
        """Check if stair navigation was successful"""
        if len(trajectory) < stair_reference['num_steps']:
            return 0.0  # Incomplete

        # Check if robot reached appropriate height
        expected_height = stair_reference['step_heights'][-1]
        actual_height = trajectory[-1][2] if len(trajectory) > 0 else 0

        height_error = abs(expected_height - actual_height)
        if height_error > 0.1:  # 10cm tolerance
            return 0.5  # Partial success

        # Check step accuracy (how close to expected step positions)
        step_accuracy = 0.0
        if len(trajectory) >= stair_reference['num_steps']:
            for i in range(min(len(stair_reference['step_heights']), len(trajectory))):
                expected_xy = [stair_reference['start_x'] + i * stair_reference['avg_run'], 0]
                actual_xy = trajectory[i][:2]
                step_error = np.linalg.norm(np.array(expected_xy) - actual_xy[:2])
                step_accuracy += max(0, 1 - step_error / 0.2)  # 20cm tolerance

            step_accuracy /= len(stair_reference['step_heights'])

        return 0.7 * (1 if height_error <= 0.05 else 0.5) + 0.3 * step_accuracy

    def evaluate_narrow_passage_navigation(self, trajectory, passage_width, execution_time):
        """Evaluate narrow passage navigation"""
        success = 1.0  # Assume success for now
        efficiency = len(trajectory) / max(1, execution_time)  # Steps per second
        safety_margin = passage_width / 0.4  # Ratio to robot width

        return {
            'success_rate': success,
            'navigation_efficiency': efficiency,
            'safety_margin': min(1.0, safety_margin),
            'overall_score': (success + min(1.0, efficiency/2) + min(1.0, safety_margin)) / 3
        }

    def evaluate_dynamic_obstacle_avoidance(self, robot_path, obstacle_trajectories, goal_reached):
        """Evaluate dynamic obstacle avoidance"""
        if not goal_reached:
            return {'success': 0.0, 'efficiency': 0.0, 'safety': 0.0}

        # Calculate path efficiency (how direct was the path despite obstacles)
        direct_distance = np.linalg.norm(robot_path[-1] - robot_path[0])
        actual_distance = sum(np.linalg.norm(robot_path[i+1] - robot_path[i])
                            for i in range(len(robot_path)-1))

        path_efficiency = direct_distance / actual_distance if actual_distance > 0 else 0.0

        # Calculate safety (minimum distance to obstacles)
        min_distance_to_obstacles = float('inf')
        for obs_traj in obstacle_trajectories:
            for robot_pos in robot_path:
                for obs_pos in [obs['position'] for obs in obs_traj]:
                    dist = np.linalg.norm(robot_pos[:2] - obs_pos[:2])
                    min_distance_to_obstacles = min(min_distance_to_obstacles, dist)

        safety_score = 1.0 if min_distance_to_obstacles > 0.5 else min_distance_to_obstacles / 0.5

        return {
            'success': 1.0,
            'path_efficiency': max(0.0, path_efficiency),
            'safety_score': safety_score,
            'overall_score': (1.0 + max(0.0, path_efficiency) + safety_score) / 3
        }

    def evaluate_slippery_surface_navigation(self, trajectory, surface_friction, slip_events):
        """Evaluate slippery surface navigation"""
        # Count successful steps without excessive slip
        successful_steps = sum(1 for slip in slip_events if slip < 0.1)  # Less than 10cm slip
        total_steps = len(slip_events)

        success_rate = successful_steps / total_steps if total_steps > 0 else 1.0

        # Evaluate energy efficiency (may be higher on slippery surfaces due to caution)
        energy_cost = len(trajectory) * 1.0  # Simplified energy model
        efficiency = 1.0 / (1.0 + energy_cost * 0.1)  # Higher cost = lower efficiency

        return {
            'success_rate': success_rate,
            'energy_efficiency': efficiency,
            'stability': success_rate,  # Stability related to slip control
            'overall_score': (success_rate + efficiency + success_rate) / 3
        }

    def evaluate_overall_complexity_handling(self, scenario_results):
        """Evaluate overall ability to handle complex terrain"""
        total_score = 0
        scenario_count = 0

        for scenario_type, results in scenario_results.items():
            if isinstance(results, dict) and 'overall_score' in results:
                total_score += results['overall_score']
                scenario_count += 1

        return total_score / scenario_count if scenario_count > 0 else 0.0

    def generate_performance_report(self, scenario_results):
        """Generate comprehensive performance report"""
        report = {
            'overall_complexity_score': self.evaluate_overall_complexity_handling(scenario_results),
            'detailed_results': scenario_results,
            'improvement_recommendations': []
        }

        # Analyze results to provide recommendations
        if report['overall_complexity_score'] < 0.7:
            report['improvement_recommendations'].append(
                "Consider improving stair detection and gait generation algorithms"
            )
        if any(results.get('safety_score', 1.0) < 0.8 for results in scenario_results.values() if isinstance(results, dict)):
            report['improvement_recommendations'].append(
                "Enhance safety monitoring and emergency stop capabilities"
            )

        return report

# Example: Performance evaluation
evaluator = ComplexTerrainEvaluator()

# Simulate evaluation of different scenarios
scenario_results = {
    'stair_navigation': evaluator.check_stair_navigation_success(
        [[0, 0, 0], [0.3, 0.1, 0.17], [0.6, -0.1, 0.34]],  # Sample trajectory
        {'num_steps': 2, 'step_heights': [0.17, 0.34], 'start_x': 0, 'avg_run': 0.3}
    ),
    'narrow_passage': evaluator.evaluate_narrow_passage_navigation(
        [[0, 0, 0.85], [0.5, 0.1, 0.85], [1, 0, 0.85]],  # Trajectory
        0.5,  # Passage width
        5.0   # Execution time
    ),
    'obstacle_avoidance': evaluator.evaluate_dynamic_obstacle_avoidance(
        [[0, 0, 0.85], [0.5, 0.2, 0.85], [1, 0, 0.85]],  # Robot path
        [[{'position': [0.5, 0.5, 0]}]],  # Obstacle trajectory
        True  # Goal reached
    )
}

# Generate report
report = evaluator.generate_performance_report(scenario_results)
print(f"\nComplex Terrain Navigation Performance Report:")
print(f"Overall Complexity Score: {report['overall_complexity_score']:.3f}")
print(f"Improvement Recommendations: {len(report['improvement_recommendations'])}")
for rec in report['improvement_recommendations']:
    print(f"  - {rec}")
```

## Conclusion

Complex terrain navigation represents the pinnacle of humanoid robotics capabilities, requiring the integration of perception, planning, control, and adaptation systems. The ability to navigate stairs, narrow passages, dynamic obstacles, and challenging surface conditions demonstrates the sophisticated engineering required for truly autonomous humanoid robots.

The key to successful complex terrain navigation lies in:
- **Robust perception systems** that can accurately understand the environment
- **Adaptive planning algorithms** that can respond to changing conditions
- **Stable control systems** that maintain balance in challenging situations
- **Real-time processing capabilities** that enable quick responses to unexpected events
- **Integrated safety systems** that protect both the robot and its environment

Modern approaches combine traditional control theory with machine learning and adaptive algorithms to create systems that can handle the wide variety of real-world scenarios that humanoid robots encounter. The evaluation of these systems requires comprehensive metrics that consider success rates, efficiency, safety, and adaptability.

The next section will explore energy efficiency considerations in humanoid locomotion, which becomes increasingly important as robots operate in complex environments for extended periods.