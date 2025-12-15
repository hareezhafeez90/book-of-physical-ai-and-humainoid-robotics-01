# Gait Generation and Pattern Formation

## Introduction: Creating Natural Walking Patterns

Gait generation is the process of creating rhythmic, coordinated movement patterns that enable humanoid robots to walk efficiently and stably. Unlike simple joint interpolation, gait generation involves the coordination of multiple body segments, timing of foot contacts, and synchronization of balance control systems. This section explores the biological inspiration, mathematical models, and implementation strategies for creating natural, efficient walking patterns.

### Biological Inspiration and Central Pattern Generators

Human walking is controlled by Central Pattern Generators (CPGs) in the spinal cord that produce rhythmic motor patterns. Robotic gait generation often mimics these biological systems to create stable, adaptive walking patterns.

## Mathematical Models for Gait Generation

### Central Pattern Generator (CPG) Models

CPGs are neural networks that generate rhythmic patterns without rhythmic input:

```python
import numpy as np
import matplotlib.pyplot as plt

class CentralPatternGenerator:
    def __init__(self, dt=0.005):
        self.dt = dt
        self.oscillators = {}
        self.connections = {}
        self.outputs = {}

    def add_oscillator(self, name, frequency=1.0, phase=0.0, amplitude=1.0):
        """Add a neural oscillator to the CPG"""
        self.oscillators[name] = {
            'frequency': frequency,
            'phase': phase,
            'amplitude': amplitude,
            'state': 0.0,
            'velocity': 0.0
        }

    def connect_oscillators(self, source, target, weight=1.0, phase_offset=0.0):
        """Connect two oscillators with specified weight and phase offset"""
        connection_key = (source, target)
        self.connections[connection_key] = {
            'weight': weight,
            'phase_offset': phase_offset
        }

    def update(self, external_input=None):
        """Update all oscillators based on connections and inputs"""
        if external_input is None:
            external_input = {}

        new_states = {}
        new_velocities = {}

        for name, osc in self.oscillators.items():
            # Calculate input from connected oscillators
            coupling_input = 0.0
            for (src, tgt), conn in self.connections.items():
                if tgt == name:
                    src_osc = self.oscillators[src]
                    coupling_input += conn['weight'] * np.sin(
                        src_osc['state'] - osc['state'] + conn['phase_offset']
                    )

            # Add external input if provided
            ext_input = external_input.get(name, 0.0)

            # Van der Pol oscillator dynamics (simplified)
            # dx/dt = y
            # dy/dt = mu*(1-x^2)*y - x + coupling_input
            mu = 2.0  # Nonlinearity parameter
            x = osc['state']
            y = osc['velocity']

            dx_dt = y
            dy_dt = mu * (1 - x**2) * y - x + coupling_input + ext_input

            new_states[name] = x + dx_dt * self.dt
            new_velocities[name] = y + dy_dt * self.dt

            # Calculate output (rhythmic signal)
            self.outputs[name] = osc['amplitude'] * np.sin(new_states[name] + osc['phase'])

        # Update states
        for name in self.oscillators:
            self.oscillators[name]['state'] = new_states[name]
            self.oscillators[name]['velocity'] = new_velocities[name]

    def get_output(self, name):
        """Get the current output of an oscillator"""
        return self.outputs.get(name, 0.0)

    def get_all_outputs(self):
        """Get all oscillator outputs"""
        return self.outputs.copy()

# Example: Simple walking CPG
cpg = CentralPatternGenerator(dt=0.005)

# Add oscillators for left and right legs
cpg.add_oscillator('left_hip', frequency=1.0, amplitude=0.5)
cpg.add_oscillator('right_hip', frequency=1.0, phase=np.pi, amplitude=0.5)  # 180° phase difference
cpg.add_oscillator('left_knee', frequency=1.0, amplitude=0.3)
cpg.add_oscillator('right_knee', frequency=1.0, phase=np.pi, amplitude=0.3)

# Connect oscillators (ipsilateral and contralateral coupling)
cpg.connect_oscillators('left_hip', 'right_hip', weight=0.5, phase_offset=np.pi)
cpg.connect_oscillators('right_hip', 'left_hip', weight=0.5, phase_offset=np.pi)
cpg.connect_oscillators('left_hip', 'left_knee', weight=0.8, phase_offset=0.0)
cpg.connect_oscillators('right_hip', 'right_knee', weight=0.8, phase_offset=0.0)

# Simulate for one walking cycle
time_points = []
outputs = []
for t in np.arange(0, 2.0, 0.005):
    cpg.update()
    time_points.append(t)
    outputs.append(cpg.get_all_outputs().copy())

# Convert to arrays for plotting
time_points = np.array(time_points)
left_hip = np.array([out['left_hip'] for out in outputs])
right_hip = np.array([out['right_hip'] for out in outputs])
left_knee = np.array([out['left_knee'] for out in outputs])
right_knee = np.array([out['right_knee'] for out in outputs])

print(f"CPG simulation completed: {len(time_points)} time steps")
print(f"Left hip amplitude: {np.max(np.abs(left_hip)):.3f}")
print(f"Right hip amplitude: {np.max(np.abs(right_hip)):.3f}")
```

### Coupled Oscillator Networks

More complex CPGs using networks of coupled oscillators:

```python
class CoupledOscillatorNetwork:
    def __init__(self, num_oscillators=8, dt=0.005):
        self.num_oscillators = num_oscillators
        self.dt = dt
        self.phases = np.random.random(num_oscillators) * 2 * np.pi
        self.frequencies = np.ones(num_oscillators)  # Base frequencies
        self.amplitudes = np.ones(num_oscillators)
        self.coupling_matrix = np.zeros((num_oscillators, num_oscillators))
        self.biases = np.zeros(num_oscillators)

    def set_coupling(self, i, j, strength):
        """Set coupling strength between oscillators i and j"""
        self.coupling_matrix[i, j] = strength
        self.coupling_matrix[j, i] = strength  # Symmetric coupling

    def set_chain_coupling(self, longitudinal_strength=1.0, transverse_strength=0.5):
        """Set coupling in a chain pattern (useful for limb coordination)"""
        for i in range(self.num_oscillators):
            # Longitudinal coupling (along the chain)
            if i < self.num_oscillators - 1:
                self.set_coupling(i, i+1, longitudinal_strength)

            # Transverse coupling (for bilateral coordination)
            if i < self.num_oscillators // 2:
                self.set_coupling(i, i + self.num_oscillators // 2, transverse_strength)

    def set_gait_pattern(self, pattern='walking'):
        """Set coupling pattern for specific gait"""
        if pattern == 'walking':
            # Walking pattern: alternating left/right legs
            self.biases[:self.num_oscillators//2] = 0  # Left side
            self.biases[self.num_oscillators//2:] = np.pi  # Right side (180° phase difference)

            # Set different frequencies for different joints
            self.frequencies[0] = 1.0  # Left hip
            self.frequencies[1] = 1.0  # Left knee
            self.frequencies[2] = 1.0  # Left ankle
            self.frequencies[4] = 1.0  # Right hip
            self.frequencies[5] = 1.0  # Right knee
            self.frequencies[6] = 1.0  # Right ankle

        elif pattern == 'running':
            # Running pattern: higher frequency, different phase relationships
            self.frequencies *= 2.0  # Double the frequency
            self.biases[:self.num_oscillators//2] = 0
            self.biases[self.num_oscillators//2:] = np.pi * 0.5  # 90° phase difference for running

    def update(self, external_inputs=None):
        """Update all oscillators using Kuramoto model"""
        if external_inputs is None:
            external_inputs = np.zeros(self.num_oscillators)

        # Kuramoto model: dθ_i/dt = ω_i + (1/N) * Σ K_ij * sin(θ_j - θ_i) + input_i
        dtheta = self.frequencies.copy()

        for i in range(self.num_oscillators):
            coupling_sum = 0.0
            for j in range(self.num_oscillators):
                if i != j:
                    coupling_sum += self.coupling_matrix[i, j] * np.sin(self.phases[j] - self.phases[i])

            dtheta[i] += (1.0 / self.num_oscillators) * coupling_sum + external_inputs[i]

        # Update phases
        self.phases += dtheta * self.dt

        # Keep phases in [-π, π]
        self.phases = np.arctan2(np.sin(self.phases), np.cos(self.phases))

    def get_outputs(self):
        """Get rhythmic outputs from all oscillators"""
        return self.amplitudes * np.sin(self.phases)

    def modulate_frequency(self, oscillator_idx, modulation_factor):
        """Modulate frequency of specific oscillator"""
        self.frequencies[oscillator_idx] *= modulation_factor

    def reset(self):
        """Reset oscillator phases"""
        self.phases = np.random.random(self.num_oscillators) * 2 * np.pi

# Example: Complex CPG for walking
cpg_network = CoupledOscillatorNetwork(num_oscillators=8, dt=0.005)

# Set up for walking
cpg_network.set_chain_coupling(longitudinal_strength=0.8, transverse_strength=0.6)
cpg_network.set_gait_pattern('walking')

# Simulate walking pattern
walking_time = np.arange(0, 3.0, 0.005)
walking_outputs = []

for t in walking_time:
    cpg_network.update()
    outputs = cpg_network.get_outputs()
    walking_outputs.append(outputs.copy())

walking_outputs = np.array(walking_outputs)

print(f"Complex CPG simulation: {len(walking_time)} time steps")
print(f"Oscillator outputs range: {np.min(walking_outputs):.3f} to {np.max(walking_outputs):.3f}")
```

## Trajectory Generation Methods

### Fourier Series-Based Trajectory Generation

Using Fourier series to generate smooth periodic trajectories:

```python
class FourierTrajectoryGenerator:
    def __init__(self, num_harmonics=5):
        self.num_harmonics = num_harmonics
        self.coefficients = {}

    def fit_trajectory(self, time_points, trajectory_values, joint_name):
        """Fit Fourier series to a trajectory"""
        # Calculate Fourier coefficients
        T = time_points[-1] - time_points[0]  # Period
        omega = 2 * np.pi / T  # Fundamental frequency

        # Calculate coefficients: a0, an, bn
        a0 = np.trapz(trajectory_values, time_points) / T

        an = []
        bn = []
        for n in range(1, self.num_harmonics + 1):
            an_n = (2/T) * np.trapz(trajectory_values * np.cos(n * omega * time_points), time_points)
            bn_n = (2/T) * np.trapz(trajectory_values * np.sin(n * omega * time_points), time_points)
            an.append(an_n)
            bn.append(bn_n)

        self.coefficients[joint_name] = {
            'a0': a0,
            'an': np.array(an),
            'bn': np.array(bn),
            'omega': omega,
            'T': T
        }

    def generate_trajectory(self, joint_name, time_points):
        """Generate trajectory using stored Fourier coefficients"""
        if joint_name not in self.coefficients:
            return np.zeros_like(time_points)

        coeffs = self.coefficients[joint_name]
        trajectory = np.full_like(time_points, coeffs['a0'])

        for n in range(1, len(coeffs['an']) + 1):
            trajectory += coeffs['an'][n-1] * np.cos(n * coeffs['omega'] * time_points)
            trajectory += coeffs['bn'][n-1] * np.sin(n * coeffs['omega'] * time_points)

        return trajectory

    def generate_multiple_trajectories(self, joint_trajectories, duration=2.0, dt=0.005):
        """Generate multiple joint trajectories simultaneously"""
        time_points = np.arange(0, duration, dt)

        # Fit each trajectory
        for joint_name, trajectory in joint_trajectories.items():
            # Create a sample trajectory (in real use, this would come from demonstration)
            sample_time = np.linspace(0, duration, len(trajectory))
            self.fit_trajectory(sample_time, trajectory, joint_name)

        # Generate all trajectories
        generated_trajectories = {}
        for joint_name in joint_trajectories.keys():
            generated_trajectories[joint_name] = self.generate_trajectory(joint_name, time_points)

        return generated_trajectories, time_points

    def add_terrain_adaptation(self, joint_name, terrain_height, adaptation_gain=0.1):
        """Add terrain adaptation to trajectory"""
        if joint_name not in self.coefficients:
            return

        # Modify coefficients based on terrain height
        self.coefficients[joint_name]['a0'] += adaptation_gain * terrain_height

    def get_trajectory_derivative(self, joint_name, time_points):
        """Get velocity (derivative) of trajectory"""
        if joint_name not in self.coefficients:
            return np.zeros_like(time_points)

        coeffs = self.coefficients[joint_name]
        velocity = np.zeros_like(time_points)

        for n in range(1, len(coeffs['an']) + 1):
            velocity -= n * coeffs['omega'] * coeffs['an'][n-1] * np.sin(n * coeffs['omega'] * time_points)
            velocity += n * coeffs['omega'] * coeffs['bn'][n-1] * np.cos(n * coeffs['omega'] * time_points)

        return velocity

# Example: Generate walking trajectories using Fourier series
fourier_gen = FourierTrajectoryGenerator(num_harmonics=8)

# Create sample trajectories (these would come from human demonstration or optimization)
t_demo = np.linspace(0, 2.0, 100)
left_hip_demo = 0.2 * np.sin(2 * np.pi * t_demo) + 0.1 * np.sin(4 * np.pi * t_demo)
right_hip_demo = 0.2 * np.sin(2 * np.pi * t_demo + np.pi) + 0.1 * np.sin(4 * np.pi * t_demo + np.pi)
left_knee_demo = 0.3 * np.sin(2 * np.pi * t_demo + np.pi/2) + 0.1 * np.cos(4 * np.pi * t_demo)
right_knee_demo = 0.3 * np.sin(2 * np.pi * t_demo + 3*np.pi/2) + 0.1 * np.cos(4 * np.pi * t_demo + np.pi)

# Fit trajectories
joint_demos = {
    'left_hip': left_hip_demo,
    'right_hip': right_hip_demo,
    'left_knee': left_knee_demo,
    'right_knee': right_knee_demo
}

generated_trajectories, time_points = fourier_gen.generate_multiple_trajectories(joint_demos)

print(f"Fourier trajectory generation completed")
print(f"Generated {len(generated_trajectories)} joint trajectories")
print(f"Time duration: {time_points[-1]:.2f} seconds")
```

### Bézier Curve-Based Trajectory Generation

Using Bézier curves for smooth trajectory generation:

```python
class BezierTrajectoryGenerator:
    def __init__(self, control_points_per_segment=4):
        self.control_points_per_segment = control_points_per_segment
        self.trajectories = {}

    def bernstein_polynomial(self, i, n, t):
        """Calculate Bernstein polynomial of degree n"""
        from math import factorial
        return (factorial(n) / (factorial(i) * factorial(n - i))) * (t**i) * ((1-t)**(n-i))

    def bezier_curve(self, control_points, t_values):
        """Generate Bézier curve from control points"""
        n = len(control_points) - 1
        curve_points = np.zeros((len(t_values), len(control_points[0])))

        for j, t in enumerate(t_values):
            point = np.zeros(len(control_points[0]))
            for i, cp in enumerate(control_points):
                point += cp * self.bernstein_polynomial(i, n, t)
            curve_points[j] = point

        return curve_points

    def generate_joint_trajectory(self, control_points, duration=1.0, dt=0.005):
        """Generate smooth trajectory using Bézier curves"""
        t_values = np.arange(0, duration, dt)
        t_normalized = t_values / duration  # Normalize to [0, 1]

        # Ensure we have enough control points
        if len(control_points) < 2:
            raise ValueError("Need at least 2 control points")

        # Generate trajectory
        trajectory = self.bezier_curve(control_points, t_normalized)
        return trajectory, t_values

    def generate_3d_trajectory(self, start_pos, end_pos, mid_height=0.05, duration=0.8, dt=0.005):
        """Generate 3D trajectory for foot movement (e.g., for stepping)"""
        # Define control points for foot trajectory
        # Start: lift foot, move forward, place foot
        control_points = np.array([
            start_pos,  # Start position
            [start_pos[0] + (end_pos[0] - start_pos[0]) * 0.25,
             start_pos[1] + (end_pos[1] - start_pos[1]) * 0.25,
             start_pos[2] + mid_height * 0.5],  # Mid point (lifted)
            [start_pos[0] + (end_pos[0] - start_pos[0]) * 0.75,
             start_pos[1] + (end_pos[1] - start_pos[1]) * 0.75,
             start_pos[2] + mid_height * 0.5],  # Another mid point
            end_pos  # End position
        ])

        return self.generate_joint_trajectory(control_points, duration, dt)

    def generate_swing_trajectory(self, start_foot_pos, target_foot_pos, step_height=0.05):
        """Generate swing phase trajectory for foot"""
        # Generate trajectory for swing foot
        trajectory, times = self.generate_3d_trajectory(
            start_foot_pos, target_foot_pos,
            mid_height=step_height,
            duration=0.4  # Swing phase duration
        )
        return trajectory, times

    def generate_com_trajectory(self, start_pos, target_pos, step_height=0.02):
        """Generate CoM trajectory with smooth transition"""
        # CoM follows a smooth path between steps
        control_points = np.array([
            start_pos,
            [start_pos[0] + (target_pos[0] - start_pos[0]) * 0.3,
             start_pos[1] + (target_pos[1] - start_pos[1]) * 0.3,
             start_pos[2] + step_height],  # Mid point (slightly higher)
            [start_pos[0] + (target_pos[0] - start_pos[0]) * 0.7,
             start_pos[1] + (target_pos[1] - start_pos[1]) * 0.7,
             start_pos[2] + step_height],  # Another mid point
            target_pos
        ])

        return self.generate_joint_trajectory(control_points, duration=0.8)

    def generate_ankle_trajectory(self, foot_pos, body_pos, step_height=0.01):
        """Generate ankle trajectory relative to body"""
        # Ankle trajectory follows foot but with less vertical movement
        control_points = np.array([
            [foot_pos[0], foot_pos[1], body_pos[2] - 0.05],  # Start (ankle position)
            [foot_pos[0] + 0.05, foot_pos[1], body_pos[2] - 0.04],  # Mid swing
            [foot_pos[0] + 0.15, foot_pos[1], body_pos[2] - 0.04],  # Mid stance
            [foot_pos[0] + 0.2, foot_pos[1], body_pos[2] - 0.05]  # End (push off)
        ])

        return self.generate_joint_trajectory(control_points, duration=0.8)

# Example: Generate walking trajectories using Bézier curves
bezier_gen = BezierTrajectoryGenerator()

# Generate swing trajectory for right foot
start_pos = np.array([0.0, -0.1, 0.0])  # Right foot starting position
target_pos = np.array([0.3, -0.1, 0.0])  # Target position (step forward)

foot_trajectory, foot_times = bezier_gen.generate_swing_trajectory(start_pos, target_pos, step_height=0.05)

# Generate CoM trajectory
com_start = np.array([0.0, 0.0, 0.85])
com_target = np.array([0.15, 0.0, 0.85])  # Move CoM forward
com_trajectory, com_times = bezier_gen.generate_com_trajectory(com_start, com_target)

print(f"Bezier trajectory generation completed")
print(f"Foot trajectory: {len(foot_trajectory)} points")
print(f"CoM trajectory: {len(com_trajectory)} points")
print(f"Foot movement: {start_pos} to {target_pos}")
print(f"CoM movement: {com_start} to {com_target}")
```

## Pattern Formation Algorithms

### Phase Oscillator Networks

Advanced pattern formation using networks of phase oscillators:

```python
class PhaseOscillatorNetwork:
    def __init__(self, num_oscillators=12, dt=0.005):
        self.num_oscillators = num_oscillators
        self.dt = dt
        self.phases = np.random.random(num_oscillators) * 2 * np.pi
        self.natural_frequencies = 2 * np.pi * np.ones(num_oscillators)  # 1 Hz
        self.coupling_strengths = np.zeros((num_oscillators, num_oscillators))
        self.phase_couplings = np.zeros((num_oscillators, num_oscillators))

    def set_coupling_topology(self, topology='chain'):
        """Set coupling topology for the network"""
        if topology == 'chain':
            # Chain topology: each oscillator connects to neighbors
            coupling_strength = 1.0
            for i in range(self.num_oscillators):
                if i > 0:
                    self.coupling_strengths[i, i-1] = coupling_strength
                    self.phase_couplings[i, i-1] = 0.0  # In-phase coupling
                if i < self.num_oscillators - 1:
                    self.coupling_strengths[i, i+1] = coupling_strength
                    self.phase_couplings[i, i+1] = 0.0  # In-phase coupling

        elif topology == 'ring':
            # Ring topology: each connects to neighbors, with wraparound
            coupling_strength = 1.0
            for i in range(self.num_oscillators):
                left_idx = (i - 1) % self.num_oscillators
                right_idx = (i + 1) % self.num_oscillators
                self.coupling_strengths[i, left_idx] = coupling_strength
                self.coupling_strengths[i, right_idx] = coupling_strength
                self.phase_couplings[i, left_idx] = 0.0
                self.phase_couplings[i, right_idx] = 0.0

        elif topology == 'bilateral':
            # Bilateral symmetry: left/right pairs with contralateral coupling
            for i in range(self.num_oscillators // 2):
                left_idx = i
                right_idx = i + self.num_oscillators // 2

                # Ipsilateral coupling
                if i > 0:
                    self.coupling_strengths[left_idx, left_idx-1] = 0.8
                    self.coupling_strengths[right_idx, right_idx-1] = 0.8
                    self.coupling_strengths[left_idx-1, left_idx] = 0.8
                    self.coupling_strengths[right_idx-1, right_idx] = 0.8

                # Contralateral coupling (180° phase difference)
                self.coupling_strengths[left_idx, right_idx] = 0.6
                self.coupling_strengths[right_idx, left_idx] = 0.6
                self.phase_couplings[left_idx, right_idx] = np.pi  # 180°
                self.phase_couplings[right_idx, left_idx] = np.pi  # 180°

    def update(self, external_inputs=None):
        """Update all oscillators using Kuramoto model with phase shifts"""
        if external_inputs is None:
            external_inputs = np.zeros(self.num_oscillators)

        dphases = self.natural_frequencies.copy()

        # Add coupling effects
        for i in range(self.num_oscillators):
            for j in range(self.num_oscillators):
                if i != j and self.coupling_strengths[i, j] > 0:
                    phase_diff = self.phases[j] - self.phases[i] + self.phase_couplings[i, j]
                    dphases[i] += self.coupling_strengths[i, j] * np.sin(phase_diff)

        # Add external inputs
        dphases += external_inputs

        # Update phases
        self.phases += dphases * self.dt

        # Keep phases in [-π, π]
        self.phases = np.arctan2(np.sin(self.phases), np.cos(self.phases))

    def get_rhythmic_outputs(self, amplitudes=None, offsets=None):
        """Get rhythmic outputs from oscillators"""
        if amplitudes is None:
            amplitudes = np.ones(self.num_oscillators)
        if offsets is None:
            offsets = np.zeros(self.num_oscillators)

        return amplitudes * np.sin(self.phases) + offsets

    def entrain_to_rhythm(self, target_frequency, oscillator_idx, coupling_strength=2.0):
        """Entrain specific oscillator to external rhythm"""
        # This would modify the natural frequency or add external forcing
        pass

    def synchronize_oscillators(self, group_indices, target_phase_diff=0.0):
        """Synchronize a group of oscillators to specific phase relationships"""
        for i in group_indices:
            for j in group_indices:
                if i != j:
                    self.coupling_strengths[i, j] = 1.5
                    self.phase_couplings[i, j] = target_phase_diff

    def generate_gait_pattern(self, gait_type='walking'):
        """Generate specific gait pattern"""
        if gait_type == 'walking':
            # Walking: alternating left/right legs
            left_indices = list(range(self.num_oscillators // 2))
            right_indices = list(range(self.num_oscillators // 2, self.num_oscillators))

            # Set contralateral phase difference
            for left_idx in left_indices:
                right_idx = left_idx + self.num_oscillators // 2
                self.phase_couplings[left_idx, right_idx] = np.pi  # 180°
                self.phase_couplings[right_idx, left_idx] = np.pi  # 180°

        elif gait_type == 'trotting':
            # For quadrupedal pattern (if applicable)
            pass

# Example: Phase oscillator network for gait generation
phase_net = PhaseOscillatorNetwork(num_oscillators=8, dt=0.005)

# Set up bilateral symmetry topology
phase_net.set_coupling_topology('bilateral')

# Generate walking pattern
time_points = np.arange(0, 2.0, 0.005)
network_outputs = []

for t in time_points:
    phase_net.update()
    outputs = phase_net.get_rhythmic_outputs()
    network_outputs.append(outputs.copy())

network_outputs = np.array(network_outputs)

print(f"Phase oscillator network simulation completed")
print(f"Network outputs shape: {network_outputs.shape}")
print(f"Left side (first 4 oscillators) mean phase: {np.mean(network_outputs[:, :4]):.3f}")
print(f"Right side (last 4 oscillators) mean phase: {np.mean(network_outputs[:, 4:]):.3f}")
```

### Neural Network-Based Pattern Generation

Using neural networks to learn and generate gait patterns:

```python
class NeuralPatternGenerator:
    def __init__(self, input_size=10, hidden_size=64, output_size=12):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        self.weights = {
            'input_hidden': np.random.randn(input_size, hidden_size) * 0.1,
            'hidden_output': np.random.randn(hidden_size, output_size) * 0.1,
            'hidden_bias': np.zeros(hidden_size),
            'output_bias': np.zeros(output_size)
        }
        self.activation = lambda x: np.tanh(x)  # Activation function
        self.activation_derivative = lambda x: 1 - np.tanh(x)**2

    def forward(self, inputs):
        """Forward pass through the network"""
        self.last_inputs = inputs
        self.hidden = self.activation(np.dot(inputs, self.weights['input_hidden']) + self.weights['hidden_bias'])
        self.outputs = np.dot(self.hidden, self.weights['hidden_output']) + self.weights['output_bias']
        return self.outputs

    def train(self, input_sequences, target_sequences, learning_rate=0.01, epochs=1000):
        """Train the network to generate target sequences"""
        for epoch in range(epochs):
            total_loss = 0
            for inputs, targets in zip(input_sequences, target_sequences):
                # Forward pass
                outputs = self.forward(inputs)

                # Calculate loss
                loss = np.mean((outputs - targets)**2)
                total_loss += loss

                # Backward pass
                output_error = outputs - targets
                hidden_error = np.dot(output_error, self.weights['hidden_output'].T) * self.activation_derivative(self.hidden)

                # Update weights
                self.weights['hidden_output'] -= learning_rate * np.outer(self.hidden, output_error)
                self.weights['output_bias'] -= learning_rate * output_error
                self.weights['input_hidden'] -= learning_rate * np.outer(self.last_inputs, hidden_error)
                self.weights['hidden_bias'] -= learning_rate * hidden_error

            if epoch % 100 == 0:
                print(f"Epoch {epoch}, Average Loss: {total_loss/len(input_sequences):.6f}")

    def generate_sequence(self, initial_input, sequence_length=100):
        """Generate a sequence of outputs"""
        sequence = []
        current_input = initial_input.copy()

        for _ in range(sequence_length):
            output = self.forward(current_input)
            sequence.append(output.copy())

            # Update input for next step (feedback)
            # This is a simple example - in practice, you might use a recurrent structure
            current_input = np.concatenate([output[:self.input_size//2], current_input[self.input_size//2:]])

        return np.array(sequence)

    def add_recurrent_connection(self, recurrent_weights=None):
        """Add recurrent connections for temporal dependencies"""
        if recurrent_weights is None:
            self.recurrent_weights = np.random.randn(self.hidden_size, self.hidden_size) * 0.01
        else:
            self.recurrent_weights = recurrent_weights
        self.last_hidden = np.zeros(self.hidden_size)

    def recurrent_forward(self, inputs):
        """Forward pass with recurrent connections"""
        self.last_inputs = inputs
        recurrent_input = np.dot(self.last_hidden, self.recurrent_weights)
        self.hidden = self.activation(
            np.dot(inputs, self.weights['input_hidden']) +
            recurrent_input +
            self.weights['hidden_bias']
        )
        self.last_hidden = self.hidden.copy()
        self.outputs = np.dot(self.hidden, self.weights['hidden_output']) + self.weights['output_bias']
        return self.outputs

    def generate_oscillatory_output(self, frequency=1.0, amplitude=1.0, duration=2.0, dt=0.005):
        """Generate oscillatory output for rhythmic movement"""
        time_points = np.arange(0, duration, dt)

        # Create oscillatory input pattern
        inputs = []
        for t in time_points:
            input_vec = np.zeros(self.input_size)
            for i in range(self.input_size):
                input_vec[i] = amplitude * np.sin(2 * np.pi * frequency * t + i * 0.5)
            inputs.append(input_vec)

        outputs = []
        for input_vec in inputs:
            output = self.forward(input_vec)
            outputs.append(output)

        return np.array(outputs), time_points

# Example: Neural network-based pattern generation
nn_gen = NeuralPatternGenerator(input_size=8, hidden_size=32, output_size=12)

# Create training data (simplified - in practice this would come from demonstrations)
input_sequences = []
target_sequences = []

for _ in range(50):  # 50 training examples
    # Create a rhythmic input pattern
    inputs = np.random.randn(50, 8) * 0.1  # 50 time steps, 8 inputs
    # Create corresponding rhythmic output pattern
    targets = np.zeros((50, 12))
    for i in range(50):
        t = i * 0.02  # 50Hz
        for j in range(12):
            targets[i, j] = np.sin(2 * np.pi * 1.0 * t + j * 0.5) + 0.1 * np.random.randn()

    input_sequences.append(inputs)
    target_sequences.append(targets)

# Train the network
print("Training neural pattern generator...")
nn_gen.train(input_sequences, target_sequences, learning_rate=0.01, epochs=500)

# Generate a sequence
initial_input = np.zeros(8)
generated_sequence = nn_gen.generate_sequence(initial_input, sequence_length=100)

print(f"Neural network generation completed")
print(f"Generated sequence shape: {generated_sequence.shape}")
print(f"Output range: {np.min(generated_sequence):.3f} to {np.max(generated_sequence):.3f}")
```

## Gait Adaptation and Learning

### Adaptive Gait Control

Implementing systems that adapt gait patterns to different conditions:

```python
class AdaptiveGaitController:
    def __init__(self, base_gait_params=None):
        if base_gait_params is None:
            base_gait_params = {
                'step_length': 0.3,
                'step_width': 0.2,
                'step_time': 0.8,
                'swing_height': 0.05,
                'com_height': 0.85
            }
        self.base_params = base_gait_params
        self.current_params = base_gait_params.copy()
        self.adaptation_history = []
        self.performance_metrics = {}

    def assess_terrain(self, sensor_data):
        """Assess terrain characteristics from sensor data"""
        terrain_info = {
            'slope': 0.0,
            'roughness': 0.0,
            'obstacles': [],
            'surface_type': 'flat'
        }

        # Analyze sensor data to determine terrain properties
        # This would process data from cameras, LIDAR, force sensors, etc.
        if 'height_map' in sensor_data:
            height_var = np.var(sensor_data['height_map'])
            terrain_info['roughness'] = min(0.1, height_var)

        if 'slope_estimate' in sensor_data:
            terrain_info['slope'] = sensor_data['slope_estimate']

        return terrain_info

    def adapt_gait_parameters(self, terrain_info, current_state=None):
        """Adapt gait parameters based on terrain and current state"""
        adapted_params = self.base_params.copy()

        # Adjust for slope
        slope_factor = 1.0 - 0.3 * abs(terrain_info['slope'])
        adapted_params['step_length'] *= slope_factor
        adapted_params['step_time'] /= slope_factor if slope_factor > 0.5 else 0.5

        # Adjust for roughness
        roughness_factor = 1.0 - 0.2 * terrain_info['roughness']
        adapted_params['step_length'] *= roughness_factor
        adapted_params['swing_height'] = max(0.08, adapted_params['swing_height'] * (1.0 + terrain_info['roughness']))

        # Adjust for obstacles
        if terrain_info['obstacles']:
            # Reduce step length to step over obstacles safely
            adapted_params['step_length'] *= 0.7
            adapted_params['swing_height'] = max(0.1, adapted_params['swing_height'] * 2)

        # Store adaptation
        adaptation_record = {
            'timestamp': time.time(),
            'terrain_info': terrain_info,
            'old_params': self.current_params.copy(),
            'new_params': adapted_params.copy()
        }
        self.adaptation_history.append(adaptation_record)

        self.current_params = adapted_params
        return adapted_params

    def generate_adaptive_trajectory(self, terrain_info, duration=2.0):
        """Generate trajectory adapted to current terrain"""
        # Get adapted parameters
        params = self.adapt_gait_parameters(terrain_info)

        # Use the trajectory generators with adapted parameters
        bezier_gen = BezierTrajectoryGenerator()

        # Generate foot trajectory
        start_pos = np.array([0.0, -params['step_width']/2, 0.0])
        target_pos = np.array([params['step_length'], -params['step_width']/2, 0.0])

        foot_trajectory, foot_times = bezier_gen.generate_swing_trajectory(
            start_pos, target_pos, step_height=params['swing_height']
        )

        # Generate CoM trajectory
        com_start = np.array([0.0, 0.0, params['com_height']])
        com_target = np.array([params['step_length']/2, 0.0, params['com_height']])

        com_trajectory, com_times = bezier_gen.generate_com_trajectory(
            com_start, com_target, step_height=0.02
        )

        return {
            'foot_trajectory': foot_trajectory,
            'com_trajectory': com_trajectory,
            'gait_params': params,
            'time_points': foot_times
        }

    def evaluate_performance(self, actual_trajectory, desired_trajectory):
        """Evaluate gait performance"""
        # Calculate tracking error
        error = np.mean(np.linalg.norm(actual_trajectory - desired_trajectory, axis=1))

        # Calculate energy efficiency (simplified)
        velocity = np.gradient(actual_trajectory, axis=0)
        energy = np.mean(np.sum(velocity**2, axis=1))

        # Calculate stability metrics
        stability = self.calculate_stability(actual_trajectory)

        performance = {
            'tracking_error': error,
            'energy_efficiency': 1.0 / (1.0 + energy),  # Higher is better
            'stability': stability,
            'overall_score': (error * 0.4 + (1.0 / (1.0 + energy)) * 0.3 + stability * 0.3)
        }

        self.performance_metrics = performance
        return performance

    def calculate_stability(self, trajectory):
        """Calculate stability metric from trajectory"""
        # Calculate Zero-Moment Point (ZMP) stability
        # This is a simplified calculation
        com_positions = trajectory[:, :2]  # X, Y positions
        com_velocities = np.gradient(com_positions, axis=0)
        com_accelerations = np.gradient(com_velocities, axis=0)

        # Assume constant CoM height for ZMP calculation
        com_height = 0.85
        gravity = 9.81

        zmp_x = com_positions[:, 0] - (com_height / gravity) * com_accelerations[:, 0]
        zmp_y = com_positions[:, 1] - (com_height / gravity) * com_accelerations[:, 1]

        # Stability is better when ZMP stays within bounds
        stability = 1.0 / (1.0 + np.mean(np.abs(zmp_x)) + np.mean(np.abs(zmp_y)))
        return stability

    def learn_from_experience(self, performance_feedback):
        """Update gait parameters based on performance feedback"""
        # Simple learning algorithm - adjust parameters to improve performance
        if performance_feedback['tracking_error'] > 0.05:  # Threshold for poor performance
            # Reduce step length for better tracking
            self.base_params['step_length'] *= 0.95
        elif performance_feedback['overall_score'] > 0.8:  # Good performance
            # Can try longer steps
            self.base_params['step_length'] = min(0.4, self.base_params['step_length'] * 1.01)

        # Store performance for learning
        self.performance_history = getattr(self, 'performance_history', [])
        self.performance_history.append(performance_feedback)

# Example: Adaptive gait control
adaptive_controller = AdaptiveGaitController()

# Simulate different terrains
terrains = [
    {'slope': 0.0, 'roughness': 0.01, 'obstacles': [], 'surface_type': 'flat'},
    {'slope': 0.1, 'roughness': 0.02, 'obstacles': [], 'surface_type': 'sloped'},
    {'slope': 0.0, 'roughness': 0.08, 'obstacles': [], 'surface_type': 'rough'}
]

for i, terrain in enumerate(terrains):
    print(f"\nTerrain {i+1}: {terrain['surface_type']}")

    # Adapt gait parameters
    adapted_params = adaptive_controller.adapt_gait_parameters(terrain)
    print(f"  Step length: {adapted_params['step_length']:.3f} m")
    print(f"  Swing height: {adapted_params['swing_height']:.3f} m")
    print(f"  Step time: {adapted_params['step_time']:.3f} s")

    # Generate trajectory for this terrain
    trajectory_data = adaptive_controller.generate_adaptive_trajectory(terrain)
    print(f"  Generated {len(trajectory_data['foot_trajectory'])} trajectory points")

print(f"\nAdaptation history: {len(adaptive_controller.adaptation_history)} changes")
```

## Rhythmic Movement Primitives

### Dynamic Movement Primitives (DMPs) for Gait

Using DMPs to generate and adapt rhythmic movements:

```python
class RhythmicMovementPrimitive:
    def __init__(self, n_bfs=10, dt=0.005):
        self.n_bfs = n_bfs  # Number of basis functions
        self.dt = dt
        self.weights = np.zeros(n_bfs)
        self.centers = np.linspace(0, 1, n_bfs)
        self.widths = np.ones(n_bfs) * (1.0 / n_bfs) * 6.0  # Overlapping basis functions
        self.phase_step = 0.05  # Phase increment per time step
        self.current_phase = 0.0
        self.initial_value = 0.0
        self.goal = 1.0
        self.last_time = 0.0

    def basis_functions(self, x):
        """Calculate basis functions at phase x"""
        # Gaussian basis functions
        psi = np.exp(-self.widths * (x - self.centers)**2)
        # Normalize
        psi_sum = np.sum(psi)
        if psi_sum > 0:
            psi = psi / psi_sum
        return psi

    def canonical_phase_system(self, t, t0, t1):
        """Canonical system for phase evolution"""
        # For rhythmic DMPs, phase evolves periodically
        period = t1 - t0
        phase = ((t - t0) % period) / period
        return phase

    def forcing_term(self, phase):
        """Calculate forcing term at given phase"""
        psi = self.basis_functions(phase)
        f = np.dot(psi, self.weights)
        return f

    def train(self, trajectory, times):
        """Train the DMP to reproduce the given trajectory"""
        # Calculate desired forcing term
        # y_ddot = f + coupling_term (simplified)
        trajectory_vel = np.gradient(trajectory, times)
        trajectory_acc = np.gradient(trajectory_vel, times)

        # For rhythmic DMP, we want to learn the periodic pattern
        # Discretize the trajectory into phase bins
        phases = np.linspace(0, 1, len(trajectory))
        target_forces = trajectory_acc  # Simplified - in practice, this involves the dynamics

        # Solve for weights using linear regression
        A = np.zeros((len(trajectory), self.n_bfs))
        for i, phase in enumerate(phases):
            A[i, :] = self.basis_functions(phase)

        # Add regularization to avoid overfitting
        reg_matrix = 1e-6 * np.eye(self.n_bfs)
        self.weights = np.linalg.solve(A.T @ A + reg_matrix, A.T @ target_forces)

    def generate_trajectory(self, duration=2.0, initial_value=0.0, goal=1.0):
        """Generate trajectory using the trained DMP"""
        self.initial_value = initial_value
        self.goal = goal

        time_points = np.arange(0, duration, self.dt)
        trajectory = []
        velocities = []
        accelerations = []

        # Reset phase
        self.current_phase = 0.0

        for t in time_points:
            # Update phase (rhythmic)
            self.current_phase = (self.current_phase + self.phase_step) % (2 * np.pi)
            normalized_phase = (self.current_phase + np.pi) / (2 * np.pi)  # Normalize to [0,1]

            # Calculate forcing term
            f = self.forcing_term(normalized_phase)

            # Integrate dynamics (simplified rhythmic oscillator)
            # In a real implementation, this would follow the DMP dynamics
            if len(velocities) == 0:
                vel = 0.0
                pos = initial_value
            else:
                acc = f  # Simplified acceleration
                vel = velocities[-1] + acc * self.dt
                pos = trajectory[-1] + vel * self.dt

            trajectory.append(pos)
            velocities.append(vel)
            accelerations.append(f)

        return np.array(trajectory), time_points

    def adapt_to_frequency(self, new_frequency):
        """Adapt the DMP to a new frequency"""
        # Adjust phase step to match new frequency
        old_frequency = 1.0 / (2 * np.pi / self.phase_step)  # Original frequency
        frequency_ratio = new_frequency / old_frequency
        self.phase_step *= frequency_ratio

    def modulate_amplitude(self, amplitude_factor):
        """Modulate the amplitude of the generated pattern"""
        self.weights *= amplitude_factor

    def combine_primitives(self, other_primitive, combination_method='additive'):
        """Combine with another DMP primitive"""
        if combination_method == 'additive':
            combined_weights = self.weights + other_primitive.weights
        elif combination_method == 'weighted':
            # Weighted combination
            combined_weights = 0.5 * self.weights + 0.5 * other_primitive.weights
        else:
            combined_weights = self.weights.copy()

        combined_dmp = RhythmicMovementPrimitive(self.n_bfs, self.dt)
        combined_dmp.weights = combined_weights
        combined_dmp.centers = self.centers.copy()
        combined_dmp.widths = self.widths.copy()
        return combined_dmp

# Example: Rhythmic DMP for walking
rhythmic_dmp = RhythmicMovementPrimitive(n_bfs=15, dt=0.005)

# Create a sample rhythmic trajectory (e.g., hip angle during walking)
time_demo = np.linspace(0, 2.0, 400)
hip_trajectory = 0.2 * np.sin(2 * np.pi * time_demo) + 0.1 * np.sin(4 * np.pi * time_demo)

# Train the DMP to reproduce this pattern
rhythmic_dmp.train(hip_trajectory, time_demo)

# Generate new trajectory using the trained DMP
generated_trajectory, gen_times = rhythmic_dmp.generate_trajectory(duration=2.0, initial_value=0.0, goal=0.0)

print(f"Rhythmic DMP training completed")
print(f"Original trajectory: {len(hip_trajectory)} points, range {np.min(hip_trajectory):.3f} to {np.max(hip_trajectory):.3f}")
print(f"Generated trajectory: {len(generated_trajectory)} points, range {np.min(generated_trajectory):.3f} to {np.max(generated_trajectory):.3f}")

# Adapt to different frequency
rhythmic_dmp.adapt_to_frequency(1.5)  # 1.5 Hz instead of 1 Hz
fast_trajectory, fast_times = rhythmic_dmp.generate_trajectory(duration=2.0)

print(f"Fast trajectory range: {np.min(fast_trajectory):.3f} to {np.max(fast_trajectory):.3f}")
```

## Integration with Control Systems

### Whole-Body Gait Controller

Integrating gait generation with whole-body control:

```python
class WholeBodyGaitController:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.cpg = CentralPatternGenerator(dt=0.005)
        self.trajectory_generator = FourierTrajectoryGenerator()
        self.adaptive_controller = AdaptiveGaitController()
        self.dmp = RhythmicMovementPrimitive(n_bfs=10)

        # Initialize CPG network for walking
        self.setup_cpg_network()

        # Joint mapping
        self.joint_mapping = {
            'left_hip': 0,
            'left_knee': 1,
            'left_ankle': 2,
            'right_hip': 3,
            'right_knee': 4,
            'right_ankle': 5,
            'left_shoulder': 6,
            'right_shoulder': 7
        }

        # Current state
        self.current_joint_positions = np.zeros(len(self.joint_mapping))
        self.current_joint_velocities = np.zeros(len(self.joint_mapping))
        self.com_position = np.array([0.0, 0.0, 0.85])
        self.com_velocity = np.array([0.0, 0.0, 0.0])

    def setup_cpg_network(self):
        """Setup CPG network for coordinated walking"""
        # Add oscillators for major joints
        joints = ['left_hip', 'right_hip', 'left_knee', 'right_knee',
                 'left_ankle', 'right_ankle', 'left_shoulder', 'right_shoulder']

        frequencies = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.5, 0.5]  # Arms slower
        amplitudes = [0.5, 0.5, 0.3, 0.3, 0.2, 0.2, 0.3, 0.3]

        for joint, freq, amp in zip(joints, frequencies, amplitudes):
            self.cpg.add_oscillator(joint, frequency=freq, amplitude=amp)

        # Set up coupling for walking pattern
        self.cpg.connect_oscillators('left_hip', 'right_hip', weight=0.6, phase_offset=np.pi)
        self.cpg.connect_oscillators('left_knee', 'right_knee', weight=0.6, phase_offset=np.pi)
        self.cpg.connect_oscillators('left_ankle', 'right_ankle', weight=0.5, phase_offset=np.pi)

        # Connect hip to knee and ankle on same side
        self.cpg.connect_oscillators('left_hip', 'left_knee', weight=0.8, phase_offset=0.0)
        self.cpg.connect_oscillators('left_hip', 'left_ankle', weight=0.4, phase_offset=0.5)
        self.cpg.connect_oscillators('right_hip', 'right_knee', weight=0.8, phase_offset=0.0)
        self.cpg.connect_oscillators('right_hip', 'right_ankle', weight=0.4, phase_offset=0.5)

    def generate_joint_commands(self, desired_speed, terrain_info, dt=0.005):
        """Generate joint commands for walking at desired speed"""
        # Update CPG
        self.cpg.update()
        cpg_outputs = self.cpg.get_all_outputs()

        # Get adaptive gait parameters
        gait_params = self.adaptive_controller.adapt_gait_parameters(terrain_info)

        # Generate joint commands based on CPG outputs and gait parameters
        joint_commands = np.zeros(len(self.joint_mapping))

        for joint_name, output in cpg_outputs.items():
            if joint_name in self.joint_mapping:
                joint_idx = self.joint_mapping[joint_name]

                # Scale output based on gait parameters
                base_command = output * gait_params.get('step_length', 0.3) * 2.0

                # Add speed modulation
                speed_factor = min(1.5, max(0.5, desired_speed / 1.0))
                joint_commands[joint_idx] = base_command * speed_factor

        # Add arm swing for balance (opposite to leg swing)
        joint_commands[self.joint_mapping['left_shoulder']] = -cpg_outputs['right_hip'] * 0.5
        joint_commands[self.joint_mapping['right_shoulder']] = -cpg_outputs['left_hip'] * 0.5

        return joint_commands

    def compute_ik_for_trajectory(self, trajectory_points, target_joints):
        """Compute inverse kinematics for desired trajectory points"""
        # This would implement inverse kinematics
        # For this example, return a simplified solution
        ik_solutions = []

        for point in trajectory_points:
            # Simplified IK - in practice, use full kinematic model
            solution = np.zeros(len(target_joints))
            # Example: map x,y,z position to joint angles
            solution[0] = np.arctan2(point[1], point[0])  # Hip yaw
            solution[1] = np.arctan2(point[2], np.sqrt(point[0]**2 + point[1]**2))  # Hip pitch
            solution[2] = 0.5 * solution[1]  # Knee follows hip

            ik_solutions.append(solution)

        return np.array(ik_solutions)

    def integrate_balance_control(self, joint_commands, com_error, zmp_error):
        """Integrate balance control with gait generation"""
        # Add balance corrections to joint commands
        balance_corrections = np.zeros_like(joint_commands)

        # Ankle strategy for small CoM errors
        if np.linalg.norm(com_error) < 0.05:
            balance_corrections[self.joint_mapping['left_ankle']] = -com_error[0] * 10
            balance_corrections[self.joint_mapping['right_ankle']] = -com_error[0] * 10
            balance_corrections[self.joint_mapping['left_ankle'] + 1] = -com_error[1] * 5  # Roll
            balance_corrections[self.joint_mapping['right_ankle'] + 1] = -com_error[1] * 5  # Roll
        else:
            # Stepping strategy for larger errors
            print("Balance recovery needed - stepping strategy")

        # Apply corrections
        corrected_commands = joint_commands + balance_corrections

        return corrected_commands

    def update_state(self, sensor_data):
        """Update internal state from sensor data"""
        # This would process actual sensor data
        # For simulation, we'll update positions based on commands
        pass

    def get_gait_status(self):
        """Get current gait status"""
        return {
            'walking_speed': np.linalg.norm(self.com_velocity[:2]),
            'step_frequency': 1.0 / self.adaptive_controller.current_params['step_time'],
            'energy_consumption': np.sum(np.abs(self.current_joint_velocities)),
            'stability_margin': self.calculate_stability_margin()
        }

    def calculate_stability_margin(self):
        """Calculate stability margin based on current state"""
        # Simplified stability calculation
        # In practice, this would use ZMP, capture point, or other stability metrics
        com_pos = self.com_position
        foot_positions = self.get_foot_positions()

        # Calculate distance to nearest support foot
        min_distance = float('inf')
        for foot_pos in foot_positions:
            distance = np.linalg.norm(com_pos[:2] - foot_pos[:2])
            min_distance = min(min_distance, distance)

        # Stability margin is distance to support polygon
        return min_distance

    def get_foot_positions(self):
        """Get current foot positions (simplified)"""
        # This would use forward kinematics in practice
        # For simulation, return approximate positions
        left_foot = np.array([self.com_position[0], self.com_position[1] + 0.1, 0.0])
        right_foot = np.array([self.com_position[0], self.com_position[1] - 0.1, 0.0])
        return [left_foot, right_foot]

# Example: Whole-body gait controller
# This would be connected to an actual robot model in practice
class SimpleRobotModel:
    def __init__(self):
        self.joint_names = ['left_hip', 'left_knee', 'left_ankle', 'right_hip', 'right_knee', 'right_ankle']
        self.mass = 70.0
        self.height = 1.7

robot_model = SimpleRobotModel()
whole_body_controller = WholeBodyGaitController(robot_model)

# Simulate walking
terrain_info = {'slope': 0.0, 'roughness': 0.01, 'obstacles': [], 'surface_type': 'flat'}
desired_speed = 0.8  # m/s

# Generate commands for one control cycle
joint_commands = whole_body_controller.generate_joint_commands(desired_speed, terrain_info)
print(f"Generated {len(joint_commands)} joint commands")
print(f"Sample commands: {joint_commands[:3]}")

# Simulate balance integration
com_error = np.array([0.02, -0.01, 0.0])  # Small CoM error
zmp_error = np.array([0.01, 0.005, 0.0])

corrected_commands = whole_body_controller.integrate_balance_control(
    joint_commands, com_error, zmp_error
)
print(f"Balance-corrected commands: {corrected_commands[:3]}")

# Get gait status
status = whole_body_controller.get_gait_status()
print(f"Gait status - Speed: {status['walking_speed']:.2f} m/s, "
      f"Stability: {status['stability_margin']:.3f} m")
```

## Conclusion

Gait generation and pattern formation represent a sophisticated approach to creating natural, adaptive walking behaviors in humanoid robots. The integration of biological inspiration through Central Pattern Generators, mathematical models like Fourier series and Bézier curves, and learning algorithms enables robots to generate stable, efficient, and adaptable walking patterns.

The key to successful gait generation lies in the coordination of multiple systems: rhythmic pattern generation, trajectory planning, adaptation to environmental conditions, and integration with balance control systems. Modern approaches combine these elements in whole-body controllers that can adapt to different terrains, speeds, and perturbations while maintaining stable locomotion.

The next section will explore terrain adaptation and footstep planning, building on these gait generation foundations to enable robots to navigate complex environments effectively.