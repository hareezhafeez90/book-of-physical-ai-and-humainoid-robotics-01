# Tactile Sensing and Haptic Feedback

## Introduction: The Sense of Touch in Humanoid Robotics

Tactile sensing provides humanoid robots with the ability to perceive physical contact, pressure, texture, and temperature, enabling them to interact safely and effectively with objects and humans. Unlike vision systems that provide information from a distance, tactile sensors offer direct physical interaction feedback that is essential for manipulation, balance, and safe human-robot interaction. This section explores tactile sensing technologies, haptic feedback systems, and their integration into humanoid robots.

### The Importance of Touch in Humanoid Interaction

Tactile sensing is crucial for humanoid robots because:

- **Safe Interaction**: Detecting contact prevents damage to robot and environment
- **Fine Manipulation**: Tactile feedback enables precise grasping and manipulation
- **Balance Recovery**: Foot tactile sensors help maintain stability
- **Human-Robot Interaction**: Touch-based communication and safety
- **Texture Recognition**: Understanding object properties through touch

## Tactile Sensor Technologies

### Resistive Tactile Sensors

Resistive sensors change resistance when pressure is applied:

```python
class ResistiveTactileSensor:
    def __init__(self, sensor_id, num_taxels=64, max_pressure=100.0):
        self.sensor_id = sensor_id
        self.num_taxels = num_taxels
        self.max_pressure = max_pressure  # Maximum pressure in N/m^2
        self.base_resistance = 10000  # Base resistance in ohms
        self.pressure_sensitivity = 0.1  # Change in resistance per unit pressure
        self.taxel_positions = self._initialize_taxel_positions()

        # Initialize sensor readings
        self.pressure_map = np.zeros(num_taxels)
        self.resistance_map = np.full(num_taxels, self.base_resistance)
        self.temperature = 25.0  # Default temperature in Celsius

    def _initialize_taxel_positions(self):
        """Initialize positions of taxels in 2D grid"""
        # For a 8x8 grid (64 taxels)
        positions = []
        for i in range(8):
            for j in range(8):
                positions.append((i * 0.01, j * 0.01))  # 1cm spacing
        return np.array(positions)

    def update_from_physical_model(self, applied_pressures):
        """Update sensor readings based on applied pressures"""
        if len(applied_pressures) != self.num_taxels:
            raise ValueError("Applied pressures must match number of taxels")

        self.pressure_map = np.clip(applied_pressures, 0, self.max_pressure)

        # Calculate resistance based on pressure (simplified model)
        # Resistance decreases with increasing pressure
        self.resistance_map = self.base_resistance / (1 + self.pressure_sensitivity * self.pressure_map)

        # Convert to sensor readings (e.g., ADC values)
        adc_values = self._resistance_to_adc(self.resistance_map)
        return adc_values

    def _resistance_to_adc(self, resistances):
        """Convert resistance values to ADC readings"""
        # Simple voltage divider model
        v_ref = 3.3  # Reference voltage
        r_pullup = 10000  # Pull-up resistor in ohms
        adc_resolution = 4096  # 12-bit ADC

        # Voltage divider: V_out = V_ref * R_sensor / (R_pullup + R_sensor)
        voltages = v_ref * resistances / (r_pullup + resistances)
        adc_values = (voltages / v_ref) * adc_resolution

        return adc_values.astype(int)

    def get_contact_info(self, threshold=0.5):
        """Get information about contact points above threshold"""
        contact_indices = np.where(self.pressure_map > threshold)[0]
        contact_pressures = self.pressure_map[contact_indices]
        contact_positions = self.taxel_positions[contact_indices]

        if len(contact_indices) == 0:
            return None

        # Calculate center of pressure
        total_force = np.sum(contact_pressures)
        if total_force > 0:
            cop_x = np.sum(contact_positions[:, 0] * contact_pressures) / total_force
            cop_y = np.sum(contact_positions[:, 1] * contact_pressures) / total_force
            center_of_pressure = (cop_x, cop_y)
        else:
            center_of_pressure = None

        return {
            'contact_indices': contact_indices,
            'contact_pressures': contact_pressures,
            'contact_positions': contact_positions,
            'center_of_pressure': center_of_pressure,
            'total_force': total_force
        }

    def detect_slip(self, time_window=0.1):
        """Detect slip based on pressure pattern changes"""
        # This would require temporal analysis
        # For now, return a simplified slip detection
        pressure_changes = np.diff(self.pressure_map) if len(self.pressure_map) > 1 else np.zeros_like(self.pressure_map)
        rapid_changes = np.abs(pressure_changes) > 5.0  # Threshold for rapid change
        return np.any(rapid_changes)
```

### Capacitive Tactile Sensors

Capacitive sensors measure changes in capacitance due to contact:

```python
class CapacitiveTactileSensor:
    def __init__(self, sensor_id, num_taxels=64):
        self.sensor_id = sensor_id
        self.num_taxels = num_taxels
        self.base_capacitance = np.full(num_taxels, 10e-12)  # 10 pF base capacitance
        self.pressure_coefficient = 1e-12  # Capacitance change per unit pressure
        self.frequency = 1e6  # 1 MHz measurement frequency
        self.temperature_coefficient = 1e-15  # Capacitance change per degree Celsius

        # Initialize sensor state
        self.current_capacitance = self.base_capacitance.copy()
        self.pressure_map = np.zeros(num_taxels)
        self.temperature = 25.0
        self.taxel_positions = self._initialize_taxel_positions()

    def _initialize_taxel_positions(self):
        """Initialize taxel positions"""
        positions = []
        for i in range(8):
            for j in range(8):
                positions.append((i * 0.008, j * 0.008))  # 8mm spacing
        return np.array(positions)

    def update_from_contact(self, applied_pressures, temperature=None):
        """Update sensor readings based on applied pressures and temperature"""
        if temperature is not None:
            self.temperature = temperature

        self.pressure_map = np.clip(applied_pressures, 0, 100.0)  # 0-100 N/m^2

        # Calculate capacitance changes due to pressure
        pressure_effect = self.pressure_coefficient * self.pressure_map

        # Calculate temperature effect
        temp_effect = self.temperature_coefficient * (self.temperature - 25.0)

        # Total capacitance
        self.current_capacitance = self.base_capacitance + pressure_effect + temp_effect

        # Convert to measurement values (e.g., frequency shift)
        measurement_values = self._capacitance_to_frequency(self.current_capacitance)
        return measurement_values

    def _capacitance_to_frequency(self, capacitances):
        """Convert capacitance to frequency for measurement"""
        # Simple oscillator model: f = 1 / (2 * pi * sqrt(L * C))
        # For simplicity, assume fixed inductance
        l = 1e-6  # 1 μH inductor
        frequencies = 1 / (2 * np.pi * np.sqrt(l * capacitances))
        return frequencies

    def get_tactile_image(self):
        """Generate tactile image from pressure distribution"""
        # Reshape 1D pressure array to 2D grid
        tactile_image = self.pressure_map.reshape(8, 8)
        return tactile_image

    def detect_object_properties(self):
        """Detect object properties from tactile data"""
        # Calculate contact area
        contact_threshold = 0.1  # Minimum pressure for contact
        contact_map = self.pressure_map > contact_threshold
        contact_area = np.sum(contact_map)

        # Calculate pressure distribution statistics
        active_pressures = self.pressure_map[contact_map]
        if len(active_pressures) > 0:
            avg_pressure = np.mean(active_pressures)
            max_pressure = np.max(active_pressures)
            pressure_variance = np.var(active_pressures)

            # Estimate object compliance (softness)
            # Higher variance may indicate texture or softness
            compliance_estimate = pressure_variance / (max_pressure + 1e-6)
        else:
            avg_pressure = 0
            max_pressure = 0
            pressure_variance = 0
            compliance_estimate = 0

        return {
            'contact_area': contact_area,
            'avg_pressure': avg_pressure,
            'max_pressure': max_pressure,
            'pressure_variance': pressure_variance,
            'compliance_estimate': compliance_estimate
        }

    def estimate_surface_roughness(self, window_size=3):
        """Estimate surface roughness from pressure variations"""
        if self.pressure_map.size < window_size**2:
            return 0

        # Reshape to 2D for spatial analysis
        tactile_2d = self.pressure_map.reshape(8, 8)

        # Calculate local pressure variations
        roughness_measure = 0
        for i in range(0, 8 - window_size + 1, window_size):
            for j in range(0, 8 - window_size + 1, window_size):
                window = tactile_2d[i:i+window_size, j:j+window_size]
                if np.any(window > 0.1):  # Only consider areas with contact
                    local_variance = np.var(window)
                    roughness_measure += local_variance

        return roughness_measure / ((8 // window_size) ** 2 + 1e-6)
```

### Piezoelectric Tactile Sensors

Piezoelectric sensors generate voltage when mechanically stressed:

```python
class PiezoelectricTactileSensor:
    def __init__(self, sensor_id, num_taxels=64):
        self.sensor_id = sensor_id
        self.num_taxels = num_taxels
        self.piezoelectric_coefficient = 2.3e-11  # C/N (for quartz)
        self.capacitance = 100e-12  # 100 pF
        self.resistance = 1e9  # 1 GΩ leakage resistance

        # Initialize sensor state
        self.charge = np.zeros(num_taxels)
        self.voltage = np.zeros(num_taxels)
        self.pressure_history = [np.zeros(num_taxels) for _ in range(10)]  # 10 time steps
        self.time_step = 0.001  # 1ms time step

    def apply_force(self, forces, dt=0.001):
        """Apply forces and update sensor readings"""
        # Generate charge based on applied force
        charge_generated = self.piezoelectric_coefficient * forces

        # Account for charge leakage
        charge_leakage = self.charge * dt / (self.resistance * self.capacitance)
        self.charge = self.charge - charge_leakage + charge_generated

        # Calculate voltage
        self.voltage = self.charge / self.capacitance

        # Update pressure history for dynamic analysis
        self.pressure_history.pop(0)
        self.pressure_history.append(forces.copy())

        return self.voltage

    def detect_dynamic_events(self):
        """Detect dynamic events like impacts or vibrations"""
        # Analyze pressure history for dynamic events
        if len(self.pressure_history) < 3:
            return []

        events = []
        for i in range(self.num_taxels):
            # Calculate pressure derivative (rate of change)
            recent_pressures = [h[i] for h in self.pressure_history[-3:]]
            pressure_rate = np.diff(recent_pressures)

            # Detect impacts (high rate of change)
            if len(pressure_rate) > 0 and abs(pressure_rate[-1]) > 10:  # Threshold
                events.append({
                    'taxel': i,
                    'type': 'impact',
                    'magnitude': abs(pressure_rate[-1]),
                    'timestamp': time.time()
                })

            # Detect vibrations (oscillatory behavior)
            if len(recent_pressures) >= 3:
                # Simple vibration detection using variance
                variance = np.var(recent_pressures)
                if variance > 5:  # Threshold for vibration
                    events.append({
                        'taxel': i,
                        'type': 'vibration',
                        'magnitude': variance,
                        'timestamp': time.time()
                    })

        return events

    def get_force_reconstruction(self):
        """Reconstruct applied forces from sensor readings"""
        # Invert the piezoelectric relationship
        # F = Q / d (where d is piezoelectric coefficient)
        forces = self.charge / self.piezoelectric_coefficient
        return forces
```

## Advanced Tactile Processing

### Tactile Feature Extraction

Extracting meaningful features from tactile data:

```python
class TactileFeatureExtractor:
    def __init__(self):
        self.feature_history = deque(maxlen=50)

    def extract_basic_features(self, tactile_data):
        """Extract basic tactile features"""
        features = {}

        # Statistical features
        features['mean_pressure'] = np.mean(tactile_data)
        features['std_pressure'] = np.std(tactile_data)
        features['max_pressure'] = np.max(tactile_data)
        features['min_pressure'] = np.min(tactile_data)
        features['pressure_range'] = features['max_pressure'] - features['min_pressure']

        # Contact features
        contact_threshold = 0.1
        contact_mask = tactile_data > contact_threshold
        features['contact_area'] = np.sum(contact_mask)
        features['contact_ratio'] = np.sum(contact_mask) / len(tactile_data)

        # Gradient features (for texture detection)
        if tactile_data.ndim == 1:
            # Reshape to 2D if needed
            size = int(np.sqrt(len(tactile_data)))
            tactile_2d = tactile_data.reshape(size, size)
        else:
            tactile_2d = tactile_data

        # Calculate gradients
        grad_x = np.gradient(tactile_2d, axis=1)
        grad_y = np.gradient(tactile_2d, axis=0)
        grad_magnitude = np.sqrt(grad_x**2 + grad_y**2)

        features['avg_gradient'] = np.mean(grad_magnitude)
        features['max_gradient'] = np.max(grad_magnitude)

        return features

    def extract_texture_features(self, tactile_sequence):
        """Extract texture features from tactile sequence"""
        if len(tactile_sequence) < 2:
            return {}

        features = {}

        # Temporal features
        pressure_changes = np.diff(tactile_sequence, axis=0)
        features['avg_temporal_change'] = np.mean(np.abs(pressure_changes))
        features['max_temporal_change'] = np.max(np.abs(pressure_changes))

        # Frequency domain features
        if len(tactile_sequence) > 10:
            # Perform FFT on average pressure over time
            avg_pressure_over_time = np.mean(tactile_sequence, axis=1)
            fft_result = np.fft.fft(avg_pressure_over_time)
            fft_magnitude = np.abs(fft_result)

            # Extract dominant frequencies
            dominant_freq_idx = np.argmax(fft_magnitude[1:len(fft_magnitude)//2]) + 1
            features['dominant_frequency'] = dominant_freq_idx
            features['dominant_freq_magnitude'] = fft_magnitude[dominant_freq_idx]

        # Spatial texture features
        if tactile_sequence[-1].ndim == 2:
            final_map = tactile_sequence[-1]
            # Calculate spatial frequency content using 2D FFT
            fft_2d = np.fft.fft2(final_map)
            fft_2d_magnitude = np.abs(fft_2d)
            features['spatial_frequency_content'] = np.mean(fft_2d_magnitude)

        return features

    def classify_material(self, tactile_features):
        """Classify material based on tactile features"""
        # Simple classification based on pressure distribution
        if tactile_features.get('std_pressure', 0) > 5.0:
            return 'rough', 0.8
        elif tactile_features.get('avg_gradient', 0) > 2.0:
            return 'textured', 0.7
        elif tactile_features.get('contact_ratio', 0) > 0.8:
            return 'soft', 0.6
        else:
            return 'smooth', 0.9
```

### Tactile-Based Object Recognition

Recognizing objects through touch:

```python
class TactileObjectRecognizer:
    def __init__(self):
        self.known_objects = {}
        self.feature_extractor = TactileFeatureExtractor()

    def register_object(self, object_name, tactile_sequence):
        """Register a known object with its tactile signature"""
        # Extract features from the tactile sequence
        features = self.feature_extractor.extract_basic_features(tactile_sequence[-1])
        temporal_features = self.feature_extractor.extract_texture_features(tactile_sequence)

        self.known_objects[object_name] = {
            'static_features': features,
            'temporal_features': temporal_features,
            'tactile_sequence': tactile_sequence
        }

    def recognize_object(self, tactile_sequence, threshold=0.7):
        """Recognize object based on tactile sequence"""
        if len(self.known_objects) == 0:
            return None, 0.0

        query_features = self.feature_extractor.extract_basic_features(tactile_sequence[-1])
        query_temporal = self.feature_extractor.extract_texture_features(tactile_sequence)

        best_match = None
        best_score = 0.0

        for obj_name, obj_data in self.known_objects.items():
            # Compare static features
            static_score = self._compare_features(
                query_features, obj_data['static_features']
            )

            # Compare temporal features
            temporal_score = self._compare_features(
                query_temporal, obj_data['temporal_features'], weights={'dominant_frequency': 2.0}
            )

            # Combined score
            combined_score = 0.6 * static_score + 0.4 * temporal_score

            if combined_score > best_score:
                best_score = combined_score
                best_match = obj_name

        if best_score > threshold:
            return best_match, best_score
        else:
            return None, best_score

    def _compare_features(self, features1, features2, weights=None):
        """Compare two feature dictionaries"""
        if weights is None:
            weights = {}

        scores = []
        for key in features1.keys():
            if key in features2:
                # Normalize the difference
                val1, val2 = features1[key], features2[key]
                max_val = max(abs(val1), abs(val2), 1e-6)
                diff = abs(val1 - val2) / max_val
                similarity = max(0, 1 - diff)

                weight = weights.get(key, 1.0)
                scores.append(similarity * weight)

        if scores:
            return np.mean(scores)
        else:
            return 0.0

    def learn_object_properties(self, object_name, tactile_data):
        """Learn object properties through tactile exploration"""
        properties = {
            'compliance': self._estimate_compliance(tactile_data),
            'roughness': self._estimate_roughness(tactile_data),
            'thermal_conductivity': self._estimate_thermal_props(tactile_data),
            'size': self._estimate_size(tactile_data)
        }

        if object_name in self.known_objects:
            self.known_objects[object_name]['properties'] = properties
        else:
            self.known_objects[object_name] = {'properties': properties}

        return properties

    def _estimate_compliance(self, tactile_data):
        """Estimate object compliance from pressure distribution"""
        # Higher pressure variance may indicate softer material
        if tactile_data.ndim > 1:
            tactile_flat = tactile_data.reshape(-1)
        else:
            tactile_flat = tactile_data

        non_zero_pressures = tactile_flat[tactile_flat > 0.1]
        if len(non_zero_pressures) > 0:
            return np.var(non_zero_pressures) / (np.mean(non_zero_pressures) + 1e-6)
        return 0

    def _estimate_roughness(self, tactile_data):
        """Estimate surface roughness from gradient analysis"""
        if tactile_data.ndim == 1:
            size = int(np.sqrt(len(tactile_data)))
            tactile_2d = tactile_data.reshape(size, size)
        else:
            tactile_2d = tactile_data

        grad_x = np.gradient(tactile_2d, axis=1)
        grad_y = np.gradient(tactile_2d, axis=0)
        grad_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        return np.mean(grad_magnitude)

    def _estimate_thermal_props(self, tactile_data):
        """Estimate thermal properties (requires thermal sensors)"""
        # Placeholder for thermal property estimation
        return 0.5  # Default value

    def _estimate_size(self, tactile_data):
        """Estimate object size from contact area"""
        contact_threshold = 0.1
        contact_map = tactile_data > contact_threshold
        contact_area = np.sum(contact_map)
        return contact_area
```

## Haptic Feedback Systems

### Vibrotactile Feedback

Vibrotactile feedback provides tactile sensations through vibration:

```python
import math

class VibrotactileFeedback:
    def __init__(self, num_actuators=8):
        self.num_actuators = num_actuators
        self.actuator_positions = self._initialize_actuator_positions()
        self.current_signals = np.zeros(num_actuators)
        self.frequency_range = (50, 1000)  # Hz
        self.amplitude_range = (0, 1)  # Normalized

    def _initialize_actuator_positions(self):
        """Initialize actuator positions in 2D space"""
        # For example, arranged in a circle on the palm
        positions = []
        for i in range(self.num_actuators):
            angle = 2 * math.pi * i / self.num_actuators
            x = 0.02 * math.cos(angle)  # 2cm radius
            y = 0.02 * math.sin(angle)
            positions.append((x, y))
        return np.array(positions)

    def generate_vibration_pattern(self, stimulus_type, intensity=0.5, frequency=200, duration=1.0):
        """Generate vibration pattern for different stimuli"""
        time_steps = int(duration * 1000)  # 1ms time steps
        time_vector = np.linspace(0, duration, time_steps)

        if stimulus_type == 'texture':
            # Simulate texture through amplitude modulation
            carrier = np.sin(2 * np.pi * frequency * time_vector)
            envelope = 1 + 0.5 * np.sin(2 * np.pi * 20 * time_vector)  # 20Hz modulation
            signal = intensity * carrier * envelope

        elif stimulus_type == 'slip':
            # Simulate slip detection with increasing frequency
            freq_sweep = np.linspace(frequency, frequency * 2, time_steps)
            signal = intensity * np.sin(2 * np.pi * freq_sweep * time_vector)
            signal = signal * np.linspace(0, 1, time_steps)  # Ramp up

        elif stimulus_type == 'pressure':
            # Simulate pressure with low frequency, high amplitude
            signal = intensity * np.sin(2 * np.pi * 50 * time_vector)

        elif stimulus_type == 'warning':
            # Warning signal - burst pattern
            burst_period = 0.2  # 200ms burst period
            burst_active = (time_vector % burst_period) < (burst_period / 4)
            carrier = np.sin(2 * np.pi * 300 * time_vector)
            signal = intensity * carrier * burst_active.astype(float)

        else:
            # Default: simple sinusoid
            signal = intensity * np.sin(2 * np.pi * frequency * time_vector)

        return signal

    def spatialize_feedback(self, stimulus_signal, contact_position):
        """Spatialize haptic feedback based on contact location"""
        # Calculate distances from contact position to each actuator
        distances = np.sqrt(np.sum((self.actuator_positions - contact_position)**2, axis=1))

        # Calculate attenuation based on distance (inverse square law)
        min_distance = np.min(distances)
        distances = np.maximum(distances, min_distance + 0.001)  # Avoid division by zero
        attenuation = 1 / (1 + distances / 0.01)  # 1cm reference distance

        # Apply spatial attenuation to the signal
        spatialized_signals = np.outer(stimulus_signal, attenuation)
        return spatialized_signals

    def render_haptic_feedback(self, tactile_event, contact_position):
        """Render appropriate haptic feedback for tactile event"""
        if tactile_event['type'] == 'slip':
            signal = self.generate_vibration_pattern('slip', intensity=0.8, duration=0.5)
        elif tactile_event['type'] == 'texture_change':
            signal = self.generate_vibration_pattern('texture', intensity=0.6, duration=0.3)
        elif tactile_event['type'] == 'high_pressure':
            signal = self.generate_vibration_pattern('pressure', intensity=0.9, duration=0.2)
        elif tactile_event['type'] == 'object_edge':
            signal = self.generate_vibration_pattern('warning', intensity=0.7, duration=0.4)
        else:
            signal = self.generate_vibration_pattern('texture', intensity=0.4, duration=0.1)

        # Spatialize the feedback
        spatialized_feedback = self.spatialize_feedback(signal, contact_position)
        return spatialized_feedback
```

### Force Feedback Systems

Force feedback provides resistance or guidance through actuators:

```python
class ForceFeedbackSystem:
    def __init__(self, max_force=50.0, max_torque=10.0):
        self.max_force = max_force  # Maximum force in Newtons
        self.max_torque = max_torque  # Maximum torque in Nm
        self.current_force = np.zeros(3)  # 3D force vector
        self.current_torque = np.zeros(3)  # 3D torque vector
        self.stiffness = 1000  # N/m for virtual fixtures
        self.damping = 50  # Ns/m for damping

    def compute_virtual_fixtures_force(self, current_pos, desired_pos, boundary_type='plane'):
        """Compute force for virtual fixtures"""
        if boundary_type == 'plane':
            # Virtual plane at z = 0.1m
            distance_to_plane = current_pos[2] - 0.1
            if distance_to_plane < 0:  # Penetrating the plane
                force_magnitude = -self.stiffness * distance_to_plane
                force_direction = np.array([0, 0, 1])  # Normal to plane
                return np.clip(force_magnitude, 0, self.max_force) * force_direction
        elif boundary_type == 'sphere':
            # Virtual sphere constraint
            center = np.array([0, 0, 0.2])  # Sphere center
            radius = 0.1  # Sphere radius
            vector_to_center = current_pos - center
            distance = np.linalg.norm(vector_to_center)
            if distance > radius:  # Outside sphere
                force_magnitude = self.stiffness * (distance - radius)
                force_direction = -vector_to_center / distance  # Push back inside
                force = np.clip(force_magnitude, 0, self.max_force) * force_direction
                return force

        return np.zeros(3)

    def compute_guidance_force(self, current_pos, desired_trajectory, lookahead_distance=0.05):
        """Compute guidance force to follow trajectory"""
        # Find closest point on trajectory
        closest_point = self._find_closest_point(current_pos, desired_trajectory)

        # Calculate direction to trajectory
        direction_to_trajectory = closest_point - current_pos
        distance_to_trajectory = np.linalg.norm(direction_to_trajectory)

        if distance_to_trajectory > 0.001:  # Avoid division by zero
            direction_to_trajectory = direction_to_trajectory / distance_to_trajectory

        # Generate guidance force
        guidance_force = self.stiffness * direction_to_trajectory * min(distance_to_trajectory, 0.01)

        return np.clip(guidance_force, -self.max_force, self.max_force)

    def _find_closest_point(self, current_pos, trajectory):
        """Find closest point on trajectory to current position"""
        if len(trajectory) == 0:
            return current_pos

        # Simple approach: find closest point in trajectory
        distances = np.linalg.norm(trajectory - current_pos, axis=1)
        closest_idx = np.argmin(distances)
        return trajectory[closest_idx]

    def compute_damping_force(self, velocity):
        """Compute damping force to reduce oscillations"""
        damping_force = -self.damping * velocity
        return np.clip(damping_force, -self.max_force, self.max_force)

    def compute_contact_stabilization(self, contact_normal, contact_force_magnitude):
        """Compute forces to stabilize contact"""
        # Generate force perpendicular to contact surface
        stabilization_force = self.stiffness * contact_normal * contact_force_magnitude * 0.001
        return np.clip(stabilization_force, -self.max_force, self.max_force)

    def update_haptic_rendering(self, current_pos, current_vel, contact_info):
        """Update haptic rendering based on current state"""
        total_force = np.zeros(3)

        # Add virtual fixtures force
        virtual_fixtures_force = self.compute_virtual_fixtures_force(current_pos, None)
        total_force += virtual_fixtures_force

        # Add damping force
        damping_force = self.compute_damping_force(current_vel)
        total_force += damping_force

        # Add contact stabilization if in contact
        if contact_info and contact_info.get('in_contact', False):
            contact_normal = contact_info.get('normal', np.array([0, 0, 1]))
            contact_force = contact_info.get('force_magnitude', 0)
            stabilization_force = self.compute_contact_stabilization(contact_normal, contact_force)
            total_force += stabilization_force

        # Limit total force
        force_magnitude = np.linalg.norm(total_force)
        if force_magnitude > self.max_force:
            total_force = (total_force / force_magnitude) * self.max_force

        self.current_force = total_force
        return total_force
```

## Tactile-Based Control

### Tactile Feedback in Grasping Control

Using tactile feedback for stable grasping:

```python
class TactileGraspingController:
    def __init__(self):
        self.slip_threshold = 0.5
        self.pressure_threshold = 5.0
        self.slip_history = deque(maxlen=10)
        self.force_history = deque(maxlen=20)
        self.grasp_state = 'searching'  # searching, contacting, grasping, stable
        self.target_force = 10.0  # Target grasp force in Newtons

    def update_grasp_control(self, tactile_data, current_force, dt=0.001):
        """Update grasping control based on tactile feedback"""
        # Process tactile data
        contact_info = self._analyze_contact(tactile_data)
        slip_detected = self._detect_slip(tactile_data)

        # Update control state based on tactile feedback
        if self.grasp_state == 'searching':
            if contact_info['contact_exists']:
                self.grasp_state = 'contacting'
                return self._initial_contact_control(current_force)

        elif self.grasp_state == 'contacting':
            if contact_info['adequate_contact']:
                self.grasp_state = 'grasping'
                return self._grasp_force_control(current_force, contact_info)

        elif self.grasp_state == 'grasping':
            if slip_detected:
                return self._slip_compensation_control(current_force, contact_info)
            elif contact_info['stable_grasp']:
                self.grasp_state = 'stable'
                return self._maintain_grasp_control(current_force)

        elif self.grasp_state == 'stable':
            if slip_detected:
                self.grasp_state = 'grasping'  # Return to grasping mode
                return self._slip_compensation_control(current_force, contact_info)
            else:
                return self._maintain_grasp_control(current_force)

        return 0  # Default: no additional force

    def _analyze_contact(self, tactile_data):
        """Analyze tactile data to determine contact state"""
        # Calculate contact statistics
        contact_threshold = 0.1
        contact_map = tactile_data > contact_threshold
        contact_area = np.sum(contact_map)
        avg_pressure = np.mean(tactile_data[tactile_data > contact_threshold]) if np.any(contact_map) else 0

        # Determine contact state
        contact_exists = contact_area > 5  # At least 5 taxels in contact
        adequate_contact = contact_area > 10 and avg_pressure > 1.0
        stable_grasp = contact_area > 15 and 2.0 < avg_pressure < 15.0

        return {
            'contact_exists': contact_exists,
            'adequate_contact': adequate_contact,
            'stable_grasp': stable_grasp,
            'contact_area': contact_area,
            'avg_pressure': avg_pressure
        }

    def _detect_slip(self, tactile_data):
        """Detect slip using tactile data analysis"""
        # Add current data to history
        self.slip_history.append(tactile_data.copy())

        if len(self.slip_history) < 3:
            return False

        # Analyze temporal changes in tactile pattern
        prev_data = self.slip_history[-2]
        curr_data = self.slip_history[-1]

        # Calculate change in pressure distribution
        pressure_change = np.abs(curr_data - prev_data)
        change_magnitude = np.mean(pressure_change)

        # Detect slip based on rapid changes in tactile pattern
        slip_detected = change_magnitude > self.slip_threshold

        return slip_detected

    def _initial_contact_control(self, current_force):
        """Control strategy for initial contact"""
        # Gentle contact: apply small force to establish contact
        target_force = min(2.0, self.target_force * 0.2)
        force_error = target_force - current_force
        control_output = 5.0 * force_error  # Simple P controller
        return control_output

    def _grasp_force_control(self, current_force, contact_info):
        """Control strategy for establishing grasp"""
        # Gradually increase force to target level
        target_force = self.target_force * 0.7  # 70% of target initially
        force_error = target_force - current_force
        control_output = 3.0 * force_error  # P controller
        return control_output

    def _slip_compensation_control(self, current_force, contact_info):
        """Control strategy when slip is detected"""
        # Rapidly increase force to prevent slip
        target_force = min(self.target_force * 1.5, 50.0)  # 150% of target, max 50N
        force_error = target_force - current_force
        control_output = 10.0 * force_error  # Aggressive P controller
        return max(control_output, 0)  # Only allow positive force increase

    def _maintain_grasp_control(self, current_force):
        """Control strategy for maintaining stable grasp"""
        # Maintain target force with small adjustments
        target_force = self.target_force
        force_error = target_force - current_force
        control_output = 2.0 * force_error  # Gentle P controller
        return control_output

    def get_grasp_quality_metrics(self, tactile_data):
        """Calculate grasp quality metrics from tactile data"""
        contact_info = self._analyze_contact(tactile_data)

        # Force distribution quality (how evenly force is distributed)
        active_taxels = tactile_data[tactile_data > 0.1]
        if len(active_taxels) > 1:
            force_uniformity = 1.0 - np.std(active_taxels) / (np.mean(active_taxels) + 1e-6)
        else:
            force_uniformity = 0

        # Contact coverage (how much of the tactile array is used)
        contact_coverage = contact_info['contact_area'] / len(tactile_data)

        # Stability indicator (based on pressure variance)
        stability = max(0, 1 - contact_info['avg_pressure'] / 20.0)  # Assuming max 20N/cm^2

        return {
            'force_uniformity': max(0, min(1, force_uniformity)),
            'contact_coverage': max(0, min(1, contact_coverage)),
            'stability': max(0, min(1, stability)),
            'grasp_quality': (force_uniformity + contact_coverage + stability) / 3.0
        }
```

## Integration with ROS 2

### ROS 2 Tactile Sensing Node

Integrating tactile sensing with ROS 2 for system-wide haptic feedback:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped, Vector3
from std_msgs.msg import Header

class TactileSensorNode(Node):
    def __init__(self):
        super().__init__('tactile_sensor_node')

        # Create publishers
        self.tactile_pub = self.create_publisher(
            Float64MultiArray,
            '/tactile_sensors/raw_data',
            10
        )

        self.contact_pub = self.create_publisher(
            WrenchStamped,
            '/contact_force',
            10
        )

        self.haptic_feedback_pub = self.create_publisher(
            Float64MultiArray,
            '/haptic_feedback',
            10
        )

        # Create subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Initialize tactile sensors
        self.tactile_sensors = {
            'left_hand': ResistiveTactileSensor('left_hand', num_taxels=64),
            'right_hand': ResistiveTactileSensor('right_hand', num_taxels=64),
            'left_foot': ResistiveTactileSensor('left_foot', num_taxels=32),
            'right_foot': ResistiveTactileSensor('right_foot', num_taxels=32)
        }

        self.haptic_renderer = VibrotactileFeedback(num_actuators=8)
        self.grasp_controller = TactileGraspingController()

        # Timer for tactile data publishing
        self.tactile_timer = self.create_timer(0.01, self.publish_tactile_data)  # 100Hz

        self.get_logger().info('Tactile sensor node initialized')

    def joint_state_callback(self, msg):
        """Process joint state information for tactile interpretation"""
        # Update tactile sensor state based on joint positions and forces
        # This would involve forward kinematics and force mapping
        pass

    def publish_tactile_data(self):
        """Publish tactile sensor data"""
        for sensor_name, sensor in self.tactile_sensors.items():
            # Simulate tactile data (in real system, this would come from hardware)
            simulated_pressures = np.random.random(sensor.num_taxels) * 10  # 0-10 N/m^2
            adc_values = sensor.update_from_physical_model(simulated_pressures)

            # Create and publish message
            tactile_msg = Float64MultiArray()
            tactile_msg.data = adc_values.tolist()

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = sensor_name
            tactile_msg.layout.data_offset = 0

            self.tactile_pub.publish(tactile_msg)

            # Process contact information
            contact_info = sensor.get_contact_info()
            if contact_info and contact_info['total_force'] > 1.0:  # 1N threshold
                # Publish contact force
                contact_msg = WrenchStamped()
                contact_msg.header = header
                contact_msg.wrench.force.x = 0.0
                contact_msg.wrench.force.y = 0.0
                contact_msg.wrench.force.z = float(contact_info['total_force'])
                contact_msg.wrench.torque = Vector3(x=0.0, y=0.0, z=0.0)

                self.contact_pub.publish(contact_msg)

                # Generate haptic feedback based on contact
                self.generate_haptic_feedback(sensor_name, contact_info)

    def generate_haptic_feedback(self, sensor_name, contact_info):
        """Generate haptic feedback based on tactile input"""
        if contact_info['total_force'] > 5.0:  # Strong contact
            haptic_signal = self.haptic_renderer.generate_vibration_pattern(
                'pressure', intensity=0.8, frequency=150, duration=0.1
            )
        elif contact_info.get('center_of_pressure') is not None:
            # Spatialized feedback based on contact location
            cop = contact_info['center_of_pressure']
            haptic_signal = self.haptic_renderer.render_haptic_feedback(
                {'type': 'texture'}, cop
            )
        else:
            return  # No significant contact

        # Publish haptic feedback
        feedback_msg = Float64MultiArray()
        feedback_msg.data = haptic_signal[-1].tolist()  # Use last time step
        self.haptic_feedback_pub.publish(feedback_msg)

    def process_grasp_control(self, hand_sensor_name):
        """Process grasp control for a hand"""
        hand_sensor = self.tactile_sensors[hand_sensor_name]

        # Get current tactile data
        current_pressures = hand_sensor.pressure_map

        # Update grasp controller
        additional_force = self.grasp_controller.update_grasp_control(
            current_pressures,
            current_force=10.0  # Placeholder for actual force measurement
        )

        # Calculate grasp quality
        quality_metrics = self.grasp_controller.get_grasp_quality_metrics(current_pressures)

        self.get_logger().info(
            f'Grasp quality for {hand_sensor_name}: {quality_metrics["grasp_quality"]:.2f}, '
            f'State: {self.grasp_controller.grasp_state}, '
            f'Additional force needed: {additional_force:.2f}N'
        )

def main(args=None):
    rclpy.init(args=args)
    tactile_node = TactileSensorNode()

    try:
        rclpy.spin(tactile_node)
    except KeyboardInterrupt:
        pass
    finally:
        tactile_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

Tactile sensing and haptic feedback systems provide humanoid robots with the crucial ability to perceive and interact through touch. These systems enable safe manipulation, balance maintenance, and natural human-robot interaction. The integration of various tactile sensing technologies with advanced processing algorithms and haptic feedback creates rich sensory experiences that enhance robot capabilities.

The next section will explore inertial and proprioceptive sensing systems that provide information about the robot's internal state and motion, completing the comprehensive perception system for humanoid robots.