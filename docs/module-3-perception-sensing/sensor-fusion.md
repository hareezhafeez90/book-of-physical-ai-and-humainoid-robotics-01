# Sensor Fusion and State Estimation

## Introduction: Combining Multiple Sensory Inputs

Sensor fusion is the process of combining data from multiple sensors to achieve more accurate, reliable, and robust perception than would be possible with any single sensor. In humanoid robotics, sensor fusion is critical because no single sensor modality can provide complete information about the robot's state and environment. This section explores the mathematical foundations, algorithms, and practical implementations of sensor fusion for humanoid robots.

### The Need for Sensor Fusion

Humanoid robots operate in complex environments where:

- **No single sensor is perfect**: Each sensor has limitations in accuracy, range, or reliability
- **Complementary information**: Different sensors provide different types of information
- **Redundancy for safety**: Multiple sensors provide backup in case of sensor failure
- **Enhanced performance**: Combined sensors can achieve better accuracy than individual sensors

## Mathematical Foundations

### Probability and Uncertainty

Sensor fusion relies on probabilistic models to handle uncertainty:

```python
import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt

class ProbabilityDistribution:
    def __init__(self, mean, covariance):
        self.mean = np.array(mean)
        self.covariance = np.array(covariance)
        self.dimension = len(mean)

    def pdf(self, x):
        """Probability density function"""
        return multivariate_normal.pdf(x, self.mean, self.covariance)

    def sample(self, n_samples=1):
        """Draw samples from the distribution"""
        return multivariate_normal.rvs(self.mean, self.covariance, n_samples)

    def add_noise(self, noise_covariance):
        """Add noise to the distribution"""
        new_covariance = self.covariance + noise_covariance
        return ProbabilityDistribution(self.mean, new_covariance)

    def combine_with(self, other):
        """Combine two Gaussian distributions (sensor fusion)"""
        # For two Gaussians with means μ1, μ2 and covariances Σ1, Σ2
        # Combined mean: μ = Σ2*(Σ1+Σ2)^(-1)*μ1 + Σ1*(Σ1+Σ2)^(-1)*μ2
        # Combined covariance: Σ = Σ1*Σ2*(Σ1+Σ2)^(-1)

        sigma1_inv = np.linalg.inv(self.covariance)
        sigma2_inv = np.linalg.inv(other.covariance)

        # Combined covariance
        combined_cov = np.linalg.inv(sigma1_inv + sigma2_inv)

        # Combined mean
        combined_mean = combined_cov @ (sigma1_inv @ self.mean + sigma2_inv @ other.mean)

        return ProbabilityDistribution(combined_mean, combined_cov)

# Example: Fusing two position estimates
pos1 = ProbabilityDistribution([1.0, 0.0], [[0.1, 0], [0, 0.1]])  # High confidence
pos2 = ProbabilityDistribution([1.2, 0.1], [[0.3, 0], [0, 0.3]])  # Lower confidence

fused_pos = pos1.combine_with(pos2)
print(f"Fused position: {fused_pos.mean}")
print(f"Fused covariance: {fused_pos.covariance}")
```

### Kalman Filtering Fundamentals

The Kalman filter is the optimal linear estimator for systems with Gaussian noise:

```python
class KalmanFilter:
    def __init__(self, state_dim, measurement_dim):
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim

        # State: [x, y, vx, vy] (position and velocity)
        self.state = np.zeros(state_dim)
        self.covariance = np.eye(state_dim) * 1000  # Initial uncertainty

        # Process model (how state evolves over time)
        self.F = np.eye(state_dim)  # Will be updated in predict step

        # Measurement model (how state relates to measurements)
        self.H = np.zeros((measurement_dim, state_dim))
        if measurement_dim == 2 and state_dim >= 2:
            # Measure only position (first 2 elements)
            self.H[0, 0] = 1  # x position
            self.H[1, 1] = 1  # y position

        # Process noise covariance
        self.Q = np.eye(state_dim) * 0.1

        # Measurement noise covariance
        self.R = np.eye(measurement_dim) * 1.0

    def predict(self, dt):
        """Prediction step: predict next state"""
        # Update process model for motion
        # For constant velocity model: x(k+1) = x(k) + v(k)*dt
        if self.state_dim >= 4:  # [x, y, vx, vy]
            self.F = np.array([
                [1, 0, dt, 0],    # x = x + vx*dt
                [0, 1, 0, dt],    # y = y + vy*dt
                [0, 0, 1, 0],     # vx = vx (constant)
                [0, 0, 0, 1]      # vy = vy (constant)
            ])

        # Predict state
        self.state = self.F @ self.state

        # Predict covariance
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q

    def update(self, measurement):
        """Update step: incorporate new measurement"""
        # Innovation (measurement residual)
        innovation = measurement - self.H @ self.state

        # Innovation covariance
        innovation_cov = self.H @ self.covariance @ self.H.T + self.R

        # Kalman gain
        K = self.covariance @ self.H.T @ np.linalg.inv(innovation_cov)

        # Update state
        self.state = self.state + K @ innovation

        # Update covariance
        self.covariance = (np.eye(self.state_dim) - K @ self.H) @ self.covariance

    def get_state(self):
        """Get current state estimate"""
        return self.state.copy()

    def get_covariance(self):
        """Get current covariance estimate"""
        return self.covariance.copy()

# Example usage
kf = KalmanFilter(state_dim=4, measurement_dim=2)  # [x, y, vx, vy] with position measurements

# Simulate tracking
true_positions = []
measurements = []
estimates = []

for t in range(100):
    dt = 0.1
    kf.predict(dt)

    # Simulate true position (with constant velocity)
    true_pos = np.array([t * 0.1, t * 0.05])  # Moving with constant velocity
    noise = np.random.normal(0, 0.1, 2)  # Measurement noise
    measurement = true_pos + noise

    kf.update(measurement)

    true_positions.append(true_pos)
    measurements.append(measurement)
    estimates.append(kf.get_state()[:2])  # Only position part

true_positions = np.array(true_positions)
measurements = np.array(measurements)
estimates = np.array(estimates)

print(f"Final position estimate: {estimates[-1]}")
print(f"Final position uncertainty: {np.sqrt(kf.get_covariance()[0,0])}")
```

## Extended Kalman Filter (EKF)

For nonlinear systems, the Extended Kalman Filter linearizes around the current state:

```python
class ExtendedKalmanFilter:
    def __init__(self, state_dim, measurement_dim):
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim
        self.state = np.zeros(state_dim)
        self.covariance = np.eye(state_dim) * 1000
        self.Q = np.eye(state_dim) * 0.1  # Process noise
        self.R = np.eye(measurement_dim) * 1.0  # Measurement noise

    def motion_model(self, state, dt):
        """Nonlinear motion model"""
        # For example, constant turn rate and velocity model
        x, y, theta, v = state
        new_x = x + v * np.cos(theta) * dt
        new_y = y + v * np.sin(theta) * dt
        new_theta = theta  # Assuming no angular velocity for simplicity
        new_v = v  # Constant velocity
        return np.array([new_x, new_y, new_theta, new_v])

    def motion_jacobian(self, state, dt):
        """Jacobian of motion model"""
        x, y, theta, v = state
        F = np.eye(self.state_dim)
        F[0, 2] = -v * np.sin(theta) * dt  # dx/dtheta
        F[0, 3] = np.cos(theta) * dt      # dx/dv
        F[1, 2] = v * np.cos(theta) * dt  # dy/dtheta
        F[1, 3] = np.sin(theta) * dt      # dy/dv
        return F

    def measurement_model(self, state):
        """Nonlinear measurement model"""
        # Measure position (x, y)
        return state[:2]

    def measurement_jacobian(self, state):
        """Jacobian of measurement model"""
        H = np.zeros((self.measurement_dim, self.state_dim))
        H[0, 0] = 1  # dx/dx
        H[1, 1] = 1  # dy/dy
        return H

    def predict(self, dt):
        """Prediction step for EKF"""
        # Linearize motion model around current state
        F = self.motion_jacobian(self.state, dt)

        # Predict state
        self.state = self.motion_model(self.state, dt)

        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update(self, measurement):
        """Update step for EKF"""
        # Linearize measurement model around current state
        H = self.measurement_jacobian(self.state)

        # Innovation
        predicted_measurement = self.measurement_model(self.state)
        innovation = measurement - predicted_measurement

        # Innovation covariance
        innovation_cov = H @ self.covariance @ H.T + self.R

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(innovation_cov)

        # Update state
        self.state = self.state + K @ innovation

        # Update covariance
        self.covariance = (np.eye(self.state_dim) - K @ H) @ self.covariance

# Example: Tracking with EKF
ekf = ExtendedKalmanFilter(state_dim=4, measurement_dim=2)  # [x, y, theta, v]
ekf.state = np.array([0, 0, 0, 1])  # Start at origin, facing right, speed 1

# Simulate
for t in range(50):
    dt = 0.1
    ekf.predict(dt)

    # Simulate measurement
    true_pos = ekf.motion_model(ekf.state, 0)[:2]  # Get true position
    measurement = true_pos + np.random.normal(0, 0.1, 2)

    ekf.update(measurement)

print(f"EKF final state: {ekf.state}")
print(f"EKF final covariance diagonal: {np.diag(ekf.covariance)}")
```

## Particle Filtering

For highly nonlinear and non-Gaussian systems, particle filters provide a more general solution:

```python
class ParticleFilter:
    def __init__(self, state_dim, num_particles=1000):
        self.state_dim = state_dim
        self.num_particles = num_particles
        self.particles = np.random.normal(0, 1, (num_particles, state_dim))
        self.weights = np.ones(num_particles) / num_particles

    def predict(self, control_input, dt):
        """Predict step: propagate particles forward"""
        # Add process noise and apply motion model
        process_noise = np.random.normal(0, 0.1, (self.num_particles, self.state_dim))

        # Simple motion model: add control input with some dynamics
        for i in range(self.num_particles):
            # Example: simple integrator model
            self.particles[i] += control_input * dt + process_noise[i]

    def update(self, measurement, measurement_noise_std=0.5):
        """Update step: reweight particles based on measurement"""
        # Calculate likelihood of each particle given measurement
        for i in range(self.num_particles):
            # Assume measurement is of first two state dimensions
            predicted_measurement = self.particles[i][:2]
            measurement_diff = measurement - predicted_measurement
            # Calculate likelihood (Gaussian)
            likelihood = np.exp(-0.5 * np.sum((measurement_diff / measurement_noise_std)**2))
            self.weights[i] *= likelihood

        # Normalize weights
        self.weights += 1e-300  # Avoid numerical issues
        self.weights /= np.sum(self.weights)

    def resample(self):
        """Resample particles based on weights"""
        # Systematic resampling
        indices = []
        cumulative_sum = np.cumsum(self.weights)
        start = np.random.uniform(0, 1/self.num_particles)
        i, j = 0, 0
        while i < self.num_particles:
            if start + i / self.num_particles < cumulative_sum[j]:
                indices.append(j)
                i += 1
            else:
                j += 1

        # Resample particles and reset weights
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def estimate(self):
        """Get state estimate as weighted average of particles"""
        return np.average(self.particles, axis=0, weights=self.weights)

    def get_covariance(self):
        """Get covariance estimate from particles"""
        mean = self.estimate()
        diff = self.particles - mean
        cov = np.zeros((self.state_dim, self.state_dim))
        for i in range(self.num_particles):
            cov += self.weights[i] * np.outer(diff[i], diff[i])
        return cov

# Example: Particle filter for tracking
pf = ParticleFilter(state_dim=4, num_particles=1000)  # [x, y, vx, vy]
control = np.array([0.1, 0.05, 0, 0])  # Constant velocity control

for t in range(20):
    dt = 0.1
    pf.predict(control, dt)

    # Simulate measurement
    true_pos = np.array([t * 0.1, t * 0.05])
    measurement = true_pos + np.random.normal(0, 0.1, 2)
    pf.update(measurement)

    if t % 5 == 0:  # Resample every 5 steps
        pf.resample()

estimate = pf.estimate()
print(f"Particle filter estimate: {estimate[:2]}")
print(f"True position: {true_pos}")
```

## Multi-Sensor Fusion Examples

### IMU and Vision Fusion

Combining inertial and visual sensors for robust state estimation:

```python
class IMUVisionFusion:
    def __init__(self):
        # Initialize EKF for state estimation
        # State: [x, y, z, roll, pitch, yaw, vx, vy, vz]
        self.state_dim = 9
        self.state = np.zeros(self.state_dim)
        self.covariance = np.eye(self.state_dim) * 1.0

        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.5, 0.5, 0.5])

        # Initialize for prediction step
        self.F = np.eye(self.state_dim)  # Will be updated
        self.last_time = time.time()

    def predict_with_imu(self, imu_data, dt):
        """Predict state using IMU data"""
        # Extract IMU measurements
        linear_acceleration = np.array([
            imu_data.get('ax', 0),
            imu_data.get('ay', 0),
            imu_data.get('az', 0)
        ])

        angular_velocity = np.array([
            imu_data.get('wx', 0),
            imu_data.get('wy', 0),
            imu_data.get('wz', 0)
        ])

        # Update state based on IMU data
        # Position integration
        self.state[6:9] += linear_acceleration * dt  # Update velocity
        self.state[0:3] += self.state[6:9] * dt      # Update position

        # Orientation integration
        self.state[3:6] += angular_velocity * dt     # Update roll, pitch, yaw

        # Update process model Jacobian
        self.F = self.compute_jacobian(dt)

        # Predict covariance
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q

    def compute_jacobian(self, dt):
        """Compute process model Jacobian"""
        F = np.eye(self.state_dim)

        # Position-velocity relationships
        F[0, 6] = dt  # dx/dvx
        F[1, 7] = dt  # dy/dvy
        F[2, 8] = dt  # dz/dvz

        # Velocity-acceleration relationships (simplified)
        # In a full implementation, this would involve rotation matrices
        return F

    def update_with_vision(self, vision_pos, vision_pos_cov):
        """Update state using vision data"""
        # Measurement model: vision provides position [x, y, z]
        H = np.zeros((3, self.state_dim))
        H[0, 0] = 1  # x measurement
        H[1, 1] = 1  # y measurement
        H[2, 2] = 1  # z measurement

        # Innovation
        y = vision_pos - self.state[0:3]

        # Innovation covariance
        S = H @ self.covariance @ H.T + vision_pos_cov

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.state += K @ y
        self.covariance = (np.eye(self.state_dim) - K @ H) @ self.covariance

    def update_with_depth(self, depth_measurement, depth_cov):
        """Update state using depth sensor data"""
        # Depth provides distance to known landmark
        # For simplicity, assume landmark is at [5, 0, 0]
        landmark_pos = np.array([5.0, 0.0, 0.0])

        # Predicted distance
        current_pos = self.state[0:3]
        predicted_distance = np.linalg.norm(landmark_pos - current_pos)

        # Measurement residual
        innovation = depth_measurement - predicted_distance

        # Measurement Jacobian
        if predicted_distance > 0:
            H = np.zeros((1, self.state_dim))
            range_vector = current_pos - landmark_pos
            H[0, 0:3] = range_vector / predicted_distance  # d/distance_d/d_pos

            # Innovation covariance
            S = H @ self.covariance @ H.T + depth_cov

            # Kalman gain
            K = self.covariance @ H.T @ np.linalg.inv(S)

            # Update
            self.state += K.flatten() * innovation
            self.covariance = (np.eye(self.state_dim) - np.outer(K, H)) @ self.covariance

    def get_state(self):
        """Get current state estimate"""
        return {
            'position': self.state[0:3],
            'orientation': self.state[3:6],
            'velocity': self.state[6:9],
            'uncertainty': np.sqrt(np.diag(self.covariance))
        }

# Example usage
imu_vision_fusion = IMUVisionFusion()

# Simulate sensor fusion over time
for step in range(100):
    dt = 0.01  # 100Hz

    # Simulate IMU data (with some motion)
    imu_data = {
        'ax': 0.1 + np.random.normal(0, 0.01),
        'ay': 0.05 + np.random.normal(0, 0.01),
        'az': -9.81 + np.random.normal(0, 0.01),  # Gravity
        'wx': np.random.normal(0, 0.001),
        'wy': np.random.normal(0, 0.001),
        'wz': np.random.normal(0, 0.001)
    }

    # Predict with IMU
    imu_vision_fusion.predict_with_imu(imu_data, dt)

    # Occasionally update with vision (e.g., every 10 steps)
    if step % 10 == 0:
        vision_pos = np.array([
            step * 0.001 + np.random.normal(0, 0.01),
            step * 0.0005 + np.random.normal(0, 0.01),
            0.8 + np.random.normal(0, 0.01)
        ])
        vision_cov = np.eye(3) * 0.01
        imu_vision_fusion.update_with_vision(vision_pos, vision_cov)

    # Occasionally update with depth
    if step % 15 == 0:
        true_distance = max(0.1, 5.0 - step * 0.001)  # Moving toward landmark
        measured_distance = true_distance + np.random.normal(0, 0.02)
        imu_vision_fusion.update_with_depth(measured_distance, 0.04)

final_state = imu_vision_fusion.get_state()
print(f"Final estimated position: {final_state['position']}")
print(f"Position uncertainty: {final_state['uncertainty'][:3]}")
```

### Multi-IMU Fusion

Combining data from multiple IMUs for improved orientation estimation:

```python
class MultiIMUFusion:
    def __init__(self, num_imus):
        self.num_imus = num_imus
        self.imu_quaternions = [np.array([1, 0, 0, 0]) for _ in range(num_imus)]
        self.imu_covariances = [np.eye(4) * 0.1 for _ in range(num_imus)]

        # Global state (best estimate)
        self.global_quaternion = np.array([1, 0, 0, 0])
        self.global_covariance = np.eye(4) * 0.1

    def update_imu_reading(self, imu_idx, accel, gyro, dt):
        """Update individual IMU estimate"""
        if imu_idx >= self.num_imus:
            return

        # Simple complementary filter for this IMU
        # Normalize accelerometer
        if np.linalg.norm(accel) > 0.1:
            accel_norm = accel / np.linalg.norm(accel)
        else:
            accel_norm = np.array([0, 0, 1])  # Default to Z-axis

        # Integrate gyroscope
        gyro_quat = self._integrate_gyro(self.imu_quaternions[imu_idx], gyro, dt)

        # Get gravity direction in IMU frame
        gravity_in_imu = self._rotate_vector(np.array([0, 0, 1]),
                                           self.imu_quaternions[imu_idx])

        # Calculate error between measured and expected gravity
        gravity_error = np.cross(gravity_in_imu, accel_norm)

        # Complementary filter: blend gyro integration with accelerometer
        kp = 0.1  # Proportional gain
        correction = kp * gravity_error
        correction_quat = np.array([0, correction[0], correction[1], correction[2]]) * 0.5

        # Apply correction
        corrected_quat = gyro_quat + correction_quat
        corrected_quat = corrected_quat / np.linalg.norm(corrected_quat)

        self.imu_quaternions[imu_idx] = corrected_quat

    def _integrate_gyro(self, quat, gyro, dt):
        """Integrate gyroscope to update quaternion"""
        gyro_quat = np.array([0, gyro[0], gyro[1], gyro[2]])
        quat_dot = 0.5 * self._quat_multiply(quat, gyro_quat)
        new_quat = quat + quat_dot * dt
        return new_quat / np.linalg.norm(new_quat)

    def _rotate_vector(self, v, q):
        """Rotate vector v by quaternion q"""
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        v_quat = np.array([0, v[0], v[1], v[2]])
        temp = self._quat_multiply(q_conj, v_quat)
        rotated = self._quat_multiply(temp, q)
        return rotated[1:]

    def _quat_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return np.array([w, x, y, z])

    def fuse_imu_data(self):
        """Fuse all IMU estimates using weighted averaging"""
        # Convert quaternions to a representation suitable for averaging
        # Use the chordal distance approach for quaternion averaging

        # Weights based on inverse covariance
        weights = []
        for cov in self.imu_covariances:
            # Use trace of covariance as a simple uncertainty measure
            uncertainty = np.trace(cov)
            weight = 1.0 / max(uncertainty, 1e-6)
            weights.append(weight)

        # Normalize weights
        total_weight = sum(weights)
        weights = [w / total_weight for w in weights]

        # Weighted quaternion averaging using Slerp-like approach
        # For simplicity, we'll use a chordal L2 mean approach
        avg_quat = np.zeros(4)
        for i, (quat, weight) in enumerate(zip(self.imu_quaternions, weights)):
            # Ensure quaternions have same handedness
            if np.dot(avg_quat, quat) < 0:
                quat = -quat
            avg_quat += weight * quat

        avg_quat = avg_quat / np.linalg.norm(avg_quat)
        self.global_quaternion = avg_quat

        # Update global covariance (simplified)
        self.global_covariance = np.mean(self.imu_covariances, axis=0)

    def get_global_orientation(self):
        """Get fused orientation estimate"""
        return {
            'quaternion': self.global_quaternion.copy(),
            'covariance': self.global_covariance.copy(),
            'euler_angles': self._quat_to_euler(self.global_quaternion)
        }

    def _quat_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = q
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

# Example: Fusing data from 3 IMUs (torso, left foot, right foot)
multi_imu = MultiIMUFusion(num_imus=3)

for step in range(100):
    dt = 0.01

    for i in range(3):
        # Simulate IMU readings (with slight differences)
        accel = np.array([0, 0, 9.81]) + np.random.normal(0, 0.1, 3)
        gyro = np.random.normal(0, 0.01, 3)
        multi_imu.update_imu_reading(i, accel, gyro, dt)

    # Fuse all IMU data
    multi_imu.fuse_imu_data()

final_orientation = multi_imu.get_global_orientation()
print(f"Fused quaternion: {final_orientation['quaternion']}")
print(f"Fused Euler angles (deg): {np.degrees(final_orientation['euler_angles'])}")
```

### Tactile and Force Sensing Fusion

Combining tactile and force sensors for manipulation:

```python
class TactileForceFusion:
    def __init__(self):
        self.tactile_data = np.zeros(64)  # Example: 64 taxels
        self.force_torque = np.zeros(6)  # [Fx, Fy, Fz, Mx, My, Mz]
        self.contact_points = []
        self.grasp_quality = 0.0

        # State for object properties estimation
        self.object_properties = {
            'compliance': 0.0,
            'friction': 0.0,
            'weight': 0.0,
            'shape': 'unknown'
        }

    def update_tactile_data(self, tactile_readings):
        """Update tactile sensor readings"""
        if len(tactile_readings) == len(self.tactile_data):
            self.tactile_data = np.array(tactile_readings)
        else:
            # Handle different sized tactile arrays
            min_len = min(len(tactile_readings), len(self.tactile_data))
            self.tactile_data[:min_len] = tactile_readings[:min_len]

    def update_force_torque(self, force_torque_readings):
        """Update force/torque sensor readings"""
        if len(force_torque_readings) == 6:
            self.force_torque = np.array(force_torque_readings)

    def estimate_contact_info(self):
        """Estimate contact information from tactile data"""
        # Find contact points (taxels with pressure above threshold)
        contact_threshold = 0.1
        contact_indices = np.where(self.tactile_data > contact_threshold)[0]

        contact_points = []
        total_force = 0

        if len(contact_indices) > 0:
            # For a 8x8 taxel array, convert linear indices to 2D positions
            for idx in contact_indices:
                row = idx // 8
                col = idx % 8
                # Convert to physical coordinates (e.g., meters)
                x = col * 0.005  # 5mm spacing
                y = row * 0.005
                pressure = self.tactile_data[idx]

                contact_points.append({
                    'position': np.array([x, y]),
                    'pressure': pressure,
                    'taxel_idx': idx
                })

                total_force += pressure

        self.contact_points = contact_points

        # Calculate center of pressure
        if total_force > 0:
            cop_x = sum(c['position'][0] * c['pressure'] for c in contact_points) / total_force
            cop_y = sum(c['position'][1] * c['pressure'] for c in contact_points) / total_force
            self.center_of_pressure = np.array([cop_x, cop_y])
        else:
            self.center_of_pressure = np.array([0.0, 0.0])

    def estimate_grasp_quality(self):
        """Estimate grasp quality from tactile and force data"""
        # Calculate various grasp quality metrics

        # 1. Contact distribution quality
        contact_area = len(self.contact_points)
        if contact_area > 0:
            # Calculate how spread out the contacts are
            positions = np.array([c['position'] for c in self.contact_points])
            if len(positions) > 1:
                centroid = np.mean(positions, axis=0)
                distances = np.linalg.norm(positions - centroid, axis=1)
                spread = np.mean(distances)
            else:
                spread = 0
        else:
            spread = 0

        # 2. Force distribution quality
        force_magnitude = np.linalg.norm(self.force_torque[:3])

        # 3. Tactile force uniformity
        active_pressures = [c['pressure'] for c in self.contact_points]
        if len(active_pressures) > 1:
            uniformity = 1.0 - np.std(active_pressures) / (np.mean(active_pressures) + 1e-6)
        else:
            uniformity = 1.0 if len(active_pressures) > 0 else 0.0

        # Combine metrics into overall quality score
        self.grasp_quality = (0.3 * min(1.0, contact_area / 10.0) +  # Adequate contacts
                             0.3 * min(1.0, spread / 0.02) +         # Good spread (2cm)
                             0.2 * min(1.0, force_magnitude / 50.0) + # Adequate force
                             0.2 * uniformity)                       # Uniform pressure

        return self.grasp_quality

    def detect_slip(self, time_window=0.1):
        """Detect slip using tactile pattern analysis"""
        # This would require temporal analysis
        # For now, return a simple slip detection based on pressure changes
        if len(self.contact_points) == 0:
            return False

        # Calculate pressure gradient across contact area
        positions = np.array([c['position'] for c in self.contact_points])
        pressures = np.array([c['pressure'] for c in self.contact_points])

        if len(positions) > 2:
            # Calculate spatial gradient of pressure
            from scipy.spatial.distance import pdist, squareform
            distances = squareform(pdist(positions))
            pressure_diffs = squareform(pdist(pressures.reshape(-1, 1)))

            # High pressure gradients might indicate slip
            avg_gradient = np.mean(pressure_diffs[distances > 1e-6]) if np.any(distances > 1e-6) else 0

            return avg_gradient > 5.0  # Threshold for slip detection

        return False

    def estimate_object_properties(self):
        """Estimate object properties from tactile and force data"""
        if len(self.contact_points) == 0:
            return self.object_properties

        # Estimate compliance (softness) from pressure distribution
        pressures = [c['pressure'] for c in self.contact_points]
        avg_pressure = np.mean(pressures) if pressures else 0

        # Higher variance in pressure might indicate softer material
        if len(pressures) > 1:
            pressure_variance = np.var(pressures)
            self.object_properties['compliance'] = pressure_variance / (avg_pressure + 1e-6)
        else:
            self.object_properties['compliance'] = 0

        # Estimate friction from force data
        normal_force = max(0.1, abs(self.force_torque[2]))  # Fz component
        tangential_force = np.linalg.norm(self.force_torque[:2])  # Fx, Fy components
        friction_coeff = tangential_force / normal_force
        self.object_properties['friction'] = min(1.0, friction_coeff)

        # Estimate weight from gravitational force
        self.object_properties['weight'] = max(0, -self.force_torque[2])  # Assuming object pulls down

        # Estimate shape from contact pattern
        if len(self.contact_points) > 2:
            positions = np.array([c['position'] for c in self.contact_points])
            from scipy.spatial.distance import pdist
            all_distances = pdist(positions)
            max_distance = np.max(all_distances) if len(all_distances) > 0 else 0

            if max_distance < 0.01:  # Small contact area
                self.object_properties['shape'] = 'small'
            elif max_distance < 0.03:  # Medium contact area
                self.object_properties['shape'] = 'medium'
            else:  # Large contact area
                self.object_properties['shape'] = 'large'

        return self.object_properties

    def get_fusion_results(self):
        """Get all fusion results"""
        self.estimate_contact_info()
        quality = self.estimate_grasp_quality()
        properties = self.estimate_object_properties()

        return {
            'grasp_quality': quality,
            'contact_points': self.contact_points,
            'center_of_pressure': getattr(self, 'center_of_pressure', np.array([0, 0])),
            'object_properties': properties,
            'slip_detected': self.detect_slip(),
            'tactile_data': self.tactile_data.copy(),
            'force_torque': self.force_torque.copy()
        }

# Example usage
tactile_force_fusion = TactileForceFusion()

# Simulate tactile data (e.g., from a 8x8 taxel array)
tactile_readings = np.random.random(64) * 5  # 0-5 N pressure
tactile_readings[10:20] = 10  # Simulate contact area
tactile_force_fusion.update_tactile_data(tactile_readings)

# Simulate force/torque data
force_torque_readings = [2.0, 1.0, -5.0, 0.1, 0.05, 0.02]  # [Fx, Fy, Fz, Mx, My, Mz]
tactile_force_fusion.update_force_torque(force_torque_readings)

results = tactile_force_fusion.get_fusion_results()
print(f"Grasp quality: {results['grasp_quality']:.3f}")
print(f"Object compliance: {results['object_properties']['compliance']:.3f}")
print(f"Object friction: {results['object_properties']['friction']:.3f}")
print(f"Estimated weight: {results['object_properties']['weight']:.3f} N")
print(f"Number of contact points: {len(results['contact_points'])}")
print(f"Slip detected: {results['slip_detected']}")
```

## Advanced Fusion Techniques

### Information Filter

The information filter is the inverse of the Kalman filter, useful for certain fusion scenarios:

```python
class InformationFilter:
    def __init__(self, state_dim):
        self.state_dim = state_dim
        self.information_state = np.zeros(state_dim)  # ξ = Y * x (information state)
        self.information_matrix = np.zeros((state_dim, state_dim))  # Y = P^(-1) (information matrix)

        # Initialize with high uncertainty (low information)
        self.information_matrix = np.eye(state_dim) * 0.001

    def predict(self, F, Q):
        """Prediction step in information form"""
        # Convert to covariance form temporarily
        covariance = np.linalg.inv(self.information_matrix)
        state = np.linalg.solve(self.information_matrix, self.information_state)

        # Standard Kalman prediction
        predicted_state = F @ state
        predicted_covariance = F @ covariance @ F.T + Q

        # Convert back to information form
        self.information_matrix = np.linalg.inv(predicted_covariance)
        self.information_state = self.information_matrix @ predicted_state

    def update(self, measurement, H, R):
        """Update step - incorporate measurement"""
        # Information contribution from measurement
        innovation_cov = H @ np.linalg.inv(self.information_matrix) @ H.T + R
        information_contribution = H.T @ np.linalg.inv(innovation_cov) @ H
        state_contribution = H.T @ np.linalg.inv(innovation_cov) @ measurement

        # Update information state and matrix
        self.information_matrix += information_contribution
        self.information_state += state_contribution

    def get_state_and_covariance(self):
        """Convert back to standard form"""
        covariance = np.linalg.inv(self.information_matrix)
        state = np.linalg.solve(self.information_matrix, self.information_state)
        return state, covariance

# Example: Information filter for sensor fusion
info_filter = InformationFilter(state_dim=2)  # [x, y] position

# Simulate measurements from different sensors
measurements = [
    (np.array([1.0, 0.1]), np.array([[1, 0], [0, 1]]), np.eye(2) * 0.1),   # High-accuracy sensor
    (np.array([1.1, 0.05]), np.array([[1, 0], [0, 1]]), np.eye(2) * 0.5),  # Low-accuracy sensor
    (np.array([0.95, 0.15]), np.array([[1, 0], [0, 1]]), np.eye(2) * 0.2)  # Medium-accuracy sensor
]

for measurement, H, R in measurements:
    info_filter.update(measurement, H, R)

final_state, final_cov = info_filter.get_state_and_covariance()
print(f"Fused position: {final_state}")
print(f"Uncertainty: {np.sqrt(np.diag(final_cov))}")
```

### Covariance Intersection

For fusing estimates when the correlation between sensors is unknown:

```python
def covariance_intersection(estimate1, cov1, estimate2, cov2):
    """
    Fuse two estimates using Covariance Intersection when correlation is unknown
    """
    # Compute fusion weights
    S1 = np.linalg.inv(cov1)
    S2 = np.linalg.inv(cov2)

    # The CI method finds optimal omega to minimize the trace of the fused covariance
    # This is a simplified approach - in practice, you'd solve for optimal omega
    omega = 0.5  # Equal weighting as a simple approach

    # Compute fused information matrix
    S_fused = omega * S1 + (1 - omega) * S2

    # Compute fused estimate
    info1 = S1 @ estimate1
    info2 = S2 @ estimate2
    info_fused = omega * info1 + (1 - omega) * info2

    # Convert back to state and covariance
    cov_fused = np.linalg.inv(S_fused)
    estimate_fused = cov_fused @ info_fused

    return estimate_fused, cov_fused

# Example: Fusing two position estimates with unknown correlation
est1 = np.array([1.0, 0.0])
cov1 = np.array([[0.1, 0.02], [0.02, 0.1]])

est2 = np.array([1.2, 0.1])
cov2 = np.array([[0.3, 0.05], [0.05, 0.3]])

fused_est, fused_cov = covariance_intersection(est1, cov1, est2, cov2)
print(f"CI fused estimate: {fused_est}")
print(f"CI fused covariance: {fused_cov}")
```

## Real-Time Implementation Considerations

### Efficient Data Structures

For real-time fusion, efficient data structures are crucial:

```python
from collections import deque
import time

class RealTimeFusionEngine:
    def __init__(self, max_buffer_size=100):
        self.max_buffer_size = max_buffer_size
        self.sensors = {}
        self.data_buffers = {}
        self.time_stamps = deque(maxlen=max_buffer_size)
        self.fusion_results = {}
        self.last_fusion_time = time.time()

    def register_sensor(self, sensor_name, topic_type, callback=None):
        """Register a sensor with the fusion engine"""
        self.sensors[sensor_name] = {
            'type': topic_type,
            'callback': callback,
            'buffer': deque(maxlen=self.max_buffer_size),
            'time_stamps': deque(maxlen=self.max_buffer_size)
        }

    def add_sensor_data(self, sensor_name, data, timestamp=None):
        """Add sensor data to the fusion engine"""
        if timestamp is None:
            timestamp = time.time()

        if sensor_name in self.sensors:
            self.sensors[sensor_name]['buffer'].append(data)
            self.sensors[sensor_name]['time_stamps'].append(timestamp)

    def synchronize_data(self, time_window=0.01):
        """Synchronize data from different sensors within time window"""
        if not self.sensors:
            return None

        # Find the latest common time
        latest_times = {}
        for sensor_name, sensor_data in self.sensors.items():
            if sensor_data['buffer']:
                latest_times[sensor_name] = sensor_data['time_stamps'][-1]

        if not latest_times:
            return None

        sync_time = min(latest_times.values())

        # Get data closest to sync time for each sensor
        synchronized_data = {}
        for sensor_name, sensor_data in self.sensors.items():
            buffer = sensor_data['buffer']
            time_stamps = sensor_data['time_stamps']

            if not buffer:
                continue

            # Find closest timestamp within window
            closest_idx = None
            closest_diff = float('inf')

            for i, ts in enumerate(time_stamps):
                diff = abs(ts - sync_time)
                if diff < closest_diff and diff < time_window:
                    closest_diff = diff
                    closest_idx = i

            if closest_idx is not None:
                synchronized_data[sensor_name] = {
                    'data': buffer[closest_idx],
                    'timestamp': time_stamps[closest_idx],
                    'age': time.time() - time_stamps[closest_idx]
                }

        return synchronized_data

    def run_fusion_cycle(self):
        """Run one cycle of sensor fusion"""
        start_time = time.time()

        # Synchronize data
        sync_data = self.synchronize_data()
        if not sync_data:
            return None

        # Perform fusion based on available data
        fusion_result = self.perform_fusion(sync_data)

        # Update timing statistics
        self.last_fusion_time = time.time()
        self.fusion_results['cycle_time'] = time.time() - start_time
        self.fusion_results['data_sync'] = sync_data

        return fusion_result

    def perform_fusion(self, sync_data):
        """Perform the actual fusion algorithm"""
        # This would contain the specific fusion algorithm
        # For example, a Kalman filter or other fusion method
        result = {
            'timestamp': time.time(),
            'sensors_used': list(sync_data.keys()),
            'data_quality': {}
        }

        # Calculate data quality metrics
        for sensor_name, sensor_data in sync_data.items():
            age = sensor_data['age']
            # Data becomes less reliable as it gets older
            quality = max(0, 1 - age)  # Simple quality model
            result['data_quality'][sensor_name] = quality

        return result

    def get_fusion_performance(self):
        """Get performance statistics"""
        return {
            'cycle_time': getattr(self, 'last_fusion_time', 0),
            'sensors_registered': len(self.sensors),
            'data_quality': self.fusion_results.get('data_quality', {}),
            'fusion_rate': 1.0 / self.fusion_results.get('cycle_time', 1.0) if self.fusion_results.get('cycle_time') else 0
        }

# Example usage
fusion_engine = RealTimeFusionEngine(max_buffer_size=50)

# Register different sensor types
fusion_engine.register_sensor('imu', 'sensor_msgs/Imu')
fusion_engine.register_sensor('camera', 'sensor_msgs/Image')
fusion_engine.register_sensor('lidar', 'sensor_msgs/PointCloud2')

# Simulate adding data
for i in range(10):
    fusion_engine.add_sensor_data('imu', {'acc': [0, 0, 9.8], 'gyro': [0, 0, 0]})
    fusion_engine.add_sensor_data('camera', {'objects': []})
    fusion_engine.add_sensor_data('lidar', {'points': []})

    result = fusion_engine.run_fusion_cycle()
    if result:
        print(f"Fusion cycle {i}: {len(result['sensors_used'])} sensors used")
        print(f"Data qualities: {result['data_quality']}")
```

## Integration with ROS 2

### ROS 2 Sensor Fusion Node

Integrating sensor fusion with ROS 2 for system-wide state estimation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import message_filters

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Create publishers for fused state
        self.odom_pub = self.create_publisher(Odometry, '/fused_odom', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/fused_pose', 10)
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, '/fused_twist', 10)

        # Create subscribers with time synchronization
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')
        self.joint_sub = message_filters.Subscriber(self, JointState, '/joint_states')

        # Synchronize messages based on timestamps
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.imu_sub, self.joint_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.ts.registerCallback(self.sync_callback)

        # Initialize fusion components
        self.kalman_filter = KalmanFilter(state_dim=13, measurement_dim=6)  # Pose + Twist
        self.multi_imu_fusion = MultiIMUFusion(num_imus=3)
        self.tactile_force_fusion = TactileForceFusion()

        # Robot state
        self.position = np.zeros(3)
        self.orientation = np.array([0, 0, 0, 1])  # x, y, z, w
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        # TF broadcaster
        from tf2_ros import TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Sensor fusion node initialized')

    def sync_callback(self, imu_msg, joint_msg):
        """Callback for synchronized IMU and joint messages"""
        # Extract IMU data
        imu_data = {
            'ax': imu_msg.linear_acceleration.x,
            'ay': imu_msg.linear_acceleration.y,
            'az': imu_msg.linear_acceleration.z,
            'wx': imu_msg.angular_velocity.x,
            'wy': imu_msg.angular_velocity.y,
            'wz': imu_msg.angular_velocity.z
        }

        # Extract joint data
        joint_positions = dict(zip(joint_msg.name, joint_msg.position))
        joint_velocities = dict(zip(joint_msg.name, joint_msg.velocity))

        # Perform sensor fusion
        self.update_fusion(imu_data, joint_positions, joint_velocities)

        # Publish results
        self.publish_fused_state(imu_msg.header.stamp)

    def update_fusion(self, imu_data, joint_positions, joint_velocities):
        """Update state estimate using sensor fusion"""
        # Update IMU-based orientation
        dt = 0.01  # Assuming 100Hz rate
        self.multi_imu_fusion.update_imu_reading(0,
            [imu_data['ax'], imu_data['ay'], imu_data['az']],
            [imu_data['wx'], imu_data['wy'], imu_data['wz']],
            dt)

        # Estimate position from joint encoders (forward kinematics)
        # This is a simplified example - in practice, you'd use full FK
        if 'left_knee' in joint_positions and 'right_knee' in joint_positions:
            # Simple leg extension model
            left_knee_angle = joint_positions['left_knee']
            right_knee_angle = joint_positions['right_knee']

            # Update position based on leg extension (simplified)
            self.position[2] += (np.sin(left_knee_angle) + np.sin(right_knee_angle)) * 0.001

        # Update Kalman filter
        measurement = np.array([
            self.position[0], self.position[1], self.position[2],  # Position
            self.orientation[0], self.orientation[1], self.orientation[2]  # Orientation (simplified)
        ])

        self.kalman_filter.update(measurement)

        # Predict next state with IMU
        self.kalman_filter.predict(dt)

        # Update with fused orientation
        fused_orientation = self.multi_imu_fusion.get_global_orientation()
        self.orientation = fused_orientation['quaternion']

    def publish_fused_state(self, stamp):
        """Publish fused state to ROS topics"""
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set pose
        odom_msg.pose.pose.position.x = float(self.position[0])
        odom_msg.pose.pose.position.y = float(self.position[1])
        odom_msg.pose.pose.position.z = float(self.position[2])

        odom_msg.pose.pose.orientation.x = float(self.orientation[0])
        odom_msg.pose.pose.orientation.y = float(self.orientation[1])
        odom_msg.pose.pose.orientation.z = float(self.orientation[2])
        odom_msg.pose.pose.orientation.w = float(self.orientation[3])

        # Set twist (velocity)
        odom_msg.twist.twist.linear.x = float(self.linear_velocity[0])
        odom_msg.twist.twist.linear.y = float(self.linear_velocity[1])
        odom_msg.twist.twist.linear.z = float(self.linear_velocity[2])

        odom_msg.twist.twist.angular.x = float(self.angular_velocity[0])
        odom_msg.twist.twist.angular.y = float(self.angular_velocity[1])
        odom_msg.twist.twist.angular.z = float(self.angular_velocity[2])

        # Set covariances (placeholder values)
        odom_msg.pose.covariance = [0.1] * 36  # Placeholder
        odom_msg.twist.covariance = [0.1] * 36  # Placeholder

        self.odom_pub.publish(odom_msg)

        # Publish pose with covariance
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose
        self.pose_pub.publish(pose_msg)

        # Publish twist with covariance
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header = odom_msg.header
        twist_msg.twist = odom_msg.twist
        self.twist_pub.publish(twist_msg)

        # Broadcast TF
        self.broadcast_transform(odom_msg)

    def broadcast_transform(self, odom_msg):
        """Broadcast transform from odom to base_link"""
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z

        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

Sensor fusion is a critical capability for humanoid robots, enabling them to create coherent, accurate, and robust representations of their state and environment from multiple, often noisy and incomplete, sensor sources. Through mathematical frameworks like Kalman filtering, particle filtering, and information fusion, robots can optimally combine data from different modalities.

The choice of fusion algorithm depends on the specific application requirements, including the nature of the sensors, the dynamics of the system, and real-time constraints. Modern humanoid robots typically employ multiple fusion techniques simultaneously, with different algorithms handling different aspects of perception and state estimation.

The integration of sensor fusion with ROS 2 enables system-wide state estimation that can be utilized by various robot subsystems, from low-level control to high-level planning and decision-making.