# Inertial and Proprioceptive Sensing

## Introduction: Understanding Self-Motion and Internal State

Inertial and proprioceptive sensing systems provide humanoid robots with awareness of their own motion, orientation, and internal configuration. These sensors form the "vestibular system" and "proprioceptive sense" of the robot, enabling it to understand its state in space and maintain balance. This section explores the principles, technologies, and applications of these critical sensing modalities.

### The Role of Inertial Sensing in Humanoid Robotics

Inertial sensors provide essential information for humanoid robots:

- **Attitude Estimation**: Understanding orientation relative to gravity
- **Motion Detection**: Detecting movement, acceleration, and rotation
- **Balance Control**: Maintaining upright posture during locomotion
- **State Estimation**: Tracking position, velocity, and orientation
- **Sensor Fusion**: Combining with other sensors for robust perception

## Inertial Measurement Units (IMUs)

### IMU Components and Principles

An IMU typically combines three types of sensors:

#### Accelerometers
Accelerometers measure linear acceleration along three orthogonal axes:

```python
class Accelerometer:
    def __init__(self, sensor_id, range_g=16.0, resolution=16):
        self.sensor_id = sensor_id
        self.range_g = range_g  # Measurement range in g (9.81 m/s^2)
        self.resolution = resolution  # ADC resolution in bits
        self.max_count = 2**(resolution-1) - 1
        self.bias = np.zeros(3)  # Bias calibration values
        self.scale_factor = np.array([1.0, 1.0, 1.0])  # Scale factor per axis
        self.current_reading = np.zeros(3)

    def read_raw(self):
        """Read raw accelerometer data (simulated)"""
        # In real hardware, this would interface with the sensor
        # For simulation, we'll add some realistic noise and bias
        true_acceleration = np.random.normal(0, 0.1, 3)  # 0.1 m/s^2 noise
        # Add gravity component (assuming Z-axis points up)
        true_acceleration[2] += 9.81  # Gravity

        # Apply bias, scale factor, and quantization
        biased_accel = true_acceleration + self.bias
        scaled_accel = biased_accel * self.scale_factor

        # Convert to digital counts and back to simulate quantization
        counts = np.round(scaled_accel * self.max_count / (self.range_g * 9.81))
        counts = np.clip(counts, -self.max_count, self.max_count)
        digital_accel = counts * (self.range_g * 9.81) / self.max_count

        return digital_accel

    def calibrate_bias(self, static_readings=100):
        """Calibrate accelerometer bias while sensor is static"""
        readings = []
        for _ in range(static_readings):
            raw = self.read_raw()
            readings.append(raw)

        avg_reading = np.mean(readings, axis=0)
        # Remove expected gravity (assuming Z-axis points up)
        self.bias = avg_reading - np.array([0, 0, 9.81])
        return self.bias

    def get_specific_force(self):
        """Get specific force (acceleration minus gravity)"""
        raw = self.read_raw()
        corrected = raw - self.bias
        # Subtract gravity component to get specific force
        specific_force = corrected - np.array([0, 0, 9.81])
        return specific_force

    def detect_impact(self, threshold=50.0):
        """Detect impact based on high acceleration"""
        raw = self.read_raw()
        magnitude = np.linalg.norm(raw)
        return magnitude > threshold
```

#### Gyroscopes
Gyroscopes measure angular velocity around three orthogonal axes:

```python
class Gyroscope:
    def __init__(self, sensor_id, range_dps=2000.0, resolution=16):
        self.sensor_id = sensor_id
        self.range_dps = range_dps  # Range in degrees per second
        self.resolution = resolution
        self.max_count = 2**(resolution-1) - 1
        self.bias = np.zeros(3)  # Bias in dps
        self.scale_factor = np.array([1.0, 1.0, 1.0])
        self.drift_rate = np.array([0.01, 0.01, 0.01])  # Drift in dps/hr
        self.integration_error = np.zeros(3)  # For drift compensation

    def read_raw(self):
        """Read raw gyroscope data (simulated)"""
        # Simulate true angular velocity with noise
        true_angular_vel = np.random.normal(0, 0.5, 3)  # 0.5 dps noise

        # Add bias and drift
        biased_vel = true_angular_vel + self.bias + self.integration_error

        # Apply scale factor
        scaled_vel = biased_vel * self.scale_factor

        # Quantization simulation
        counts = np.round(scaled_vel * self.max_count / self.range_dps)
        counts = np.clip(counts, -self.max_count, self.max_count)
        digital_vel = counts * self.range_dps / self.max_count

        return digital_vel

    def calibrate_bias(self, static_readings=100):
        """Calibrate gyroscope bias while sensor is stationary"""
        readings = []
        for _ in range(static_readings):
            raw = self.read_raw()
            readings.append(raw)

        self.bias = np.mean(readings, axis=0)
        return self.bias

    def integrate_for_orientation(self, dt, current_orientation_quat):
        """Integrate gyroscope data to update orientation"""
        angular_vel = self.read_raw() - self.bias  # Remove bias

        # Convert angular velocity to quaternion derivative
        # For quaternion q = [w, x, y, z], q_dot = 0.5 * Omega * q
        # where Omega is the skew-symmetric matrix of angular velocity
        omega_norm = np.linalg.norm(angular_vel)

        if omega_norm > 1e-6:  # Avoid division by zero
            axis = angular_vel / omega_norm
            angle = omega_norm * dt

            # Create rotation quaternion for this time step
            sin_half_angle = np.sin(angle / 2)
            cos_half_angle = np.cos(angle / 2)

            rotation_quat = np.array([
                cos_half_angle,
                axis[0] * sin_half_angle,
                axis[1] * sin_half_angle,
                axis[2] * sin_half_angle
            ])

            # Multiply with current orientation
            new_orientation = self.quaternion_multiply(rotation_quat, current_orientation_quat)
        else:
            new_orientation = current_orientation_quat

        return new_orientation / np.linalg.norm(new_orientation)  # Normalize

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

    def detect_rotation(self, threshold=10.0):
        """Detect significant rotation"""
        angular_vel = self.read_raw() - self.bias
        magnitude = np.linalg.norm(angular_vel)
        return magnitude > threshold
```

#### Magnetometers
Magnetometers provide absolute heading reference:

```python
class Magnetometer:
    def __init__(self, sensor_id, range_ga=4.0, resolution=16):
        self.sensor_id = sensor_id
        self.range_ga = range_ga  # Range in Gauss
        self.resolution = resolution
        self.max_count = 2**(resolution-1) - 1
        self.bias = np.zeros(3)  # Hard iron bias
        self.soft_iron_matrix = np.eye(3)  # Soft iron distortion matrix
        self.current_field = np.array([25000, 5000, 40000])  # Earth's magnetic field (nT)

    def read_raw(self):
        """Read raw magnetometer data (simulated)"""
        # Simulate magnetic field reading with noise
        noise = np.random.normal(0, 500, 3)  # 500 nT noise
        true_field = self.current_field + noise

        # Apply distortion (transpose of soft iron matrix)
        distorted_field = np.dot(self.soft_iron_matrix.T, true_field / 1000.0)  # Convert to Gauss

        # Add bias
        biased_field = distorted_field + self.bias

        # Quantization
        counts = np.round(biased_field * self.max_count / self.range_ga)
        counts = np.clip(counts, -self.max_count, self.max_count)
        digital_field = counts * self.range_ga / self.max_count

        return digital_field

    def get_heading(self, orientation_quat):
        """Calculate magnetic heading from magnetometer and orientation"""
        # Read magnetic field
        field_reading = self.read_raw()

        # Remove bias
        corrected_field = field_reading - self.bias

        # Transform to world frame using orientation quaternion
        world_field = self.quaternion_rotate_vector(orientation_quat, corrected_field)

        # Calculate heading (angle from magnetic north)
        heading = np.arctan2(world_field[1], world_field[0])
        return heading

    def quaternion_rotate_vector(self, q, v):
        """Rotate vector v by quaternion q"""
        # Convert vector to pure quaternion
        v_quat = np.array([0, v[0], v[1], v[2]])

        # q * v * q_conj
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        temp = self.quaternion_multiply(q, v_quat)
        rotated_quat = self.quaternion_multiply(temp, q_conj)

        return rotated_quat[1:]  # Return vector part

    def calibrate_hard_iron(self, num_samples=100):
        """Calibrate hard iron bias by sampling over 360 degrees"""
        readings = []

        print("Rotate sensor in figure-8 pattern for hard iron calibration...")
        for i in range(num_samples):
            reading = self.read_raw()
            readings.append(reading)
            time.sleep(0.1)  # 100ms between samples

        readings = np.array(readings)
        self.bias = np.mean(readings, axis=0)
        return self.bias

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])
```

### Complete IMU System

Combining all sensors into a complete IMU:

```python
class IMU:
    def __init__(self, sensor_id, position=np.zeros(3), orientation=np.eye(3)):
        self.sensor_id = sensor_id
        self.position = position  # Position relative to robot body
        self.orientation = orientation  # Orientation matrix
        self.accelerometer = Accelerometer(f"{sensor_id}_accel")
        self.gyroscope = Gyroscope(f"{sensor_id}_gyro")
        self.magnetometer = Magnetometer(f"{sensor_id}_mag")

        # State estimation
        self.orientation_quat = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        self.gravity = np.array([0, 0, 9.81])

        # Time tracking
        self.last_update_time = time.time()

    def read_all(self):
        """Read all IMU sensors"""
        accel = self.accelerometer.read_raw()
        gyro = self.gyroscope.read_raw()
        mag = self.magnetometer.read_raw()

        return {
            'accelerometer': accel,
            'gyroscope': gyro,
            'magnetometer': mag
        }

    def update_state(self, dt):
        """Update orientation and other state using sensor fusion"""
        # Get current readings
        readings = self.read_all()

        # Update orientation using gyroscope integration
        self.orientation_quat = self.gyroscope.integrate_for_orientation(
            dt, self.orientation_quat
        )

        # Store linear acceleration (remove gravity)
        accel_with_gravity = readings['accelerometer']
        gravity_in_sensor_frame = self.rotate_vector_to_sensor_frame(
            self.gravity, self.orientation_quat
        )
        self.linear_acceleration = accel_with_gravity - gravity_in_sensor_frame

        # Store angular velocity
        self.angular_velocity = readings['gyroscope'] - self.gyroscope.bias

    def rotate_vector_to_sensor_frame(self, vector, quat):
        """Rotate a vector from world frame to sensor frame using quaternion"""
        # q_conj * v_quat * q (where v_quat is [0, v_x, v_y, v_z])
        v_quat = np.array([0, vector[0], vector[1], vector[2]])
        q_conj = np.array([quat[0], -quat[1], -quat[2], -quat[3]])

        temp = self.gyroscope.quaternion_multiply(q_conj, v_quat)
        rotated_quat = self.gyroscope.quaternion_multiply(temp, quat)

        return rotated_quat[1:]  # Return vector part

    def get_euler_angles(self):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = self.orientation_quat

        # Normalize quaternion
        norm = np.linalg.norm(self.orientation_quat)
        if norm > 0:
            w, x, y, z = self.orientation_quat / norm

        # Convert to Euler angles
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    def is_upright(self, threshold=0.1):
        """Check if sensor is approximately upright"""
        euler = self.get_euler_angles()
        return abs(euler[0]) < threshold and abs(euler[1]) < threshold
```

## Attitude and Heading Reference Systems (AHRS)

### Complementary Filter for Attitude Estimation

Combining accelerometer, gyroscope, and magnetometer data:

```python
class ComplementaryFilter:
    def __init__(self, kp_acc=0.5, kp_mag=0.3, ki=0.0):
        self.kp_acc = kp_acc  # Proportional gain for accelerometer
        self.kp_mag = kp_mag  # Proportional gain for magnetometer
        self.ki = ki          # Integral gain
        self.integral_error = np.zeros(3)

        # Initial orientation (quaternion)
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])

        # Bias estimation
        self.gyro_bias = np.zeros(3)

    def update(self, accel, gyro, mag, dt):
        """Update orientation estimate using sensor fusion"""
        # Normalize accelerometer measurement
        if np.linalg.norm(accel) > 0:
            accel_norm = accel / np.linalg.norm(accel)
        else:
            accel_norm = np.array([0, 0, 1])  # Default to Z-axis if no acceleration

        # Integrate gyroscope measurements
        gyro_corrected = gyro - self.gyro_bias
        dq_gyro = self.integrate_gyro(gyro_corrected, dt)

        # Calculate correction from accelerometer
        gravity_est = self.rotate_vector(np.array([0, 0, 1]), self.orientation)
        acc_error = np.cross(gravity_est, accel_norm)

        # Apply proportional correction
        acc_correction = self.kp_acc * acc_error
        dq_acc = self.vector_to_quaternion(acc_correction)

        # Calculate correction from magnetometer
        if mag is not None and np.linalg.norm(mag) > 0:
            mag_norm = mag / np.linalg.norm(mag)
            # Calculate expected magnetic field in body frame
            mag_body = self.rotate_vector(mag_norm,
                                        self.quaternion_conjugate(self.orientation))
            # Calculate error in horizontal plane
            mag_error = np.array([
                mag_body[1],  # East component should be 0
                -mag_body[0], # North component should align with Y
                0
            ])
            mag_correction = self.kp_mag * mag_error
            dq_mag = self.vector_to_quaternion(mag_correction)
        else:
            dq_mag = np.array([0, 0, 0, 0])

        # Combine corrections
        dq_total = dq_gyro + dq_acc + dq_mag

        # Normalize and integrate
        dq_total = dq_total / np.linalg.norm(dq_total) if np.linalg.norm(dq_total) > 0 else dq_total
        self.orientation = self.quaternion_multiply(
            self.orientation,
            dt * dq_total * 0.5
        )

        # Normalize quaternion
        self.orientation = self.orientation / np.linalg.norm(self.orientation)

        # Update bias estimation if correction is small (likely due to bias)
        if np.linalg.norm(acc_error) < 0.1:
            self.integral_error += acc_error * dt
            self.gyro_bias += self.ki * self.integral_error

    def integrate_gyro(self, gyro, dt):
        """Integrate gyroscope to update orientation"""
        # Convert angular velocity to quaternion derivative
        gyro_quat = np.array([0, gyro[0], gyro[1], gyro[2]])
        dq = self.quaternion_multiply(self.orientation, gyro_quat) * 0.5
        return dq

    def rotate_vector(self, v, q):
        """Rotate vector v by quaternion q"""
        # q * [0, v] * q_conj
        v_quat = np.array([0, v[0], v[1], v[2]])
        q_conj = self.quaternion_conjugate(q)
        temp = self.quaternion_multiply(q, v_quat)
        rotated_quat = self.quaternion_multiply(temp, q_conj)
        return rotated_quat[1:]

    def quaternion_conjugate(self, q):
        """Return quaternion conjugate"""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

    def vector_to_quaternion(self, v):
        """Convert 3D vector to quaternion (pure quaternion)"""
        return np.array([0, v[0], v[1], v[2]])
```

### Extended Kalman Filter for IMU Fusion

More sophisticated filtering approach:

```python
class EKFFilter:
    def __init__(self):
        # State: [q_w, q_x, q_y, q_z, b_x, b_y, b_z]
        # where q is orientation quaternion and b is gyroscope bias
        self.state = np.array([1, 0, 0, 0, 0, 0, 0], dtype=float)
        self.covariance = np.eye(7) * 0.1  # Initial uncertainty

        # Process noise
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6])

        # Measurement noise
        self.R_acc = np.eye(3) * 0.01  # Accelerometer noise
        self.R_mag = np.eye(3) * 0.02  # Magnetometer noise

    def predict(self, gyro, dt):
        """Prediction step using gyroscope measurements"""
        q = self.state[:4]  # Orientation quaternion
        b = self.state[4:]  # Gyroscope bias

        # Corrected angular velocity
        omega = gyro - b

        # Quaternion derivative
        omega_skew = np.array([
            [0, -omega[0], -omega[1], -omega[2]],
            [omega[0], 0, omega[2], -omega[1]],
            [omega[1], -omega[2], 0, omega[0]],
            [omega[2], omega[1], -omega[0], 0]
        ])

        # Update quaternion
        q_dot = 0.5 * np.dot(omega_skew, q)
        q_new = q + q_dot * dt

        # Normalize quaternion
        q_new = q_new / np.linalg.norm(q_new)

        # Update state
        self.state[:4] = q_new

        # Jacobian of process model
        F = np.eye(7)
        F[:4, :4] = self.compute_quaternion_jacobian(omega, dt)
        # Bias remains constant in prediction

        # Update covariance
        self.covariance = np.dot(np.dot(F, self.covariance), F.T) + self.Q

    def compute_quaternion_jacobian(self, omega, dt):
        """Compute Jacobian of quaternion propagation"""
        norm_omega = np.linalg.norm(omega)
        if norm_omega < 1e-6:
            # Small angle approximation
            return np.eye(4)

        omega_unit = omega / norm_omega
        cos_half_angle = np.cos(norm_omega * dt / 2)
        sin_half_angle = np.sin(norm_omega * dt / 2)

        # Jacobian computation
        F = np.zeros((4, 4))
        F[0, 0] = cos_half_angle
        F[0, 1:] = -sin_half_angle * omega_unit * dt / 2
        F[1:, 0] = sin_half_angle * omega_unit * dt / 2

        # Cross product terms
        omega_skew = np.array([
            [0, -omega_unit[2], omega_unit[1]],
            [omega_unit[2], 0, -omega_unit[0]],
            [-omega_unit[1], omega_unit[0], 0]
        ])

        F[1:, 1:] = cos_half_angle * np.eye(3) + \
                   (1 - cos_half_angle) * np.outer(omega_unit, omega_unit) + \
                   sin_half_angle * omega_skew

        return F

    def update_with_accelerometer(self, accel):
        """Update step using accelerometer measurement"""
        if np.linalg.norm(accel) < 1e-6:
            return

        # Normalize accelerometer
        z_acc = accel / np.linalg.norm(accel)

        # Expected measurement (gravity in body frame)
        q = self.state[:4]
        gravity_body = self.rotate_vector_to_body(np.array([0, 0, 1]), q)

        # Measurement residual
        y = z_acc - gravity_body

        # Measurement Jacobian
        H = np.zeros((3, 7))
        H[:, :4] = self.compute_h_jacobian(q)

        # Kalman gain
        S = np.dot(np.dot(H, self.covariance), H.T) + self.R_acc
        K = np.dot(np.dot(self.covariance, H.T), np.linalg.inv(S))

        # Update state and covariance
        self.state += np.dot(K, y)
        self.covariance = np.dot((np.eye(7) - np.dot(K, H)), self.covariance)

        # Re-normalize quaternion
        self.state[:4] = self.state[:4] / np.linalg.norm(self.state[:4])

    def rotate_vector_to_body(self, v, q):
        """Rotate vector from world to body frame"""
        # q_conj * [0, v] * q
        v_quat = np.array([0, v[0], v[1], v[2]])
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        temp = self.quaternion_multiply(q_conj, v_quat)
        rotated = self.quaternion_multiply(temp, q)
        return rotated[1:]

    def compute_h_jacobian(self, q):
        """Compute measurement Jacobian for accelerometer"""
        # Simplified Jacobian computation
        H = np.zeros((3, 7))

        # For gravity vector [0, 0, 1] rotated to body frame
        w, x, y, z = q
        H[0, 0] = 2*(w*y - x*z)
        H[0, 1] = 2*(x*y + w*z)
        H[0, 2] = w*w - x*x + y*y - z*z
        H[0, 3] = 2*(y*z - w*x)

        H[1, 0] = 2*(-w*x - y*z)
        H[1, 1] = w*w - x*x - y*y + z*z
        H[1, 2] = 2*(x*y - w*z)
        H[1, 3] = 2*(w*y + x*z)

        H[2, 0] = 2*(w*z - x*y)
        H[2, 1] = 2*(w*y + x*z)
        H[2, 2] = 2*(-w*x + y*z)
        H[2, 3] = w*w + x*x - y*y - z*z

        return H

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])
```

## Joint Position and Proprioceptive Sensing

### Encoder-Based Position Sensing

Joint encoders provide precise position feedback:

```python
class JointEncoder:
    def __init__(self, joint_name, type='incremental', resolution=12):
        self.joint_name = joint_name
        self.type = type  # 'incremental' or 'absolute'
        self.resolution = resolution  # bits
        self.max_count = 2**resolution
        self.current_count = 0
        self.position = 0.0  # radians
        self.velocity = 0.0  # rad/s
        self.acceleration = 0.0  # rad/s^2
        self.gear_ratio = 1.0
        self.offset = 0.0
        self.previous_position = 0.0
        self.previous_time = time.time()

        # For absolute encoders
        self.multi_turn_count = 0
        self.last_raw_count = 0

    def read_raw(self):
        """Read raw encoder count (simulated)"""
        # In real hardware, this would read from encoder
        # For simulation, we'll return a changing value
        import random
        change = random.uniform(-10, 10)  # Simulate position change
        self.current_count = (self.current_count + int(change)) % self.max_count
        return self.current_count

    def update_position(self, dt):
        """Update position, velocity, and acceleration"""
        raw_count = self.read_raw()

        if self.type == 'incremental':
            # Handle overflow/underflow for incremental encoders
            if raw_count - self.last_raw_count > self.max_count / 2:
                # Wrapped around from max to 0
                self.multi_turn_count -= 1
            elif raw_count - self.last_raw_count < -self.max_count / 2:
                # Wrapped around from 0 to max
                self.multi_turn_count += 1

            # Calculate total count including turns
            total_count = raw_count + self.multi_turn_count * self.max_count
            self.last_raw_count = raw_count
        else:
            # Absolute encoder - no multi-turn tracking needed
            total_count = raw_count

        # Convert to radians
        raw_angle = (total_count / self.max_count) * 2 * np.pi
        self.position = raw_angle * self.gear_ratio - self.offset

        # Calculate velocity
        current_time = time.time()
        if dt > 0:
            self.velocity = (self.position - self.previous_position) / dt
            self.acceleration = (self.velocity - self.previous_velocity) / dt if dt > 0 else 0.0

        # Update previous values
        self.previous_position = self.position
        self.previous_velocity = self.velocity
        self.previous_time = current_time

    def calibrate(self, reference_position):
        """Calibrate encoder offset"""
        raw_count = self.read_raw()
        raw_angle = (raw_count / self.max_count) * 2 * np.pi
        current_angle = raw_angle * self.gear_ratio
        self.offset = current_angle - reference_position

    def get_position(self):
        """Get current joint position"""
        return self.position

    def get_velocity(self):
        """Get current joint velocity"""
        return self.velocity

    def get_acceleration(self):
        """Get current joint acceleration"""
        return self.acceleration

    def is_at_limit(self, min_pos, max_pos, tolerance=0.01):
        """Check if joint is at position limits"""
        return (self.position <= min_pos + tolerance or
                self.position >= max_pos - tolerance)

    def detect_backlash(self, direction_change_threshold=0.1):
        """Detect backlash by monitoring direction changes"""
        # This would require more sophisticated analysis
        # For now, return a simple backlash indicator
        if abs(self.velocity) > 0.1:  # Moving
            if self.velocity * self.previous_velocity < 0:  # Direction changed
                return True
        return False
```

### Motor Current Sensing

Motor current provides information about applied torque:

```python
class MotorCurrentSensor:
    def __init__(self, motor_name, max_current=10.0, torque_constant=0.1):
        self.motor_name = motor_name
        self.max_current = max_current  # Amperes
        self.torque_constant = torque_constant  # Nm/A
        self.current = 0.0
        self.torque = 0.0
        self.temperature = 25.0
        self.overcurrent_threshold = 0.8 * max_current
        self.current_history = deque(maxlen=100)

    def read_current(self):
        """Read motor current (simulated)"""
        # Simulate current reading with noise
        load_factor = np.random.uniform(0.1, 0.9)  # 10-90% of max current
        noise = np.random.normal(0, 0.05)  # 50mA noise
        current = load_factor * self.max_current + noise
        return np.clip(current, -self.max_current, self.max_current)

    def update_torque_estimate(self):
        """Update torque estimate based on current"""
        self.current = self.read_current()
        self.torque = self.current * self.torque_constant
        self.current_history.append(self.current)

        # Update temperature estimate based on current
        self.temperature += 0.01 * (self.current**2) * 0.001  # Simplified heating model

        return self.torque

    def detect_overload(self):
        """Detect motor overload condition"""
        return abs(self.current) > self.overcurrent_threshold

    def detect_stall(self, current_threshold=0.9 * self.max_current,
                     duration_threshold=0.1):
        """Detect motor stall condition"""
        if abs(self.current) > current_threshold:
            # Check if high current persists
            recent_currents = list(self.current_history)[-int(duration_threshold/0.001):]
            if len(recent_currents) > 0:
                avg_current = np.mean(np.abs(recent_currents))
                return avg_current > current_threshold
        return False

    def get_effort_percentage(self):
        """Get motor effort as percentage of maximum"""
        return abs(self.current) / self.max_current * 100.0

    def detect_oscillation(self, threshold=0.5):
        """Detect oscillatory behavior in current"""
        if len(self.current_history) < 10:
            return False

        recent_currents = list(self.current_history)[-10:]
        current_changes = np.diff(recent_currents)

        # Count direction changes
        direction_changes = np.sum(current_changes[1:] * current_changes[:-1] < 0)
        return direction_changes > threshold * len(current_changes)
```

## Sensor Fusion for State Estimation

### Kalman Filter for Joint State Estimation

Combining multiple proprioceptive sensors:

```python
class JointStateEstimator:
    def __init__(self, joint_name):
        self.joint_name = joint_name
        self.dt = 0.001  # 1ms update rate

        # State: [position, velocity, acceleration, torque]
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        self.covariance = np.eye(4) * 0.1

        # Process model (constant acceleration model)
        self.F = np.array([
            [1, self.dt, 0.5*self.dt**2, 0],
            [0, 1, self.dt, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]  # Torque is independent
        ])

        # Process noise
        self.Q = np.diag([1e-4, 1e-3, 1e-2, 1e-1])

        # Measurement matrices
        self.H_pos = np.array([[1, 0, 0, 0]])  # Position measurement
        self.H_vel = np.array([[0, 1, 0, 0]])  # Velocity measurement
        self.H_torque = np.array([[0, 0, 0, 1]])  # Torque measurement

        # Measurement noise
        self.R_pos = np.array([[0.001]])  # 1mrad position noise
        self.R_vel = np.array([[0.01]])   # 0.01 rad/s velocity noise
        self.R_torque = np.array([[0.1]]) # 0.1 Nm torque noise

    def predict(self):
        """Prediction step"""
        self.state = np.dot(self.F, self.state)
        self.covariance = np.dot(np.dot(self.F, self.covariance), self.F.T) + self.Q

    def update_position(self, measured_pos, noise=None):
        """Update with position measurement"""
        if noise is not None:
            R = np.array([[noise]])
        else:
            R = self.R_pos

        y = np.array([measured_pos - self.state[0]])  # Measurement residual
        S = np.dot(np.dot(self.H_pos, self.covariance), self.H_pos.T) + R
        K = np.dot(np.dot(self.covariance, self.H_pos.T), np.linalg.inv(S))

        self.state += np.dot(K, y)
        self.covariance = np.dot((np.eye(4) - np.dot(K, self.H_pos)), self.covariance)

    def update_velocity(self, measured_vel, noise=None):
        """Update with velocity measurement"""
        if noise is not None:
            R = np.array([[noise]])
        else:
            R = self.R_vel

        y = np.array([measured_vel - self.state[1]])  # Measurement residual
        S = np.dot(np.dot(self.H_vel, self.covariance), self.H_vel.T) + R
        K = np.dot(np.dot(self.covariance, self.H_vel.T), np.linalg.inv(S))

        self.state += np.dot(K, y)
        self.covariance = np.dot((np.eye(4) - np.dot(K, self.H_vel)), self.covariance)

    def update_torque(self, measured_torque, noise=None):
        """Update with torque measurement"""
        if noise is not None:
            R = np.array([[noise]])
        else:
            R = self.R_torque

        y = np.array([measured_torque - self.state[3]])  # Measurement residual
        S = np.dot(np.dot(self.H_torque, self.covariance), self.H_torque.T) + R
        K = np.dot(np.dot(self.covariance, self.H_torque.T), np.linalg.inv(S))

        self.state += np.dot(K, y)
        self.covariance = np.dot((np.eye(4) - np.dot(K, self.H_torque)), self.covariance)

    def get_state(self):
        """Get current estimated state"""
        return {
            'position': self.state[0],
            'velocity': self.state[1],
            'acceleration': self.state[2],
            'torque': self.state[3],
            'uncertainty': np.sqrt(np.diag(self.covariance))
        }
```

### Whole-Body State Estimation

Estimating the state of the entire humanoid robot:

```python
class WholeBodyStateEstimator:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.joint_estimators = {}
        self.imus = {}
        self.contact_sensors = {}

        # Initialize estimators for each joint
        for joint_name in robot_model.joint_names:
            self.joint_estimators[joint_name] = JointStateEstimator(joint_name)

        # Initialize IMUs (one on torso, one on each foot, etc.)
        self.imus['torso'] = IMU('torso_imu')
        self.imus['left_foot'] = IMU('left_foot_imu')
        self.imus['right_foot'] = IMU('right_foot_imu')

        # Robot state
        self.com_position = np.zeros(3)
        self.com_velocity = np.zeros(3)
        self.com_acceleration = np.zeros(3)
        self.support_polygon = None
        self.zmp = np.zeros(2)  # Zero-Moment Point

    def update_joint_states(self, joint_positions, joint_velocities, joint_torques):
        """Update all joint state estimators"""
        for i, joint_name in enumerate(self.robot_model.joint_names):
            if i < len(joint_positions):
                # Update joint estimator with measurements
                self.joint_estimators[joint_name].update_position(joint_positions[i])
                if i < len(joint_velocities):
                    self.joint_estimators[joint_name].update_velocity(joint_velocities[i])
                if i < len(joint_torques):
                    self.joint_estimators[joint_name].update_torque(joint_torques[i])

                # Prediction step
                self.joint_estimators[joint_name].predict()

    def update_imu_states(self, imu_data):
        """Update IMU-based state estimates"""
        for imu_name, data in imu_data.items():
            if imu_name in self.imus:
                # Update IMU state
                self.imus[imu_name].accelerometer.current_reading = data['accel']
                self.imus[imu_name].gyroscope.current_reading = data['gyro']
                if 'mag' in data:
                    self.imus[imu_name].magnetometer.current_field = data['mag']

                # Update orientation
                dt = 0.001  # 1ms
                self.imus[imu_name].update_state(dt)

    def estimate_center_of_mass(self):
        """Estimate center of mass position and velocity"""
        # This would involve forward kinematics and mass distribution
        # For this example, we'll use a simplified approach

        # Get current joint positions from estimators
        joint_positions = {}
        joint_velocities = {}

        for joint_name, estimator in self.joint_estimators.items():
            state = estimator.get_state()
            joint_positions[joint_name] = state['position']
            joint_velocities[joint_name] = state['velocity']

        # Calculate CoM using robot model and current configuration
        # This requires the robot's URDF and mass distribution
        # For now, return a placeholder
        self.com_position = np.array([0.0, 0.0, 0.8])  # Simplified
        self.com_velocity = np.array([0.0, 0.0, 0.0])
        self.com_acceleration = np.array([0.0, 0.0, 0.0])

        return self.com_position, self.com_velocity, self.com_acceleration

    def estimate_zmp(self):
        """Estimate Zero-Moment Point"""
        # ZMP = CoM - (CoM_height / g) * CoM_acceleration_xy
        if self.com_acceleration[2] != 0:  # Avoid division by zero
            zmp_x = self.com_position[0] - (self.com_position[2] / 9.81) * self.com_acceleration[0]
            zmp_y = self.com_position[1] - (self.com_position[2] / 9.81) * self.com_acceleration[1]
            self.zmp = np.array([zmp_x, zmp_y])

        return self.zmp

    def detect_support_polygon(self, contact_data):
        """Detect support polygon from contact sensors"""
        # Determine which feet/points are in contact
        contacts = []

        if contact_data.get('left_foot_contact', False):
            contacts.append(self.get_foot_position('left'))

        if contact_data.get('right_foot_contact', False):
            contacts.append(self.get_foot_position('right'))

        if len(contacts) > 0:
            self.support_polygon = np.array(contacts)

        return self.support_polygon

    def get_foot_position(self, foot_side):
        """Get foot position in world coordinates"""
        # This would require forward kinematics
        # For now, return a placeholder
        if foot_side == 'left':
            return np.array([0.1, -0.1, 0.0])
        else:
            return np.array([0.1, 0.1, 0.0])

    def is_balanced(self):
        """Check if robot is in balance"""
        if self.support_polygon is None or len(self.support_polygon) == 0:
            return False

        zmp = self.estimate_zmp()

        # Check if ZMP is within support polygon
        # Simple check for rectangular support polygon
        if len(self.support_polygon) >= 3:
            # Calculate bounding box of support polygon
            min_x = np.min(self.support_polygon[:, 0])
            max_x = np.max(self.support_polygon[:, 0])
            min_y = np.min(self.support_polygon[:, 1])
            max_y = np.max(self.support_polygon[:, 1])

            return (min_x <= zmp[0] <= max_x and min_y <= zmp[1] <= max_y)

        return False
```

## Integration with ROS 2

### ROS 2 Inertial Sensing Node

Integrating inertial and proprioceptive sensing with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header, Float64MultiArray
from tf2_ros import TransformBroadcaster

class InertialSensingNode(Node):
    def __init__(self):
        super().__init__('inertial_sensing_node')

        # Create publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states_est', 10)
        self.com_pub = self.create_publisher(Float64MultiArray, '/center_of_mass', 10)

        # Create subscribers
        self.raw_imu_sub = self.create_subscription(
            Imu, '/imu/raw', self.raw_imu_callback, 10
        )
        self.raw_joint_sub = self.create_subscription(
            JointState, '/joint_states', self.raw_joint_callback, 10
        )

        # Initialize sensors and estimators
        self.imu_filter = ComplementaryFilter()
        self.whole_body_estimator = WholeBodyStateEstimator(robot_model=None)  # Placeholder
        self.joint_encoders = {}  # Will be populated from joint names

        # Initialize joint encoders for common humanoid joints
        humanoid_joints = [
            'left_hip_roll', 'left_hip_pitch', 'left_hip_yaw',
            'left_knee', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_roll', 'right_hip_pitch', 'right_hip_yaw',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            'torso_yaw', 'torso_pitch', 'torso_roll',
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow', 'left_wrist_yaw', 'left_wrist_pitch',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
            'right_elbow', 'right_wrist_yaw', 'right_wrist_pitch'
        ]

        for joint_name in humanoid_joints:
            self.joint_encoders[joint_name] = JointEncoder(joint_name)

        # Timer for state estimation
        self.estimation_timer = self.create_timer(0.01, self.update_estimation)  # 100Hz

        # TF broadcaster for transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Inertial sensing node initialized')

    def raw_imu_callback(self, msg):
        """Process raw IMU data"""
        # Extract IMU readings
        accel = np.array([msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z])
        mag = None  # Magnetometer might not be available in all IMU messages
        if msg.orientation.w != 0:  # Check if orientation is valid
            # Extract magnetic field from orientation (simplified)
            # In practice, separate magnetometer data would be used
            pass

        # Update IMU filter
        dt = 0.01  # Assuming 100Hz rate
        self.imu_filter.update(accel, gyro, mag, dt)

    def raw_joint_callback(self, msg):
        """Process raw joint state data"""
        # Update joint encoders with raw measurements
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_encoders:
                # Update encoder with raw position
                self.joint_encoders[joint_name].position = msg.position[i]
                if i < len(msg.velocity):
                    self.joint_encoders[joint_name].velocity = msg.velocity[i]
                if i < len(msg.effort):
                    # Use effort to estimate torque
                    self.joint_encoders[joint_name].torque = msg.effort[i]

    def update_estimation(self):
        """Update state estimation and publish results"""
        # Update joint state estimates
        joint_positions = []
        joint_velocities = []
        joint_names = []

        for joint_name, encoder in self.joint_encoders.items():
            if encoder.position is not None:
                joint_positions.append(encoder.position)
                joint_velocities.append(encoder.velocity)
                joint_names.append(joint_name)

        # Update whole body state estimator
        if joint_positions and joint_velocities:
            self.whole_body_estimator.update_joint_states(
                joint_positions, joint_velocities, [0]*len(joint_positions)  # Zero torques for now
            )

        # Publish estimated joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = 'base_link'
        joint_state_msg.name = joint_names
        joint_state_msg.position = joint_positions
        joint_state_msg.velocity = joint_velocities
        # No effort in this example

        self.joint_state_pub.publish(joint_state_msg)

        # Publish IMU data with filtered orientation
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Set orientation from filter
        orientation = self.imu_filter.orientation
        imu_msg.orientation = Quaternion(
            x=float(orientation[1]),
            y=float(orientation[2]),
            z=float(orientation[3]),
            w=float(orientation[0])
        )

        # Set angular velocity and linear acceleration
        # These would come from the actual sensors
        imu_msg.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        imu_msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)

        # Set covariance matrices (placeholder values)
        imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        self.imu_pub.publish(imu_msg)

        # Publish center of mass estimate
        com_msg = Float64MultiArray()
        com_msg.data = [0.0, 0.0, 0.8]  # Placeholder CoM position
        self.com_pub.publish(com_msg)

        # Broadcast TF transforms
        self.broadcast_transforms()

    def broadcast_transforms(self):
        """Broadcast coordinate frame transforms"""
        from geometry_msgs.msg import TransformStamped

        # Create transform from base to IMU
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.8  # IMU at torso height

        # Identity rotation (IMU aligned with base frame)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    inertial_node = InertialSensingNode()

    try:
        rclpy.spin(inertial_node)
    except KeyboardInterrupt:
        pass
    finally:
        inertial_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

Inertial and proprioceptive sensing systems provide humanoid robots with crucial self-awareness, enabling them to understand their own motion, orientation, and internal configuration. These systems form the foundation for balance control, state estimation, and coordinated movement. Through sensor fusion techniques like complementary filtering and Kalman filtering, multiple sensor modalities are combined to provide robust and accurate state estimates.

The integration of these sensors with ROS 2 enables system-wide awareness of the robot's state, supporting higher-level control and planning functions. The next section will explore sensor fusion techniques that combine data from multiple sensing modalities to create comprehensive environmental and self-state representations.