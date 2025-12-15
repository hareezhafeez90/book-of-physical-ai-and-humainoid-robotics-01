# Sensory Modalities in Humanoid Robotics

## Introduction: The Need for Multiple Senses

Humanoid robots require diverse sensory modalities to operate effectively in human environments. Unlike traditional industrial robots that work in controlled settings, humanoid robots must perceive and respond to complex, dynamic environments filled with humans, objects, and unpredictable situations. This section explores the different types of sensors used in humanoid robotics, categorized by their function and the information they provide.

### Classification of Sensors

Sensors in humanoid robotics can be broadly classified into two categories:

#### Proprioceptive Sensors
- **Function**: Provide information about the robot's internal state
- **Examples**: Joint encoders, IMUs, motor current sensors
- **Purpose**: Monitor robot configuration, balance, and internal forces

#### Exteroceptive Sensors
- **Function**: Provide information about the external environment
- **Examples**: Cameras, LIDAR, tactile sensors, microphones
- **Purpose**: Perceive objects, obstacles, humans, and environmental conditions

## Proprioceptive Sensing Systems

### Joint Position Sensors

Joint encoders are fundamental to humanoid operation, providing precise measurements of joint angles:

```python
class JointEncoder:
    def __init__(self, joint_name, resolution=16):
        self.joint_name = joint_name
        self.resolution = resolution  # bits
        self.max_count = 2**resolution
        self.offset = 0.0  # radians
        self.position = 0.0  # radians

    def read_position(self):
        """Read raw encoder count and convert to angle"""
        # In practice, this would interface with hardware
        raw_count = self.get_raw_count()  # Placeholder
        angle = (raw_count / self.max_count) * 2 * np.pi
        return angle - self.offset

    def calibrate(self, reference_position):
        """Calibrate encoder offset"""
        current_raw = self.get_raw_count()
        current_angle = (current_raw / self.max_count) * 2 * np.pi
        self.offset = current_angle - reference_position
```

### Inertial Measurement Units (IMUs)

IMUs provide crucial information about orientation, acceleration, and angular velocity:

```python
import numpy as np

class IMUSensor:
    def __init__(self, sensor_id):
        self.sensor_id = sensor_id
        self.orientation = np.array([1, 0, 0, 0])  # Quaternion [w, x, y, z]
        self.angular_velocity = np.zeros(3)  # rad/s [x, y, z]
        self.linear_acceleration = np.zeros(3)  # m/s^2 [x, y, z]
        self.temperature = 25.0  # degrees Celsius

    def update_from_raw_data(self, raw_orientation, raw_angular_vel, raw_linear_acc):
        """Update sensor state from raw measurements"""
        # Normalize quaternion
        self.orientation = raw_orientation / np.linalg.norm(raw_orientation)
        self.angular_velocity = raw_angular_vel
        self.linear_acceleration = raw_linear_acc

    def get_euler_angles(self):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = self.orientation

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    def is_upright(self, threshold=0.1):
        """Check if sensor is approximately upright (for balance)"""
        euler = self.get_euler_angles()
        return abs(euler[0]) < threshold and abs(euler[1]) < threshold
```

### Force/Torque Sensors

Force/torque sensors provide information about external forces acting on the robot:

```python
class ForceTorqueSensor:
    def __init__(self, sensor_frame):
        self.frame = sensor_frame
        self.force = np.zeros(3)  # [Fx, Fy, Fz] in Newtons
        self.torque = np.zeros(3)  # [Mx, My, Mz] in Nm
        self.bias = np.zeros(6)  # Bias values for calibration

    def read_raw_data(self):
        """Read raw force/torque data"""
        # In practice, this would interface with hardware
        raw_data = np.random.normal(0, 0.1, 6)  # Placeholder
        return raw_data

    def get_calibrated_data(self):
        """Get bias-corrected force/torque measurements"""
        raw_data = self.read_raw_data()
        calibrated_data = raw_data - self.bias
        self.force = calibrated_data[:3]
        self.torque = calibrated_data[3:]
        return self.force, self.torque

    def detect_contact(self, force_threshold=5.0, torque_threshold=1.0):
        """Detect if sensor is in contact with environment"""
        force_magnitude = np.linalg.norm(self.force)
        torque_magnitude = np.linalg.norm(self.torque)
        return (force_magnitude > force_threshold or
                torque_magnitude > torque_threshold)

    def get_wrench(self):
        """Return 6D wrench (force + torque)"""
        return np.concatenate([self.force, self.torque])
```

## Exteroceptive Sensing Systems

### Vision Systems

Cameras provide rich visual information for humanoid robots:

```python
import cv2
import numpy as np

class VisionSystem:
    def __init__(self, camera_id=0, resolution=(640, 480)):
        self.camera_id = camera_id
        self.resolution = resolution
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])

        # Camera intrinsic parameters (example values)
        self.fx = resolution[0] / 2  # Focal length x
        self.fy = resolution[1] / 2  # Focal length y
        self.cx = resolution[0] / 2  # Principal point x
        self.cy = resolution[1] / 2  # Principal point y

    def capture_image(self):
        """Capture and return image"""
        ret, frame = self.cap.read()
        if ret:
            return frame
        else:
            return None

    def detect_faces(self, image):
        """Detect faces in image using Haar cascades"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        return faces

    def detect_objects(self, image):
        """Detect objects using color-based segmentation"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color (example)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        # Create mask
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                objects.append({'bbox': (x, y, w, h), 'area': cv2.contourArea(contour)})

        return objects

    def depth_from_stereo(self, left_image, right_image):
        """Compute depth from stereo camera pair"""
        # Create stereo matcher
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16*10,  # Must be divisible by 16
            blockSize=5,
            P1=8 * 3 * 5**2,
            P2=32 * 3 * 5**2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=0,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # Compute disparity
        disparity = stereo.compute(left_image, right_image).astype(np.float32) / 16.0

        # Convert disparity to depth
        baseline = 0.1  # Baseline distance between cameras (m)
        depth = (self.fx * baseline) / (disparity + 1e-6)  # Avoid division by zero

        return depth
```

### Range Sensors

Range sensors provide distance measurements to obstacles:

```python
class RangeSensor:
    def __init__(self, sensor_type="lidar", num_beams=360):
        self.type = sensor_type
        self.num_beams = num_beams
        self.angles = np.linspace(0, 2*np.pi, num_beams, endpoint=False)
        self.ranges = np.zeros(num_beams)
        self.max_range = 10.0  # meters
        self.min_range = 0.1   # meters

    def update_ranges(self, new_ranges):
        """Update range measurements"""
        self.ranges = np.clip(new_ranges, self.min_range, self.max_range)

    def get_obstacle_distances(self, angle_min=-np.pi/4, angle_max=np.pi/4):
        """Get distances in a specific angular sector (e.g., front of robot)"""
        angle_indices = np.where((self.angles >= angle_min) & (self.angles <= angle_max))[0]
        return self.angles[angle_indices], self.ranges[angle_indices]

    def detect_obstacles(self, threshold=2.0):
        """Detect obstacles closer than threshold"""
        obstacle_angles = self.angles[self.ranges < threshold]
        obstacle_distances = self.ranges[self.ranges < threshold]
        return obstacle_angles, obstacle_distances

    def create_occupancy_grid(self, grid_size=(100, 100), resolution=0.1):
        """Create occupancy grid from range data"""
        grid = np.zeros(grid_size)  # 0 = unknown, 1 = free, -1 = occupied

        robot_x, robot_y = grid_size[0] // 2, grid_size[1] // 2

        for i, (angle, distance) in enumerate(zip(self.angles, self.ranges)):
            if distance >= self.max_range:
                continue  # No return, mark as free to max range

            # Calculate endpoint of beam
            end_x = robot_x + int((distance * np.cos(angle)) / resolution)
            end_y = robot_y + int((distance * np.sin(angle)) / resolution)

            # Bresenham's algorithm to mark free space
            self._mark_free_space(grid, robot_x, robot_y, end_x, end_y)

            # Mark endpoint as occupied if it's a valid obstacle
            if distance < self.max_range:
                if 0 <= end_x < grid_size[0] and 0 <= end_y < grid_size[1]:
                    grid[end_x, end_y] = -1  # Occupied

        return grid

    def _mark_free_space(self, grid, x0, y0, x1, y1):
        """Mark free space along a ray using Bresenham's algorithm"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        err = dx - dy

        while True:
            if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
                grid[x, y] = 1  # Mark as free
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
```

### Tactile Sensing

Tactile sensors provide information about contact and pressure:

```python
class TactileSensor:
    def __init__(self, sensor_location, num_taxels=64):
        self.location = sensor_location  # e.g., "hand", "foot", "torso"
        self.num_taxels = num_taxels
        self.pressure_map = np.zeros(num_taxels)
        self.temperature = np.zeros(num_taxels)
        self.contact_detected = np.zeros(num_taxels, dtype=bool)

    def update_sensor_data(self, pressure_values, temp_values):
        """Update tactile sensor readings"""
        self.pressure_map = np.clip(pressure_values, 0, 100)  # 0-100 N/m^2
        self.temperature = temp_values
        self.contact_detected = self.pressure_map > 0.1  # Threshold for contact

    def get_contact_info(self):
        """Get contact information"""
        contact_indices = np.where(self.contact_detected)[0]
        contact_pressures = self.pressure_map[contact_indices]
        return contact_indices, contact_pressures

    def get_contact_center_of_pressure(self):
        """Calculate center of pressure for contact area"""
        if not np.any(self.contact_detected):
            return None

        # For simplicity, assume 2D taxel array
        # In practice, taxel positions would be known
        contact_indices = np.where(self.contact_detected)[0]
        pressures = self.pressure_map[contact_indices]

        # Calculate weighted average position
        total_force = np.sum(pressures)
        if total_force == 0:
            return None

        # Convert linear indices to 2D coordinates (assuming 8x8 grid)
        rows = contact_indices // 8
        cols = contact_indices % 8

        center_row = np.sum(rows * pressures) / total_force
        center_col = np.sum(cols * pressures) / total_force

        return center_row, center_col

    def detect_slip(self, time_window=0.1):
        """Detect potential slip based on pressure changes"""
        # This would require temporal analysis
        # For now, return a simple slip indicator
        pressure_changes = np.diff(self.pressure_map) if len(self.pressure_map) > 1 else np.zeros_like(self.pressure_map)
        return np.any(np.abs(pressure_changes) > 10)  # Threshold for slip detection
```

## Sensor Integration and Synchronization

### Time Synchronization

Proper timing is crucial for sensor fusion:

```python
import time
from collections import deque

class SensorSynchronizer:
    def __init__(self, max_buffer_size=100):
        self.sensors = {}
        self.buffers = {}
        self.max_buffer_size = max_buffer_size
        self.sync_threshold = 0.01  # 10ms sync window

    def register_sensor(self, sensor_name, callback=None):
        """Register a sensor with the synchronizer"""
        self.sensors[sensor_name] = callback
        self.buffers[sensor_name] = deque(maxlen=self.max_buffer_size)

    def add_sensor_data(self, sensor_name, data, timestamp=None):
        """Add sensor data with timestamp"""
        if timestamp is None:
            timestamp = time.time()

        self.buffers[sensor_name].append((timestamp, data))

    def get_synchronized_data(self):
        """Get time-synchronized sensor data"""
        if not self.buffers:
            return None

        # Find the latest common timestamp
        latest_times = {}
        for sensor_name, buffer in self.buffers.items():
            if buffer:
                latest_times[sensor_name] = buffer[-1][0]

        if not latest_times:
            return None

        # Find the oldest of the latest times (most conservative sync)
        sync_time = min(latest_times.values())

        # Get data closest to sync time for each sensor
        synchronized_data = {}
        for sensor_name, buffer in self.buffers.items():
            if not buffer:
                continue

            # Find closest timestamp within threshold
            closest_data = None
            closest_time_diff = float('inf')

            for ts, data in buffer:
                time_diff = abs(ts - sync_time)
                if time_diff < closest_time_diff and time_diff < self.sync_threshold:
                    closest_time_diff = time_diff
                    closest_data = data

            if closest_data is not None:
                synchronized_data[sensor_name] = (closest_data, sync_time)

        return synchronized_data if synchronized_data else None
```

### Sensor Calibration

Calibration ensures accurate sensor readings:

```python
class SensorCalibrator:
    def __init__(self):
        self.calibration_data = {}
        self.is_calibrated = {}

    def collect_calibration_data(self, sensor_name, raw_data, reference_data):
        """Collect data for calibration"""
        if sensor_name not in self.calibration_data:
            self.calibration_data[sensor_name] = {'raw': [], 'ref': []}

        self.calibration_data[sensor_name]['raw'].append(raw_data)
        self.calibration_data[sensor_name]['ref'].append(reference_data)

    def compute_calibration_parameters(self, sensor_name):
        """Compute calibration parameters using least squares"""
        if sensor_name not in self.calibration_data:
            return None

        raw_data = np.array(self.calibration_data[sensor_name]['raw'])
        ref_data = np.array(self.calibration_data[sensor_name]['ref'])

        # For linear calibration: y = ax + b
        # Solve for a and b using least squares
        A = np.vstack([raw_data, np.ones(len(raw_data))]).T
        a, b = np.linalg.lstsq(A, ref_data, rcond=None)[0]

        self.calibration_parameters = {'a': a, 'b': b}
        self.is_calibrated[sensor_name] = True

        return {'a': a, 'b': b}

    def apply_calibration(self, sensor_name, raw_value):
        """Apply calibration to raw sensor value"""
        if not self.is_calibrated.get(sensor_name, False):
            return raw_value  # Return raw value if not calibrated

        params = self.calibration_parameters
        return params['a'] * raw_value + params['b']
```

## Sensor Reliability and Fault Detection

### Health Monitoring

Monitoring sensor health is essential for reliable operation:

```python
class SensorHealthMonitor:
    def __init__(self, sensor_names):
        self.sensor_names = sensor_names
        self.reading_history = {name: deque(maxlen=50) for name in sensor_names}
        self.health_status = {name: True for name in sensor_names}

    def update_sensor_reading(self, sensor_name, reading):
        """Update sensor reading and check health"""
        self.reading_history[sensor_name].append(reading)

        # Check for sensor faults
        self.health_status[sensor_name] = self._check_sensor_health(sensor_name)

    def _check_sensor_health(self, sensor_name):
        """Check if sensor is operating normally"""
        if len(self.reading_history[sensor_name]) < 10:
            return True  # Not enough data to assess

        readings = list(self.reading_history[sensor_name])

        # Check for constant values (stuck sensor)
        if len(set(readings)) < len(readings) * 0.1:  # Less than 10% unique values
            return False

        # Check for extreme values
        current = readings[-1]
        mean = np.mean(readings[:-1])  # Exclude current reading
        std = np.std(readings[:-1])

        if std > 0 and abs(current - mean) > 5 * std:  # 5-sigma outlier
            return False

        # Check for sudden jumps
        if len(readings) > 1:
            recent_change = abs(readings[-1] - readings[-2])
            historical_changes = [abs(readings[i] - readings[i-1])
                                for i in range(1, len(readings)-1)]
            if historical_changes:
                mean_change = np.mean(historical_changes)
                if mean_change > 0 and recent_change > 10 * mean_change:
                    return False

        return True

    def get_unhealthy_sensors(self):
        """Get list of unhealthy sensors"""
        return [name for name, healthy in self.health_status.items() if not healthy]
```

## Conclusion

Sensory modalities form the foundation of humanoid robot perception, providing the information necessary for intelligent behavior. Proprioceptive sensors enable the robot to understand its own state, while exteroceptive sensors provide awareness of the external environment. Proper sensor integration, calibration, and health monitoring are essential for reliable operation.

The next section will explore computer vision techniques specifically designed for humanoid robots, building on these fundamental sensing concepts to enable visual perception and object recognition.