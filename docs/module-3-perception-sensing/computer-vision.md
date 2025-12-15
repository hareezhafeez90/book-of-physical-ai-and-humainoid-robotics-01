# Computer Vision for Humanoid Robots

## Introduction: Visual Perception in Physical AI

Computer vision enables humanoid robots to interpret and understand their visual environment, providing crucial information for navigation, manipulation, and human interaction. Unlike traditional computer vision applications that process pre-captured images, humanoid robots must perform real-time visual processing while moving and interacting in dynamic environments. This section explores the specialized computer vision techniques required for humanoid robotics.

### Challenges in Humanoid Computer Vision

Humanoid robots face unique challenges in visual perception:

- **Ego-motion**: The robot's own movement affects visual input
- **Real-time constraints**: Processing must occur at video frame rates
- **Dynamic environments**: Scenes change rapidly with moving objects and people
- **Resource limitations**: Embedded systems have limited computational power
- **Safety requirements**: Vision systems must be robust and reliable

## Camera Systems for Humanoid Robots

### Stereo Vision Systems

Stereo vision provides depth information essential for 3D scene understanding:

```python
import cv2
import numpy as np

class StereoVisionSystem:
    def __init__(self, left_camera_params, right_camera_params):
        self.left_cam = self.load_camera_params(left_camera_params)
        self.right_cam = self.load_camera_params(right_camera_params)

        # Stereo rectification parameters
        self.R1, self.R2, self.P1, self.P2, self.Q = None, None, None, None, None
        self.disparity_to_depth_map = None

        # Initialize stereo matcher
        self.stereo_matcher = cv2.StereoSGBM_create(
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

    def load_camera_params(self, params_file):
        """Load camera intrinsic and extrinsic parameters"""
        # In practice, this would load from calibration file
        return params_file

    def stereo_rectify(self):
        """Compute stereo rectification parameters"""
        # This would typically be computed during calibration
        # For this example, we'll use placeholder values
        size = (640, 480)  # Image size

        # Compute rectification transforms
        self.R1, self.R2, self.P1, self.P2, self.Q = cv2.stereoRectify(
            self.left_cam['camera_matrix'], self.left_cam['dist_coeffs'],
            self.right_cam['camera_matrix'], self.right_cam['dist_coeffs'],
            size, self.left_cam['R'], self.left_cam['T'],
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1
        )

    def compute_depth_map(self, left_image, right_image):
        """Compute depth map from stereo pair"""
        # Convert to grayscale if needed
        if len(left_image.shape) == 3:
            left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left_image
            right_gray = right_image

        # Compute disparity
        disparity = self.stereo_matcher.compute(left_gray, right_gray).astype(np.float32)

        # Convert disparity to depth
        # Using the Q matrix from stereoRectify
        # Points in 3D: (X, Y, Z) = (x, y, disparity) * Q
        # We only need the Z component (depth)
        if self.Q is not None:
            # Apply Q matrix transformation
            points_4d = cv2.reprojectImageTo3D(disparity, self.Q)
            depth_map = points_4d[:, :, 2]  # Z component is depth
        else:
            # Use simplified formula if Q matrix not available
            baseline = 0.1  # Distance between cameras (m)
            focal_length = self.left_cam['camera_matrix'][0, 0]
            depth_map = (baseline * focal_length) / (disparity / 16.0 + 1e-6)

        return depth_map

    def get_3d_point(self, u, v, disparity):
        """Convert image coordinates + disparity to 3D world coordinates"""
        if self.Q is None:
            return None

        # Use Q matrix to convert disparity to 3D coordinates
        disparity_value = disparity[v, u] if disparity.ndim > 1 else disparity
        if disparity_value <= 0:
            return None

        # Reproject to 3D
        points_4d = cv2.reprojectImageTo3D(np.array([[disparity_value]]), self.Q)
        x, y, z = points_4d[0, 0]
        return np.array([x, y, z])
```

### RGB-D Vision Systems

RGB-D cameras provide both color and depth information:

```python
class RGBDVisionSystem:
    def __init__(self, camera_params):
        self.camera_params = camera_params
        self.fx = camera_params['fx']
        self.fy = camera_params['fy']
        self.cx = camera_params['cx']
        self.cy = camera_params['cy']

    def depth_to_point_cloud(self, depth_image, color_image=None):
        """Convert depth image to 3D point cloud"""
        height, width = depth_image.shape
        points = []
        colors = []

        for v in range(height):
            for u in range(width):
                z = depth_image[v, u] / 1000.0  # Convert mm to m
                if z > 0 and z < 5.0:  # Valid depth range
                    # Convert pixel coordinates to 3D world coordinates
                    x = (u - self.cx) * z / self.fx
                    y = (v - self.cy) * z / self.fy

                    points.append([x, y, z])

                    if color_image is not None:
                        color = color_image[v, u]
                        colors.append(color)

        return np.array(points), np.array(colors) if colors else None

    def segment_objects(self, depth_image, color_image):
        """Segment objects based on depth and color"""
        # Create mask for valid depth values
        valid_depth = (depth_image > 100) & (depth_image < 3000)  # 10cm to 3m
        valid_depth = valid_depth.astype(np.uint8) * 255

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        valid_depth = cv2.morphologyEx(valid_depth, cv2.MORPH_CLOSE, kernel)
        valid_depth = cv2.morphologyEx(valid_depth, cv2.MORPH_OPEN, kernel)

        # Find connected components (potential objects)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(valid_depth)

        objects = []
        for i in range(1, num_labels):  # Skip background (label 0)
            # Create mask for current component
            mask = (labels == i).astype(np.uint8) * 255

            # Calculate object properties
            area = stats[i, cv2.CC_STAT_AREA]
            if area > 1000:  # Filter small components
                # Calculate bounding box
                x, y, w, h = stats[i, cv2.CC_STAT_LEFT:cv2.CC_STAT_TOP+2]

                # Calculate average depth in object region
                avg_depth = np.mean(depth_image[labels == i]) / 1000.0  # Convert to meters

                objects.append({
                    'bbox': (x, y, w, h),
                    'area': area,
                    'avg_depth': avg_depth,
                    'centroid': centroids[i],
                    'mask': mask
                })

        return objects

    def detect_planes(self, depth_image):
        """Detect planar surfaces using RANSAC"""
        # Convert depth image to point cloud
        points, _ = self.depth_to_point_cloud(depth_image)

        if len(points) < 1000:  # Need sufficient points for RANSAC
            return []

        # RANSAC plane detection
        planes = []
        remaining_points = points.copy()

        for _ in range(5):  # Find up to 5 planes
            if len(remaining_points) < 100:
                break

            # Random sample for RANSAC
            best_inliers = []
            best_plane = None

            for _ in range(100):  # RANSAC iterations
                # Sample 3 random points
                sample_indices = np.random.choice(len(remaining_points), 3, replace=False)
                sample_points = remaining_points[sample_indices]

                # Fit plane to 3 points
                # Plane equation: ax + by + cz + d = 0
                v1 = sample_points[1] - sample_points[0]
                v2 = sample_points[2] - sample_points[0]
                normal = np.cross(v1, v2)
                if np.linalg.norm(normal) < 1e-6:
                    continue

                normal = normal / np.linalg.norm(normal)
                d = -np.dot(normal, sample_points[0])

                # Count inliers
                distances = np.abs(np.dot(remaining_points, normal) + d)
                inliers = remaining_points[distances < 0.02]  # 2cm threshold

                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_plane = (normal, d)

            if best_plane is not None and len(best_inliers) > 500:
                planes.append({
                    'normal': best_plane[0],
                    'd': best_plane[1],
                    'inliers': best_inliers
                })

                # Remove inliers from remaining points
                distances = np.abs(np.dot(remaining_points, best_plane[0]) + best_plane[1])
                remaining_points = remaining_points[distances >= 0.02]

        return planes
```

## Object Detection and Recognition

### Deep Learning-Based Object Detection

Modern humanoid robots increasingly use deep learning for object detection:

```python
import torch
import torchvision.transforms as transforms
from torchvision.models.detection import fasterrcnn_resnet50_fpn

class DeepObjectDetector:
    def __init__(self, model_path=None, confidence_threshold=0.5):
        self.confidence_threshold = confidence_threshold

        # Load pre-trained model
        if model_path:
            self.model = torch.load(model_path)
        else:
            self.model = fasterrcnn_resnet50_fpn(pretrained=True)

        self.model.eval()
        self.transform = transforms.Compose([
            transforms.ToTensor(),
        ])

        # COCO dataset class names (first 20 for example)
        self.class_names = [
            '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow'
        ]

    def detect_objects(self, image):
        """Detect objects in image using deep learning model"""
        # Preprocess image
        image_tensor = self.transform(image).unsqueeze(0)

        with torch.no_grad():
            predictions = self.model(image_tensor)

        # Extract results
        boxes = predictions[0]['boxes'].cpu().numpy()
        labels = predictions[0]['labels'].cpu().numpy()
        scores = predictions[0]['scores'].cpu().numpy()

        # Filter by confidence threshold
        valid_indices = scores >= self.confidence_threshold
        boxes = boxes[valid_indices]
        labels = labels[valid_indices]
        scores = scores[valid_indices]

        # Create detection results
        detections = []
        for box, label, score in zip(boxes, labels, scores):
            detections.append({
                'bbox': box,  # [x1, y1, x2, y2]
                'label': self.class_names[label] if label < len(self.class_names) else f'object_{label}',
                'confidence': score,
                'center': ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)  # Center coordinates
            })

        return detections

    def track_objects(self, image, previous_detections):
        """Simple object tracking using detection matching"""
        current_detections = self.detect_objects(image)

        # For each previous detection, find the closest current detection
        tracked_objects = []
        used_current = set()

        for prev_det in previous_detections:
            min_dist = float('inf')
            best_match = None

            for i, curr_det in enumerate(current_detections):
                if i in used_current:
                    continue

                # Calculate distance between centers
                dist = np.sqrt((prev_det['center'][0] - curr_det['center'][0])**2 +
                              (prev_det['center'][1] - curr_det['center'][1])**2)

                if dist < min_dist and dist < 50:  # 50 pixel threshold
                    min_dist = dist
                    best_match = i

            if best_match is not None:
                tracked_objects.append({
                    **current_detections[best_match],
                    'id': prev_det.get('id', len(tracked_objects))
                })
                used_current.add(best_match)
            else:
                # Object lost
                tracked_objects.append({
                    **prev_det,
                    'tracked': False
                })

        # Add new detections
        for i, det in enumerate(current_detections):
            if i not in used_current:
                tracked_objects.append({
                    **det,
                    'id': len(tracked_objects)
                })

        return tracked_objects
```

### Feature-Based Object Recognition

Traditional computer vision methods remain important for certain applications:

```python
class FeatureBasedRecognizer:
    def __init__(self):
        self.sift = cv2.SIFT_create()
        self.bf_matcher = cv2.BFMatcher()
        self.known_objects = {}  # Dictionary of known object descriptors

    def extract_features(self, image):
        """Extract SIFT features from image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        keypoints, descriptors = self.sift.detectAndCompute(gray, None)
        return keypoints, descriptors

    def register_object(self, object_name, template_image):
        """Register a known object with its features"""
        keypoints, descriptors = self.extract_features(template_image)
        self.known_objects[object_name] = {
            'keypoints': keypoints,
            'descriptors': descriptors
        }

    def recognize_object(self, image, min_matches=10):
        """Recognize objects in image using feature matching"""
        img_keypoints, img_descriptors = self.extract_features(image)

        if img_descriptors is None:
            return []

        recognized_objects = []

        for obj_name, obj_data in self.known_objects.items():
            if obj_data['descriptors'] is None:
                continue

            # Match features
            matches = self.bf_matcher.knnMatch(
                obj_data['descriptors'], img_descriptors, k=2
            )

            # Apply Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)

            if len(good_matches) >= min_matches:
                # Estimate object location using homography
                src_pts = np.float32([obj_data['keypoints'][m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([img_keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                # Find homography matrix
                if len(src_pts) >= 4:
                    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                    if H is not None:
                        recognized_objects.append({
                            'name': obj_name,
                            'matches': len(good_matches),
                            'confidence': len(good_matches) / min_matches
                        })

        return recognized_objects

    def estimate_object_pose(self, object_name, image):
        """Estimate 3D pose of known object"""
        if object_name not in self.known_objects:
            return None

        obj_data = self.known_objects[object_name]
        img_keypoints, img_descriptors = self.extract_features(image)

        if img_descriptors is None:
            return None

        # Match features
        matches = self.bf_matcher.knnMatch(
            obj_data['descriptors'], img_descriptors, k=2
        )

        # Apply Lowe's ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)

        if len(good_matches) < 10:
            return None

        # Get corresponding points
        obj_pts = np.float32([obj_data['keypoints'][m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        img_pts = np.float32([img_keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # For pose estimation, we need 3D object points
        # This requires a 3D model of the object
        # For now, we'll return the 2D transformation
        H, mask = cv2.findHomography(obj_pts, img_pts, cv2.RANSAC, 5.0)

        return {
            'homography': H,
            'matches': len(good_matches),
            'inliers': np.sum(mask) if mask is not None else 0
        }
```

## Human Detection and Tracking

### Person Detection and Pose Estimation

Humanoid robots need to detect and understand human poses:

```python
class HumanDetector:
    def __init__(self):
        # Load Haar cascade for face detection
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # For pose estimation, we'll use a simple body part detection approach
        # In practice, you might use OpenPose, MediaPipe, or similar
        self.body_part_cascades = {
            'upper_body': cv2.CascadeClassifier('upper_body_cascade.xml')  # Placeholder
        }

    def detect_faces(self, image):
        """Detect faces in image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )
        return faces

    def detect_humans_simple(self, image):
        """Simple human detection using HOG and SVM"""
        # HOG descriptor
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Detect people
        boxes, weights = hog.detectMultiScale(image, winStride=(8, 8))

        humans = []
        for (x, y, w, h), weight in zip(boxes, weights):
            humans.append({
                'bbox': (x, y, w, h),
                'confidence': float(weight),
                'center': (x + w//2, y + h//2)
            })

        return humans

    def estimate_head_orientation(self, face_bbox, image):
        """Estimate head orientation from face bounding box"""
        x, y, w, h = face_bbox

        # Extract face region
        face_region = image[y:y+h, x:x+w]

        # Simple method: track eye positions
        # In practice, use facial landmark detection
        gray_face = cv2.cvtColor(face_region, cv2.COLOR_BGR2GRAY)

        # Use Haar cascades for eyes
        eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')
        eyes = eye_cascade.detectMultiScale(gray_face)

        if len(eyes) >= 2:
            # Calculate relative positions of eyes for orientation estimation
            eye_centers = []
            for (ex, ey, ew, eh) in eyes:
                eye_centers.append((x + ex + ew//2, y + ey + eh//2))

            if len(eye_centers) >= 2:
                # Estimate orientation based on eye positions
                eye1, eye2 = eye_centers[:2]
                dx = eye2[0] - eye1[0]
                dy = eye2[1] - eye1[1]
                angle = np.arctan2(dy, dx) * 180 / np.pi
                return angle

        return 0  # Default: frontal face

    def track_person(self, image, person_id, last_position):
        """Track person using optical flow"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Use Lucas-Kanade optical flow for tracking
        if not hasattr(self, 'lk_params'):
            self.lk_params = dict(
                winSize=(15, 15),
                maxLevel=2,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
            )

        # If this is the first frame for tracking, initialize points
        if not hasattr(self, 'tracked_points') or person_id not in self.tracked_points:
            # Use face or person bounding box center as initial point
            x, y = last_position
            self.tracked_points = {person_id: np.array([[x, y]], dtype=np.float32)}
            return (x, y)

        # Calculate optical flow
        new_points, status, error = cv2.calcOpticalFlowPyrLK(
            getattr(self, 'prev_gray', gray),
            gray,
            self.tracked_points[person_id],
            None,
            **self.lk_params
        )

        # Update tracked points
        if status[0][0] == 1:
            self.tracked_points[person_id] = new_points
            self.prev_gray = gray
            return (int(new_points[0][0][0]), int(new_points[0][0][1]))

        # If tracking failed, return last known position
        return last_position
```

## Visual Servoing

### Image-Based Visual Servoing

Visual servoing enables robots to control their motion based on visual feedback:

```python
class VisualServoController:
    def __init__(self, camera_params, control_gain=1.0):
        self.camera_params = camera_params
        self.K = control_gain  # Control gain
        self.feature_history = []  # Store feature history for stability

    def compute_image_jacobian(self, feature_point, depth):
        """Compute image Jacobian for a 2D feature point"""
        u, v = feature_point
        fx, fy = self.camera_params['fx'], self.camera_params['fy']
        cx, cy = self.camera_params['cx'], self.camera_params['cy']

        # Image Jacobian matrix (2x6) relating image velocity to camera velocity
        # [du/dt, dv/dt]^T = L * [twist]^T
        # where twist = [linear_vel; angular_vel] (6x1)
        L = np.zeros((2, 6))

        # Linear velocity terms
        L[0, 0] = -fx / depth  # du/dtx
        L[0, 1] = 0           # du/dty
        L[0, 2] = (u - cx) / depth * fx / depth  # du/dtz
        L[1, 0] = 0           # dv/dtx
        L[1, 1] = -fy / depth  # dv/dty
        L[1, 2] = (v - cy) / depth * fy / depth  # dv/dtz

        # Angular velocity terms
        L[0, 3] = (u - cx) * (v - cy) / depth  # du/dwx
        L[0, 4] = -(fx + (u - cx)**2 / fx) / depth  # du/dwy
        L[0, 5] = (v - cy) / fx  # du/dwz
        L[1, 3] = fy + (v - cy)**2 / fy  # dv/dwx
        L[1, 4] = -(u - cx) * (v - cy) / fy  # dv/dwy
        L[1, 5] = -(u - cx) / fy  # dv/dwz

        return L

    def image_based_servo(self, current_features, desired_features, depth_estimates):
        """Compute camera velocity to move features to desired positions"""
        if len(current_features) != len(desired_features):
            raise ValueError("Number of current and desired features must match")

        # Compute feature errors
        errors = []
        for curr, desired in zip(current_features, desired_features):
            error = np.array(desired) - np.array(curr)
            errors.extend(error)

        error_vector = np.array(errors)

        # Compute combined Jacobian
        J = []
        for i, (feature, depth) in enumerate(zip(current_features, depth_estimates)):
            feature_jac = self.compute_image_jacobian(feature, depth)
            J.append(feature_jac)

        J_combined = np.vstack(J)

        # Compute camera velocity command
        # v = -K * J^+ * e (where J^+ is pseudoinverse)
        try:
            J_pinv = np.linalg.pinv(J_combined)
            camera_velocity = -self.K * J_pinv @ error_vector
        except np.linalg.LinAlgError:
            # If Jacobian is singular, use damped least squares
            damping = 0.01
            J_pinv = J_combined.T @ np.linalg.inv(J_combined @ J_combined.T + damping**2 * np.eye(J_combined.shape[0]))
            camera_velocity = -self.K * J_pinv @ error_vector

        return camera_velocity, error_vector

    def position_based_servo(self, current_pose, desired_pose):
        """Compute velocity to move camera to desired pose"""
        # Position error
        pos_error = desired_pose[:3] - current_pose[:3]

        # Orientation error (using angle-axis representation)
        current_rot = current_pose[3:]
        desired_rot = desired_pose[3:]

        # Convert to rotation matrices for easier computation
        # For simplicity, we'll use a basic approach
        # In practice, use proper rotation representation
        rot_error = desired_rot - current_rot

        # Combine position and orientation errors
        pose_error = np.concatenate([pos_error, rot_error])

        # Compute velocity command
        velocity = -self.K * pose_error

        return velocity, pose_error
```

## Real-Time Performance Optimization

### Efficient Processing Pipelines

Real-time computer vision requires careful optimization:

```python
import threading
import queue
from collections import deque

class RealTimeVisionProcessor:
    def __init__(self, max_fps=30):
        self.max_fps = max_fps
        self.frame_interval = 1.0 / max_fps
        self.input_queue = queue.Queue(maxsize=2)  # Only keep most recent frames
        self.output_queue = queue.Queue(maxsize=10)
        self.running = False
        self.processing_thread = None

        # Processing pipeline stages
        self.pipeline_stages = {
            'preprocessing': self.preprocess_frame,
            'detection': self.detect_objects,
            'tracking': self.track_objects,
            'postprocessing': self.postprocess_results
        }

    def start_processing(self):
        """Start the vision processing pipeline"""
        self.running = True
        self.processing_thread = threading.Thread(target=self._processing_loop)
        self.processing_thread.start()

    def stop_processing(self):
        """Stop the vision processing pipeline"""
        self.running = False
        if self.processing_thread:
            self.processing_thread.join()

    def _processing_loop(self):
        """Main processing loop"""
        last_time = time.time()

        while self.running:
            start_time = time.time()

            try:
                # Get frame from input queue
                frame = self.input_queue.get_nowait()
            except queue.Empty:
                time.sleep(0.001)  # Brief pause if no frame available
                continue

            # Process frame through pipeline
            results = self.process_frame(frame)

            # Add results to output queue
            try:
                self.output_queue.put_nowait(results)
            except queue.Full:
                pass  # Drop old results if output queue is full

            # Maintain frame rate
            elapsed = time.time() - start_time
            sleep_time = max(0, self.frame_interval - elapsed)
            time.sleep(sleep_time)

    def process_frame(self, frame):
        """Process a single frame through the pipeline"""
        # Preprocessing
        processed_frame = self.pipeline_stages['preprocessing'](frame)

        # Object detection
        detections = self.pipeline_stages['detection'](processed_frame)

        # Object tracking (if needed)
        tracked_objects = self.pipeline_stages['tracking'](detections)

        # Postprocessing
        results = self.pipeline_stages['postprocessing'](tracked_objects)

        return results

    def preprocess_frame(self, frame):
        """Preprocess frame for faster processing"""
        # Resize frame to reduce computational load
        height, width = frame.shape[:2]
        scale_factor = min(1.0, 640.0 / max(height, width))  # Max 640px in any dimension
        new_width = int(width * scale_factor)
        new_height = int(height * scale_factor)

        resized_frame = cv2.resize(frame, (new_width, new_height))
        return resized_frame

    def detect_objects(self, frame):
        """Detect objects in frame (placeholder)"""
        # In practice, this would call the actual detection algorithm
        return []

    def track_objects(self, detections):
        """Track objects across frames"""
        # Implement tracking logic here
        return detections

    def postprocess_results(self, results):
        """Postprocess detection results"""
        # Add any final processing steps
        return results

    def submit_frame(self, frame):
        """Submit a frame for processing"""
        try:
            self.input_queue.put_nowait(frame)
        except queue.Full:
            pass  # Drop frame if queue is full (older frames are more important)
```

## Integration with ROS 2

### ROS 2 Computer Vision Node

Integrating computer vision with ROS 2 enables system-wide perception:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Initialize OpenCV bridge
        self.cv_bridge = CvBridge()

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/vision/detections',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/vision/debug_image',
            10
        )

        # Initialize vision components
        self.object_detector = DeepObjectDetector()
        self.human_detector = HumanDetector()
        self.camera_params = None

        self.get_logger().info('Vision node initialized')

    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Perform object detection
        detections = self.object_detector.detect_objects(cv_image)

        # Perform human detection
        humans = self.human_detector.detect_humans_simple(cv_image)

        # Combine results
        all_detections = []
        for det in detections:
            detection_msg = self.create_detection_msg(det, msg.header)
            all_detections.append(detection_msg)

        for human in humans:
            human_det = {
                'bbox': human['bbox'],
                'label': 'person',
                'confidence': human['confidence'],
                'center': human['center']
            }
            detection_msg = self.create_detection_msg(human_det, msg.header)
            all_detections.append(detection_msg)

        # Create and publish detection array
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        detection_array.detections = all_detections

        self.detection_pub.publish(detection_array)

        # Create debug image with detections
        debug_image = self.draw_detections(cv_image, detections, humans)
        debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)

    def camera_info_callback(self, msg):
        """Update camera parameters from camera info"""
        self.camera_params = {
            'fx': msg.k[0],  # Focal length x
            'fy': msg.k[4],  # Focal length y
            'cx': msg.k[2],  # Principal point x
            'cy': msg.k[5],  # Principal point y
            'width': msg.width,
            'height': msg.height
        }

    def create_detection_msg(self, detection, header):
        """Create vision_msgs/Detection2D message"""
        detection_msg = Detection2D()
        detection_msg.header = header

        # Bounding box
        bbox = detection['bbox']
        detection_msg.bbox.center.x = (bbox[0] + bbox[2]) / 2.0
        detection_msg.bbox.center.y = (bbox[1] + bbox[3]) / 2.0
        detection_msg.bbox.size_x = bbox[2]
        detection_msg.bbox.size_y = bbox[3]

        # Results (classification)
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = detection['label']
        result.hypothesis.score = detection['confidence']
        detection_msg.results = [result]

        return detection_msg

    def draw_detections(self, image, detections, humans):
        """Draw detection results on image for debugging"""
        result_image = image.copy()

        # Draw object detections
        for det in detections:
            bbox = det['bbox']
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(result_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(result_image, f"{det['label']}: {det['confidence']:.2f}",
                       (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw human detections
        for human in humans:
            x, y, w, h = human['bbox']
            cv2.rectangle(result_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(result_image, f"person: {human['confidence']:.2f}",
                       (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return result_image

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()

    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

Computer vision systems provide humanoid robots with the ability to perceive and understand their visual environment. From stereo vision and RGB-D sensing to deep learning-based object detection and visual servoing, these technologies enable robots to navigate, manipulate objects, and interact with humans effectively.

The integration of computer vision with ROS 2 and real-time processing requirements presents unique challenges that require careful consideration of computational efficiency, sensor fusion, and system reliability. The next section will explore tactile sensing and haptic feedback systems that provide robots with the sense of touch.