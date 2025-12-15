# Environmental Perception and Mapping

## Introduction: Understanding the Surroundings

Environmental perception enables humanoid robots to understand and navigate their surroundings, creating spatial representations that support navigation, manipulation, and safe interaction. This section explores the algorithms and techniques that allow robots to perceive and map their environment, detect obstacles, and understand spatial relationships in 3D space.

### The Challenge of Environmental Perception

Humanoid robots face unique challenges in environmental perception:

- **Dynamic Environments**: Humans and objects move continuously
- **Multi-Modal Integration**: Combining vision, LIDAR, and other sensors
- **Real-Time Requirements**: Processing must occur at video frame rates
- **Uncertainty Management**: Dealing with sensor noise and occlusions
- **Scale Variations**: From detailed manipulation to large-scale navigation

## 3D Scene Understanding

### Point Cloud Processing

Point clouds from RGB-D cameras and LIDAR provide rich 3D spatial information:

```python
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor

class PointCloudProcessor:
    def __init__(self):
        self.points = None
        self.colors = None
        self.normals = None

    def load_point_cloud(self, points, colors=None):
        """Load point cloud data"""
        self.points = np.array(points)
        self.colors = np.array(colors) if colors is not None else None

    def estimate_normals(self, k=10):
        """Estimate surface normals for each point"""
        if self.points is None:
            return None

        normals = []
        for i, point in enumerate(self.points):
            # Find k nearest neighbors
            distances = np.linalg.norm(self.points - point, axis=1)
            nearest_indices = np.argpartition(distances, k+1)[:k+1]
            nearest_points = self.points[nearest_indices[1:]]  # Exclude the point itself

            # Calculate covariance matrix
            cov_matrix = np.cov(nearest_points.T)

            # Get eigenvectors
            eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

            # Normal is the eigenvector corresponding to smallest eigenvalue
            normal = eigenvectors[:, 0]
            normals.append(normal)

        self.normals = np.array(normals)
        return self.normals

    def remove_outliers(self, method='statistical', k=20, std_ratio=2.0):
        """Remove outlier points using statistical or radius-based method"""
        if self.points is None:
            return None

        if method == 'statistical':
            # Statistical outlier removal
            distances = []
            for i, point in enumerate(self.points):
                neighbor_distances = np.linalg.norm(self.points - point, axis=1)
                k_nearest = np.partition(neighbor_distances, k+1)[1:k+1]
                avg_distance = np.mean(k_nearest)
                distances.append(avg_distance)

            distances = np.array(distances)
            mean_dist = np.mean(distances)
            std_dist = np.std(distances)

            # Keep points with distance within std_ratio * std_dev
            valid_indices = distances < (mean_dist + std_ratio * std_dist)

        elif method == 'radius':
            # Radius-based outlier removal
            valid_indices = np.ones(len(self.points), dtype=bool)
            for i, point in enumerate(self.points):
                neighbor_count = np.sum(np.linalg.norm(self.points - point, axis=1) < 0.1)  # 10cm radius
                if neighbor_count < 5:  # Require at least 5 neighbors
                    valid_indices[i] = False

        # Filter point cloud
        self.points = self.points[valid_indices]
        if self.colors is not None:
            self.colors = self.colors[valid_indices]
        if self.normals is not None:
            self.normals = self.normals[valid_indices]

        return self.points, self.colors

    def segment_planes(self, distance_threshold=0.01, min_points=100):
        """Segment planar surfaces using RANSAC"""
        if self.points is None:
            return []

        remaining_points = self.points.copy()
        planes = []

        while len(remaining_points) > min_points:
            # Apply RANSAC to find best plane
            best_model = None
            best_inliers = []
            best_score = 0

            for _ in range(100):  # RANSAC iterations
                # Randomly sample 3 points
                sample_indices = np.random.choice(len(remaining_points), 3, replace=False)
                sample_points = remaining_points[sample_indices]

                # Check for degenerate case (collinear points)
                v1 = sample_points[1] - sample_points[0]
                v2 = sample_points[2] - sample_points[0]
                normal = np.cross(v1, v2)
                if np.linalg.norm(normal) < 1e-6:  # Points are collinear
                    continue

                # Create plane equation: ax + by + cz + d = 0
                normal = normal / np.linalg.norm(normal)
                d = -np.dot(normal, sample_points[0])

                # Find inliers
                distances = np.abs(np.dot(remaining_points, normal) + d)
                inliers = remaining_points[distances < distance_threshold]

                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_model = (normal, d)

            if len(best_inliers) >= min_points and best_model is not None:
                planes.append({
                    'model': best_model,
                    'inliers': best_inliers,
                    'center': np.mean(best_inliers, axis=0),
                    'normal': best_model[0],
                    'd': best_model[1]
                })

                # Remove inliers from remaining points
                distances = np.abs(np.dot(remaining_points, best_model[0]) + best_model[1])
                remaining_points = remaining_points[distances >= distance_threshold]
            else:
                break  # No more significant planes found

        return planes

    def segment_objects(self, eps=0.05, min_samples=50):
        """Segment objects using DBSCAN clustering"""
        if self.points is None:
            return []

        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(self.points)
        labels = clustering.labels_

        objects = []
        for label in set(labels):
            if label == -1:  # Noise points
                continue

            # Get points belonging to this cluster
            object_points = self.points[labels == label]
            object_colors = self.colors[labels == label] if self.colors is not None else None

            # Calculate object properties
            center = np.mean(object_points, axis=0)
            bbox_min = np.min(object_points, axis=0)
            bbox_max = np.max(object_points, axis=0)
            dimensions = bbox_max - bbox_min

            objects.append({
                'points': object_points,
                'colors': object_colors,
                'center': center,
                'bbox_min': bbox_min,
                'bbox_max': bbox_max,
                'dimensions': dimensions,
                'label': label
            })

        return objects

    def downsample(self, voxel_size=0.01):
        """Downsample point cloud using voxel grid filter"""
        if self.points is None:
            return None

        # Create voxel grid
        min_bound = np.min(self.points, axis=0)
        max_bound = np.max(self.points, axis=0)
        dims = np.ceil((max_bound - min_bound) / voxel_size).astype(int)

        # Create voxel hash map
        voxel_map = {}
        for i, point in enumerate(self.points):
            voxel_idx = ((point - min_bound) / voxel_size).astype(int)
            voxel_key = tuple(voxel_idx)

            if voxel_key not in voxel_map:
                voxel_map[voxel_key] = {'points': [], 'colors': [] if self.colors is not None else None}

            voxel_map[voxel_key]['points'].append(point)
            if self.colors is not None:
                voxel_map[voxel_key]['colors'].append(self.colors[i])

        # Take centroid of each voxel
        downsampled_points = []
        downsampled_colors = []

        for voxel_data in voxel_map.values():
            centroid = np.mean(voxel_data['points'], axis=0)
            downsampled_points.append(centroid)

            if self.colors is not None:
                color_centroid = np.mean(voxel_data['colors'], axis=0)
                downsampled_colors.append(color_centroid)

        self.points = np.array(downsampled_points)
        if self.colors is not None:
            self.colors = np.array(downsampled_colors)

        return self.points, self.colors
```

### Surface Reconstruction

Creating continuous surfaces from discrete point cloud data:

```python
from scipy.spatial import Delaunay
from scipy.interpolate import griddata

class SurfaceReconstruction:
    def __init__(self):
        self.triangles = None
        self.grid_resolution = 0.01

    def delaunay_triangulation(self, points_2d):
        """Create Delaunay triangulation from 2D points"""
        tri = Delaunay(points_2d)
        return tri.simplices

    def poisson_surface_reconstruction(self, points, normals):
        """Simplified Poisson surface reconstruction"""
        # This is a very simplified version
        # In practice, you'd use more sophisticated algorithms like marching cubes

        if len(points) < 3:
            return None

        # Create a signed distance field (simplified approach)
        # Find bounding box
        min_point = np.min(points, axis=0)
        max_point = np.max(points, axis=0)

        # Create a 3D grid
        grid_size = (int((max_point[0] - min_point[0]) / self.grid_resolution),
                    int((max_point[1] - min_point[1]) / self.grid_resolution),
                    int((max_point[2] - min_point[2]) / self.grid_resolution))

        # For each grid point, calculate distance to nearest surface
        # This is computationally expensive and simplified here
        pass

    def extract_mesh(self, points, method='alpha_shape'):
        """Extract mesh from point cloud using various methods"""
        if method == 'convex_hull':
            from scipy.spatial import ConvexHull
            hull = ConvexHull(points)
            return hull.simplices, hull.points[hull.vertices]

        elif method == 'alpha_shape':
            # Alpha shape is a generalization of convex hull
            # It can handle concave shapes
            return self.alpha_shape(points, alpha=0.02)

        elif method == 'ball_pivoting':
            # Ball pivoting algorithm (simplified)
            return self.ball_pivoting(points, radius=0.02)

    def alpha_shape(self, points, alpha):
        """Compute alpha shape of point cloud"""
        # This would typically use specialized libraries like CGAL
        # For this example, we'll use a simplified approach
        # Create Delaunay triangulation
        tri = Delaunay(points)
        simplices = tri.simplices

        # Filter simplices based on circumsphere radius
        filtered_simplices = []
        for simplex in simplices:
            tetra = points[simplex]
            # Calculate circumsphere radius
            # For a tetrahedron with vertices a, b, c, d
            # The circumsphere radius calculation is complex
            # We'll use a simplified distance check
            centroid = np.mean(tetra, axis=0)
            max_dist = np.max(np.linalg.norm(tetra - centroid, axis=1))
            if max_dist < alpha:
                filtered_simplices.append(simplex)

        return np.array(filtered_simplices) if filtered_simplices else None, points

    def ball_pivoting(self, points, radius):
        """Ball pivoting algorithm (simplified)"""
        # Ball pivoting is complex to implement correctly
        # This is a very simplified version
        # The real algorithm would involve more sophisticated geometry
        pass
```

## Object Detection and Recognition in 3D

### 3D Object Detection

Detecting objects using 3D information:

```python
class Object3DDetector:
    def __init__(self):
        self.voxel_size = 0.05  # 5cm voxels
        self.min_object_size = 0.1  # 10cm minimum
        self.max_object_size = 2.0  # 2m maximum

    def detect_objects_voxel_grid(self, points):
        """Detect objects using voxel grid analysis"""
        # Create voxel grid
        min_bound = np.min(points, axis=0)
        max_bound = np.max(points, axis=0)
        dims = np.ceil((max_bound - min_bound) / self.voxel_size).astype(int)

        # Create occupancy grid
        grid_shape = (dims[0], dims[1], dims[2])
        occupancy_grid = np.zeros(grid_shape, dtype=bool)

        # Fill occupancy grid
        for point in points:
            voxel_idx = ((point - min_bound) / self.voxel_size).astype(int)
            voxel_idx = np.clip(voxel_idx, [0, 0, 0], np.array(grid_shape) - 1)
            occupancy_grid[tuple(voxel_idx)] = True

        # Find connected components in 3D
        from scipy.ndimage import label
        structure = np.ones((3, 3, 3))  # 6-connectivity
        labeled_grid, num_objects = label(occupancy_grid, structure=structure)

        # Extract object properties
        objects = []
        for i in range(1, num_objects + 1):
            # Get indices of this object
            object_indices = np.where(labeled_grid == i)
            object_points = points[
                (labeled_grid == i)
            ]

            # Convert voxel indices back to world coordinates
            world_points = []
            for idx in zip(*object_indices):
                world_point = min_bound + np.array(idx) * self.voxel_size
                world_points.append(world_point)

            world_points = np.array(world_points)

            if len(world_points) > 10:  # Minimum number of points
                center = np.mean(world_points, axis=0)
                bbox_min = np.min(world_points, axis=0)
                bbox_max = np.max(world_points, axis=0)
                dimensions = bbox_max - bbox_min

                if (self.min_object_size < dimensions[0] < self.max_object_size and
                    self.min_object_size < dimensions[1] < self.max_object_size and
                    self.min_object_size < dimensions[2] < self.max_object_size):

                    objects.append({
                        'points': world_points,
                        'center': center,
                        'bbox': (bbox_min, bbox_max),
                        'dimensions': dimensions,
                        'num_points': len(world_points)
                    })

        return objects

    def estimate_object_pose(self, object_points):
        """Estimate object pose (position and orientation)"""
        if len(object_points) < 4:
            return None

        # Principal Component Analysis for orientation
        centered_points = object_points - np.mean(object_points, axis=0)
        cov_matrix = np.cov(centered_points.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

        # Sort eigenvectors by eigenvalues (descending)
        sort_idx = np.argsort(eigenvalues)[::-1]
        eigenvectors = eigenvectors[:, sort_idx]

        # The eigenvectors form the rotation matrix
        # (They might need to be adjusted for proper orientation)
        if np.linalg.det(eigenvectors) < 0:
            eigenvectors[:, -1] = -eigenvectors[:, -1]  # Ensure right-handed coordinate system

        # Return pose: [x, y, z, qw, qx, qy, qz]
        position = np.mean(object_points, axis=0)
        rotation_matrix = eigenvectors
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

        return np.concatenate([position, quaternion])

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion"""
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s

        return np.array([qw, qx, qy, qz])

    def classify_object_shape(self, dimensions):
        """Classify object shape based on dimensions"""
        # Sort dimensions
        sorted_dims = np.sort(dimensions)

        if sorted_dims[2] / sorted_dims[0] > 10:  # Very elongated
            return 'rod', 0.8
        elif sorted_dims[2] / sorted_dims[1] < 1.5 and sorted_dims[1] / sorted_dims[0] < 1.5:
            # All dimensions similar
            return 'cube', 0.9
        elif sorted_dims[2] / sorted_dims[1] > 3 and sorted_dims[1] / sorted_dims[0] < 3:
            # Flat object
            return 'disc', 0.8
        else:
            return 'irregular', 0.6

    def track_objects(self, current_objects, previous_objects, max_distance=0.1):
        """Simple object tracking by nearest neighbor matching"""
        if not previous_objects:
            # Assign new IDs to current objects
            for i, obj in enumerate(current_objects):
                obj['id'] = i
            return current_objects

        tracked_objects = []
        used_previous = set()

        for curr_obj in current_objects:
            min_distance = float('inf')
            best_match = None

            for j, prev_obj in enumerate(previous_objects):
                if j in used_previous:
                    continue

                dist = np.linalg.norm(curr_obj['center'] - prev_obj['center'])
                if dist < min_distance and dist < max_distance:
                    min_distance = dist
                    best_match = j

            if best_match is not None:
                # Update tracked object
                tracked_obj = current_objects[current_objects.index(curr_obj)]
                tracked_obj['id'] = previous_objects[best_match]['id']
                tracked_obj['velocity'] = (tracked_obj['center'] -
                                         previous_objects[best_match]['center']) / 0.1  # dt = 0.1s
                used_previous.add(best_match)
            else:
                # New object
                tracked_obj = curr_obj
                tracked_obj['id'] = len(previous_objects) + len(tracked_objects)

            tracked_objects.append(tracked_obj)

        return tracked_objects
```

### Feature-Based 3D Recognition

Recognizing objects using geometric features:

```python
class FeatureBased3DRecognizer:
    def __init__(self):
        self.reference_features = {}
        self.feature_extractor = PointCloudProcessor()

    def extract_geometric_features(self, points):
        """Extract geometric features for object recognition"""
        features = {}

        # Statistical features
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid

        # Covariance-based features
        cov_matrix = np.cov(centered_points.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        eigenvalues = np.sort(eigenvalues)[::-1]  # Sort in descending order

        # Shape descriptors
        features['linearity'] = (eigenvalues[0] - eigenvalues[1]) / eigenvalues[0]
        features['planarity'] = (eigenvalues[1] - eigenvalues[2]) / eigenvalues[0]
        features['scattering'] = eigenvalues[2] / eigenvalues[0]
        features['omnivariance'] = np.cbrt(eigenvalues[0] * eigenvalues[1] * eigenvalues[2])
        features['anisotropy'] = (eigenvalues[0] - eigenvalues[2]) / eigenvalues[0]
        features['eigenentropy'] = -np.sum(eigenvalues / np.sum(eigenvalues) *
                                          np.log(eigenvalues / np.sum(eigenvalues) + 1e-6))

        # Size features
        bbox_min = np.min(points, axis=0)
        bbox_max = np.max(points, axis=0)
        dimensions = bbox_max - bbox_min
        features['volume'] = np.prod(dimensions)
        features['surface_area'] = 2 * (dimensions[0]*dimensions[1] +
                                       dimensions[1]*dimensions[2] +
                                       dimensions[0]*dimensions[2])
        features['dimensions'] = dimensions

        # Density features
        convex_hull_volume = self.estimate_convex_hull_volume(points)
        features['compactness'] = features['volume'] / (convex_hull_volume + 1e-6)

        return features

    def estimate_convex_hull_volume(self, points):
        """Estimate volume using convex hull (simplified)"""
        try:
            from scipy.spatial import ConvexHull
            hull = ConvexHull(points)
            return hull.volume
        except:
            # Fallback: use bounding box volume
            bbox_min = np.min(points, axis=0)
            bbox_max = np.max(points, axis=0)
            return np.prod(bbox_max - bbox_min)

    def register_object(self, object_name, point_cloud):
        """Register a known object with its features"""
        features = self.extract_geometric_features(point_cloud)
        self.reference_features[object_name] = features

    def recognize_object(self, point_cloud, threshold=0.3):
        """Recognize object by comparing features with registered objects"""
        if not self.reference_features:
            return None, 0.0

        query_features = self.extract_geometric_features(point_cloud)

        best_match = None
        best_score = 0.0

        for obj_name, ref_features in self.reference_features.items():
            score = self.compare_features(query_features, ref_features)
            if score > best_score:
                best_score = score
                best_match = obj_name

        if best_score > threshold:
            return best_match, best_score
        else:
            return None, best_score

    def compare_features(self, features1, features2):
        """Compare two feature sets"""
        scores = []
        weights = {
            'linearity': 0.15,
            'planarity': 0.15,
            'scattering': 0.15,
            'volume': 0.1,
            'dimensions': 0.2,
            'compactness': 0.15,
            'anisotropy': 0.1
        }

        for feature_name, weight in weights.items():
            if feature_name in features1 and feature_name in features2:
                val1 = features1[feature_name]
                val2 = features2[feature_name]

                if isinstance(val1, (list, np.ndarray)):
                    # Handle array features like dimensions
                    diff = np.mean(np.abs(np.array(val1) - np.array(val2)))
                    max_val = max(np.max(np.abs(val1)), np.max(np.abs(val2)), 1e-6)
                    similarity = max(0, 1 - diff / max_val)
                else:
                    # Handle scalar features
                    diff = abs(val1 - val2)
                    max_val = max(abs(val1), abs(val2), 1e-6)
                    similarity = max(0, 1 - diff / max_val)

                scores.append(similarity * weight)

        return sum(scores) if scores else 0.0

    def extract_signature(self, points):
        """Extract a signature that's invariant to transformations"""
        # Create a shape signature that's invariant to rotation and translation
        # Using a simplified approach with radial distribution

        centroid = np.mean(points, axis=0)
        distances = np.linalg.norm(points - centroid, axis=1)

        # Create histogram of distances
        hist, _ = np.histogram(distances, bins=20, density=True)
        return hist

    def match_signatures(self, sig1, sig2):
        """Match two shape signatures"""
        # Use histogram intersection
        intersection = np.sum(np.minimum(sig1, sig2))
        return intersection
```

## Simultaneous Localization and Mapping (SLAM)

### Occupancy Grid Mapping

Creating 2D maps from sensor data:

```python
class OccupancyGridMapper:
    def __init__(self, width=20, height=20, resolution=0.1):
        self.width = width  # meters
        self.height = height  # meters
        self.resolution = resolution  # meters per cell
        self.grid_size = (int(width / resolution), int(height / resolution))

        # Initialize grid with unknown (0.5) occupancy
        self.occupancy_grid = np.full(self.grid_size, 0.5)

        # Log-odds representation for mathematical convenience
        self.log_odds = np.zeros(self.grid_size)

        # Robot position in grid coordinates
        self.robot_x = self.grid_size[0] // 2
        self.robot_y = self.grid_size[1] // 2

        # Sensor parameters
        self.max_range = 5.0  # meters
        self.angle_resolution = 0.01745  # radians (1 degree)

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x + self.width/2) / self.resolution)
        grid_y = int((y + self.height/2) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        x = grid_x * self.resolution - self.width/2
        y = grid_y * self.resolution - self.height/2
        return x, y

    def ray_trace(self, start_x, start_y, end_x, end_y):
        """Ray tracing to update free space along a beam"""
        # Bresenham's algorithm for line drawing
        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        x_step = 1 if end_x > start_x else -1
        y_step = 1 if end_y > start_y else -1

        error = dx - dy
        x, y = start_x, start_y

        points = []
        while True:
            points.append((x, y))
            if x == end_x and y == end_y:
                break

            error2 = 2 * error
            if error2 > -dy:
                error -= dy
                x += x_step
            if error2 < dx:
                error += dx
                y += y_step

        return points

    def update_with_laser_scan(self, robot_pose, ranges, angles):
        """Update occupancy grid with laser scan data"""
        robot_x, robot_y, robot_theta = robot_pose

        # Convert robot pose to grid coordinates
        robot_grid_x, robot_grid_y = self.world_to_grid(robot_x, robot_y)

        # Update grid for each laser beam
        for i, (range_reading, angle) in enumerate(zip(ranges, angles)):
            if range_reading < 0.1 or range_reading > self.max_range:
                continue  # Invalid reading

            # Calculate end point of beam in world coordinates
            beam_angle = robot_theta + angle
            end_x = robot_x + range_reading * np.cos(beam_angle)
            end_y = robot_y + range_reading * np.sin(beam_angle)

            # Convert to grid coordinates
            end_grid_x, end_grid_y = self.world_to_grid(end_x, end_y)

            # Ray trace to update free space
            free_cells = self.ray_trace(robot_grid_x, robot_grid_y,
                                      end_grid_x, end_grid_y)

            for x, y in free_cells[:-1]:  # Don't update the obstacle cell yet
                if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
                    # Update with free space (decrease occupancy)
                    self.log_odds[x, y] += self.log_probability_update(-0.1)  # Free space

            # Update the endpoint with obstacle information
            if 0 <= end_grid_x < self.grid_size[0] and 0 <= end_grid_y < self.grid_size[1]:
                self.log_odds[end_grid_x, end_grid_y] += self.log_probability_update(0.8)  # Obstacle

        # Convert log odds back to probability
        self.occupancy_grid = self.log_odds_to_probability(self.log_odds)

    def log_probability_update(self, prob):
        """Convert probability to log odds for updating"""
        prob = max(0.01, min(0.99, prob))  # Clamp to avoid log(0)
        return np.log(prob / (1 - prob))

    def log_odds_to_probability(self, log_odds):
        """Convert log odds back to probability"""
        odds = np.exp(log_odds)
        return odds / (1 + odds)

    def get_occupancy_probability(self, x, y):
        """Get occupancy probability at world coordinates"""
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.grid_size[0] and 0 <= grid_y < self.grid_size[1]:
            return self.occupancy_grid[grid_x, grid_y]
        return 0.5  # Unknown

    def find_free_space(self, robot_x, robot_y, radius=1.0):
        """Find nearby free space"""
        grid_x, grid_y = self.world_to_grid(robot_x, robot_y)
        radius_cells = int(radius / self.resolution)

        # Search in a square around the robot
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                x = grid_x + dx
                y = grid_y + dy

                if (0 <= x < self.grid_size[0] and
                    0 <= y < self.grid_size[1] and
                    self.occupancy_grid[x, y] < 0.3):  # Free space threshold
                    return self.grid_to_world(x, y)

        return robot_x, robot_y  # Return current position if no free space found

    def get_pathable_area(self, threshold=0.7):
        """Get coordinates of potentially pathable areas"""
        free_cells = np.where(self.occupancy_grid < threshold)
        free_coords = []

        for x, y in zip(free_cells[0], free_cells[1]):
            world_x, world_y = self.grid_to_world(x, y)
            free_coords.append((world_x, world_y))

        return free_coords

    def get_map_as_image(self):
        """Return occupancy grid as an image array"""
        # Normalize to 0-255 for image representation
        img = (self.occupancy_grid * 255).astype(np.uint8)
        return img
```

### Graph-Based SLAM

Advanced SLAM using pose graphs:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize

class PoseGraphSLAM:
    def __init__(self):
        self.poses = []  # List of [x, y, theta] poses
        self.constraints = []  # List of relative pose constraints
        self.optimization_result = None

    def add_pose(self, pose, is_fixed=False):
        """Add a new pose to the graph"""
        pose_id = len(self.poses)
        self.poses.append({
            'id': pose_id,
            'pose': np.array(pose),  # [x, y, theta]
            'is_fixed': is_fixed,
            'constraints': []
        })
        return pose_id

    def add_constraint(self, from_id, to_id, relative_pose, information_matrix=None):
        """Add a constraint between two poses"""
        if information_matrix is None:
            # Default information matrix (inverse covariance)
            information_matrix = np.eye(3) * 10  # High confidence

        constraint = {
            'from': from_id,
            'to': to_id,
            'relative_pose': np.array(relative_pose),  # [dx, dy, dtheta]
            'information': information_matrix
        }

        self.constraints.append(constraint)
        self.poses[from_id]['constraints'].append(len(self.constraints) - 1)

    def error_function(self, x):
        """Error function for pose graph optimization"""
        # Reshape x back to poses
        num_poses = len(self.poses)
        reshaped_poses = x.reshape(num_poses, 3)

        total_error = 0

        for constraint in self.constraints:
            from_id = constraint['from']
            to_id = constraint['to']

            pose_from = reshaped_poses[from_id]
            pose_to = reshaped_poses[to_id]
            relative_target = constraint['relative_pose']

            # Calculate relative pose from current estimates
            # Transform from pose_from to pose_to
            dx = pose_to[0] - pose_from[0]
            dy = pose_to[1] - pose_from[1]
            dtheta = pose_to[2] - pose_from[2]

            # Convert to the frame of pose_from
            dx_local = dx * np.cos(-pose_from[2]) - dy * np.sin(-pose_from[2])
            dy_local = dx * np.sin(-pose_from[2]) + dy * np.cos(-pose_from[2])
            dtheta_local = dtheta

            relative_current = np.array([dx_local, dy_local, dtheta_local])

            # Calculate error
            error = relative_current - relative_target

            # Apply information matrix (covariance weighting)
            weighted_error = error.T @ constraint['information'] @ error
            total_error += weighted_error

        return total_error

    def optimize_poses(self):
        """Optimize the pose graph"""
        if len(self.poses) < 2:
            return self.poses

        # Initialize optimization vector
        initial_poses = []
        free_pose_indices = []

        for i, pose_info in enumerate(self.poses):
            initial_poses.extend(pose_info['pose'])
            if not pose_info['is_fixed']:
                free_pose_indices.extend([i*3, i*3+1, i*3+2])

        if not free_pose_indices:
            return self.poses  # No free poses to optimize

        # Separate into fixed and variable poses
        all_indices = set(range(len(initial_poses)))
        fixed_indices = list(all_indices - set(free_pose_indices))

        def reduced_error_function(free_variables):
            """Error function with fixed variables substituted"""
            full_vector = np.array(initial_poses)
            full_vector[free_pose_indices] = free_variables

            return self.error_function(full_vector)

        # Optimize only the free variables
        result = minimize(
            reduced_error_function,
            x0=np.array(initial_poses)[free_pose_indices],
            method='BFGS'
        )

        # Update poses with optimization result
        optimized_vector = np.array(initial_poses)
        optimized_vector[free_pose_indices] = result.x

        for i in range(len(self.poses)):
            self.poses[i]['pose'] = optimized_vector[i*3:(i+1)*3]

        self.optimization_result = result
        return self.poses

    def get_trajectory(self):
        """Get the optimized trajectory"""
        trajectory = []
        for pose_info in self.poses:
            trajectory.append(pose_info['pose'])
        return np.array(trajectory)

    def add_odometry_constraint(self, from_id, to_id, odometry_measurement, covariance_scale=1.0):
        """Add an odometry-based constraint"""
        # Odometry typically has higher uncertainty
        information_matrix = np.diag([1.0, 1.0, 0.1]) / covariance_scale
        self.add_constraint(from_id, to_id, odometry_measurement, information_matrix)

    def add_loop_closure_constraint(self, from_id, to_id, measurement, covariance_scale=0.1):
        """Add a loop closure constraint (typically more accurate)"""
        # Loop closures are typically more accurate than odometry
        information_matrix = np.diag([10.0, 10.0, 1.0]) / covariance_scale
        self.add_constraint(from_id, to_id, measurement, information_matrix)
```

## Navigation and Path Planning

### Path Planning in Dynamic Environments

Planning paths that account for dynamic obstacles:

```python
import heapq

class DynamicPathPlanner:
    def __init__(self, occupancy_grid, resolution=0.1):
        self.occupancy_grid = occupancy_grid
        self.resolution = resolution
        self.grid_height, self.grid_width = occupancy_grid.shape
        self.object_predictor = ObjectMotionPredictor()

    def a_star(self, start, goal, dynamic_obstacles=None):
        """A* path planning with consideration for dynamic obstacles"""
        start_x, start_y = start
        goal_x, goal_y = goal

        # Convert to grid coordinates
        start_grid = (int(start_x / self.resolution), int(start_y / self.resolution))
        goal_grid = (int(goal_x / self.resolution), int(goal_y / self.resolution))

        # Check if start and goal are valid
        if not self.is_valid_cell(start_grid[0], start_grid[1]) or \
           not self.is_valid_cell(goal_grid[0], goal_grid[1]):
            return None

        # A* algorithm
        open_set = [(0, start_grid)]
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal_grid:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_grid)
                path.reverse()

                # Convert back to world coordinates
                world_path = [(x * self.resolution, y * self.resolution) for x, y in path]
                return world_path

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                # Check dynamic obstacle prediction
                if dynamic_obstacles:
                    neighbor_world = (neighbor[0] * self.resolution, neighbor[1] * self.resolution)
                    time_to_reach = tentative_g_score * 0.1  # Assume 10 m/s max speed
                    collision_risk = self.check_dynamic_collision(neighbor_world, time_to_reach, dynamic_obstacles)
                    if collision_risk > 0.7:  # High collision risk
                        continue

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

    def is_valid_cell(self, x, y):
        """Check if a grid cell is valid (not occupied)"""
        if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
            return self.occupancy_grid[y, x] < 0.7  # Occupancy threshold
        return False

    def get_neighbors(self, cell):
        """Get valid neighboring cells"""
        x, y = cell
        neighbors = []
        for dx, dy in [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]:
            nx, ny = x + dx, y + dy
            if self.is_valid_cell(nx, ny):
                neighbors.append((nx, ny))
        return neighbors

    def heuristic(self, a, b):
        """Heuristic function (Euclidean distance)"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def distance(self, a, b):
        """Distance between adjacent cells"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def check_dynamic_collision(self, position, time, dynamic_obstacles):
        """Check collision risk with predicted obstacle positions"""
        risk = 0.0
        for obstacle in dynamic_obstacles:
            predicted_pos = self.object_predictor.predict_position(obstacle, time)
            dist = np.linalg.norm(np.array(position) - np.array(predicted_pos))
            if dist < 0.5:  # 50cm safety margin
                # Higher risk as distance decreases
                risk = max(risk, 1.0 - dist/0.5)
        return risk

class ObjectMotionPredictor:
    def __init__(self):
        self.object_trajectories = {}  # Store past positions for prediction

    def update_object_position(self, object_id, position, timestamp):
        """Update object position and store for prediction"""
        if object_id not in self.object_trajectories:
            self.object_trajectories[object_id] = []

        self.object_trajectories[object_id].append((position, timestamp))

        # Keep only recent positions (last 5 seconds)
        current_time = timestamp
        self.object_trajectories[object_id] = [
            (pos, time) for pos, time in self.object_trajectories[object_id]
            if current_time - time < 5.0
        ]

    def predict_position(self, object_data, future_time):
        """Predict object position at future time"""
        object_id = object_data['id']

        if object_id not in self.object_trajectories or len(self.object_trajectories[object_id]) < 2:
            # No prediction possible, return current position
            return object_data['position']

        # Use the last two positions to estimate velocity
        positions = self.object_trajectories[object_id]
        pos1, time1 = positions[-2]
        pos2, time2 = positions[-1]

        if time2 == time1:
            return pos2

        velocity = (np.array(pos2) - np.array(pos1)) / (time2 - time1)
        predicted_position = np.array(pos2) + velocity * future_time

        return predicted_position.tolist()

    def predict_trajectory(self, object_id, time_horizon=2.0, dt=0.1):
        """Predict trajectory over time horizon"""
        trajectory = []
        for t in np.arange(0, time_horizon, dt):
            pos = self.predict_position({'id': object_id, 'position': [0, 0]}, t)
            trajectory.append((t, pos))
        return trajectory
```

### Human-Aware Navigation

Navigation that considers human presence and behavior:

```python
class HumanAwareNavigator:
    def __init__(self):
        self.social_force_model = SocialForceModel()
        self.anticipation_horizon = 3.0  # seconds to look ahead
        self.personal_space_radius = 0.8  # meters
        self.comfort_zone_radius = 1.2   # meters

    def plan_path_with_human_awareness(self, start, goal, humans, robot_speed=1.0):
        """Plan path considering human presence and expected movements"""
        # Use social force model to predict human movements
        predicted_human_trajectories = []
        for human in humans:
            trajectory = self.social_force_model.predict_trajectory(
                human['position'], human['velocity'], self.anticipation_horizon
            )
            predicted_human_trajectories.append({
                'id': human['id'],
                'trajectory': trajectory,
                'radius': self.personal_space_radius
            })

        # Plan path using a method that considers dynamic obstacles
        path_planner = DynamicPathPlanner(None)  # Would use actual occupancy grid
        path = path_planner.a_star(start, goal, predicted_human_trajectories)

        return path

    def calculate_social_comfort(self, robot_position, humans):
        """Calculate how comfortable the robot's position is for humans"""
        comfort_score = 1.0  # 1.0 is maximum comfort

        for human in humans:
            dist_to_human = np.linalg.norm(
                np.array(robot_position) - np.array(human['position'])
            )

            if dist_to_human < self.comfort_zone_radius:
                # Reduce comfort as we get closer to comfort zone
                penalty = (self.comfort_zone_radius - dist_to_human) / self.comfort_zone_radius
                comfort_score -= penalty * 0.5  # Weight the penalty

        return max(0, comfort_score)

    def adapt_behavior_for_humans(self, robot_state, humans):
        """Adapt robot behavior based on human presence"""
        closest_human_dist = float('inf')
        closest_human = None

        for human in humans:
            dist = np.linalg.norm(
                np.array(robot_state['position']) - np.array(human['position'])
            )
            if dist < closest_human_dist:
                closest_human_dist = dist
                closest_human = human

        if closest_human_dist < 2.0:  # Within 2m
            # Adjust behavior based on distance
            if closest_human_dist < 1.0:
                # Very close - slow down and give space
                speed_factor = 0.3
                prefer_side = self.get_preferred_passing_side(closest_human)
            elif closest_human_dist < 1.5:
                # Close - moderate caution
                speed_factor = 0.7
                prefer_side = self.get_preferred_passing_side(closest_human)
            else:
                # Far enough - normal speed
                speed_factor = 1.0
                prefer_side = None

            return {
                'speed_factor': speed_factor,
                'prefer_side': prefer_side,
                'is_alert': True
            }

        return {
            'speed_factor': 1.0,
            'prefer_side': None,
            'is_alert': False
        }

    def get_preferred_passing_side(self, human):
        """Determine preferred side to pass human"""
        # Simple heuristic: pass on the side where there's more free space
        # In practice, this would consider hallway geometry, walking direction, etc.
        return 'right'  # Convention in many countries

class SocialForceModel:
    def __init__(self):
        self.pedestrian_attraction = 1.0  # Tendency to follow others
        self.wall_repulsion = 5.0         # Repulsion from walls
        self.personal_space = 0.8         # Desired personal space (m)

    def predict_trajectory(self, position, velocity, time_horizon, dt=0.1):
        """Predict human trajectory using social forces"""
        trajectory = []
        current_pos = np.array(position)
        current_vel = np.array(velocity)

        for t in np.arange(0, time_horizon, dt):
            # Calculate social forces (simplified)
            acceleration = self.calculate_social_forces(current_pos, current_vel)

            # Integrate velocity and position
            current_vel += acceleration * dt
            current_pos += current_vel * dt

            trajectory.append(current_pos.copy())

        return trajectory

    def calculate_social_forces(self, position, velocity):
        """Calculate net force on pedestrian"""
        # This would include:
        # - Desired direction force
        # - Repulsive forces from other pedestrians and obstacles
        # - Attractive forces (e.g., following behavior)
        # For this example, return a simple force toward a destination

        # Example: move toward (5, 5) with some noise
        desired_pos = np.array([5.0, 5.0])
        desired_vel = (desired_pos - position) * 0.5  # 0.5 m/s desired speed

        # Calculate force to reach desired velocity
        max_speed = 1.3  # Max human walking speed
        current_speed = np.linalg.norm(velocity)
        if current_speed > max_speed:
            velocity = velocity * max_speed / current_speed

        force = (desired_vel - velocity) * 2.0  # Acceleration toward desired velocity

        return force
```

## Integration with ROS 2

### ROS 2 Environmental Perception Node

Integrating environmental perception with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

class EnvironmentalPerceptionNode(Node):
    def __init__(self):
        super().__init__('environmental_perception_node')

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception_markers', 10)

        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/points', self.pointcloud_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Initialize perception components
        self.point_processor = PointCloudProcessor()
        self.object_detector = Object3DDetector()
        self.mapper = OccupancyGridMapper()
        self.path_planner = DynamicPathPlanner(self.mapper.occupancy_grid)
        self.human_navigator = HumanAwareNavigator()

        # Previous detections for tracking
        self.previous_objects = []

        # Timer for periodic processing
        self.process_timer = self.create_timer(0.1, self.process_perception)

        self.get_logger().info('Environmental perception node initialized')

    def pointcloud_callback(self, msg):
        """Process incoming point cloud data"""
        try:
            # Convert ROS PointCloud2 to numpy array
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])

            if points:
                self.point_processor.load_point_cloud(points)
                self.point_processor.remove_outliers()

                # Process point cloud for objects
                objects = self.object_detector.detect_objects_voxel_grid(self.point_processor.points)

                # Update object tracking
                self.previous_objects = self.object_detector.track_objects(
                    objects, self.previous_objects
                )

                self.get_logger().info(f'Detected {len(objects)} objects')

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def scan_callback(self, msg):
        """Process laser scan data for mapping"""
        try:
            # Convert laser scan to ranges and angles
            ranges = np.array(msg.ranges)
            angles = np.array([msg.angle_min + i * msg.angle_increment
                              for i in range(len(ranges))])

            # Filter invalid ranges
            valid_indices = (ranges > msg.range_min) & (ranges < msg.range_max)
            valid_ranges = ranges[valid_indices]
            valid_angles = angles[valid_indices]

            # Update occupancy grid (assuming robot is at origin for this example)
            robot_pose = [0.0, 0.0, 0.0]  # [x, y, theta]
            self.mapper.update_with_laser_scan(robot_pose, valid_ranges, valid_angles)

            self.get_logger().info(f'Processed {len(valid_ranges)} laser points')

        except Exception as e:
            self.get_logger().error(f'Error processing laser scan: {e}')

    def process_perception(self):
        """Periodic processing of perception data"""
        try:
            # Publish current map
            self.publish_map()

            # Publish detected objects as markers
            self.publish_object_markers()

            # Example: plan a path (in a real system, goal would come from navigation)
            if len(self.previous_objects) > 0:
                start = [0.0, 0.0]
                goal = [2.0, 2.0]  # Example goal

                path = self.path_planner.a_star(start, goal)
                if path:
                    self.publish_path(path)

        except Exception as e:
            self.get_logger().error(f'Error in perception processing: {e}')

    def publish_map(self):
        """Publish occupancy grid map"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.mapper.resolution
        msg.info.width = self.mapper.grid_size[0]
        msg.info.height = self.mapper.grid_size[1]
        msg.info.origin.position.x = -self.mapper.width / 2
        msg.info.origin.position.y = -self.mapper.height / 2

        # Convert occupancy grid to the format needed by OccupancyGrid
        flat_grid = (self.mapper.occupancy_grid * 100).astype(np.int8).flatten()
        # Convert probabilities to ROS format (-1: unknown, 0: free, 100: occupied)
        flat_grid = np.where(flat_grid > 50, 100, np.where(flat_grid < 30, 0, -1))
        msg.data = flat_grid.tolist()

        self.map_pub.publish(msg)

    def publish_object_markers(self):
        """Publish detected objects as visualization markers"""
        marker_array = MarkerArray()

        for i, obj in enumerate(self.previous_objects):
            # Create marker for object position
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'objects'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(obj['center'][0])
            marker.pose.position.y = float(obj['center'][1])
            marker.pose.position.z = float(obj['center'][2])
            marker.pose.orientation.w = 1.0

            # Size based on object dimensions
            max_dim = max(obj['dimensions'])
            marker.scale.x = max_dim
            marker.scale.y = max_dim
            marker.scale.z = max_dim

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Semi-transparent

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def publish_path(self, path):
        """Publish planned path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for point in path:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    perception_node = EnvironmentalPerceptionNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

Environmental perception systems provide humanoid robots with the ability to understand and navigate their surroundings. From 3D point cloud processing to SLAM and path planning, these systems enable robots to operate safely and effectively in complex, dynamic environments. The integration of multiple sensor modalities through sensor fusion allows for robust and reliable environmental understanding, while real-time processing capabilities ensure that perception can keep pace with the demands of dynamic humanoid behavior.

The next section will provide practical examples and exercises that demonstrate the implementation of environmental perception techniques, allowing students to gain hands-on experience with these important concepts.