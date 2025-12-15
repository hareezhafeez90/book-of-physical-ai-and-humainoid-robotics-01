# Practical Examples and Exercises: Environmental Perception Implementation

## Introduction: From Theory to Practice

This section provides hands-on examples and exercises that demonstrate the implementation of environmental perception concepts using Python and ROS 2. Through these practical examples, students will gain experience implementing the theoretical concepts covered in previous sections, from basic point cloud processing to advanced SLAM and navigation systems.

## Exercise 1: Point Cloud Processing Pipeline

Let's start with a complete point cloud processing pipeline:

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

class PointCloudPipeline:
    def __init__(self):
        self.points = None
        self.colors = None
        self.processed_points = None
        self.processed_colors = None

    def generate_sample_point_cloud(self, num_points=1000):
        """Generate a sample point cloud with multiple objects"""
        np.random.seed(42)  # For reproducible results

        # Create a plane (floor)
        floor_points = []
        for _ in range(300):
            x = np.random.uniform(-2, 2)
            y = np.random.uniform(-2, 2)
            z = -1 + np.random.normal(0, 0.01)  # Floor at z=-1
            floor_points.append([x, y, z])

        # Create a box
        box_points = []
        for _ in range(200):
            x = np.random.uniform(0.5, 1.5)
            y = np.random.uniform(-0.5, 0.5)
            z = np.random.uniform(-0.5, 0.5)
            box_points.append([x, y, z])

        # Create a cylinder
        cylinder_points = []
        for _ in range(200):
            angle = np.random.uniform(0, 2*np.pi)
            radius = np.random.uniform(0, 0.3)
            x = -1 + radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = np.random.uniform(-0.5, 0.5)
            cylinder_points.append([x, y, z])

        # Add some noise
        noise_points = []
        for _ in range(100):
            x = np.random.uniform(-3, 3)
            y = np.random.uniform(-3, 3)
            z = np.random.uniform(-2, 2)
            noise_points.append([x, y, z])

        # Combine all points
        all_points = floor_points + box_points + cylinder_points + noise_points
        self.points = np.array(all_points)

        # Generate colors based on Z-height
        self.colors = plt.cm.viridis((self.points[:, 2] - np.min(self.points[:, 2])) /
                                   (np.max(self.points[:, 2]) - np.min(self.points[:, 2])))

        return self.points, self.colors

    def downsample(self, voxel_size=0.05):
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

        self.processed_points = np.array(downsampled_points)
        if self.colors is not None:
            self.processed_colors = np.array(downsampled_colors)

        return self.processed_points, self.processed_colors

    def remove_outliers(self, method='statistical', k=20, std_ratio=2.0):
        """Remove outlier points"""
        if self.processed_points is None:
            self.processed_points = self.points.copy()
            self.processed_colors = self.colors.copy() if self.colors is not None else None

        if method == 'statistical':
            distances = []
            for i, point in enumerate(self.processed_points):
                neighbor_distances = np.linalg.norm(self.processed_points - point, axis=1)
                k_nearest = np.partition(neighbor_distances, k+1)[1:k+1]
                avg_distance = np.mean(k_nearest)
                distances.append(avg_distance)

            distances = np.array(distances)
            mean_dist = np.mean(distances)
            std_dist = np.std(distances)

            valid_indices = distances < (mean_dist + std_ratio * std_dist)
            self.processed_points = self.processed_points[valid_indices]
            if self.processed_colors is not None:
                self.processed_colors = self.processed_colors[valid_indices]

        return self.processed_points, self.processed_colors

    def segment_planes(self, distance_threshold=0.01, min_points=100):
        """Segment planar surfaces using RANSAC"""
        if self.processed_points is None:
            return []

        remaining_points = self.processed_points.copy()
        planes = []

        while len(remaining_points) > min_points:
            best_model = None
            best_inliers = []
            best_score = 0

            for _ in range(100):  # RANSAC iterations
                sample_indices = np.random.choice(len(remaining_points), 3, replace=False)
                sample_points = remaining_points[sample_indices]

                v1 = sample_points[1] - sample_points[0]
                v2 = sample_points[2] - sample_points[0]
                normal = np.cross(v1, v2)
                if np.linalg.norm(normal) < 1e-6:
                    continue

                normal = normal / np.linalg.norm(normal)
                d = -np.dot(normal, sample_points[0])

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

                distances = np.abs(np.dot(remaining_points, best_model[0]) + best_model[1])
                remaining_points = remaining_points[distances >= distance_threshold]
            else:
                break

        return planes

    def segment_objects(self, eps=0.05, min_samples=20):
        """Segment objects using DBSCAN clustering"""
        if self.processed_points is None:
            return []

        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(self.processed_points)
        labels = clustering.labels_

        objects = []
        for label in set(labels):
            if label == -1:  # Noise points
                continue

            object_points = self.processed_points[labels == label]
            object_colors = self.processed_colors[labels == label] if self.processed_colors is not None else None

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

    def visualize(self, title="Point Cloud"):
        """Visualize the point cloud"""
        fig = plt.figure(figsize=(12, 5))

        # Original point cloud
        ax1 = fig.add_subplot(121, projection='3d')
        if self.points is not None:
            ax1.scatter(self.points[:, 0], self.points[:, 1], self.points[:, 2],
                      c=self.colors, s=1, alpha=0.6)
        ax1.set_title('Original Point Cloud')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')

        # Processed point cloud
        ax2 = fig.add_subplot(122, projection='3d')
        if self.processed_points is not None:
            ax2.scatter(self.processed_points[:, 0], self.processed_points[:, 1], self.processed_points[:, 2],
                      c=self.processed_colors, s=1, alpha=0.6)
        ax2.set_title('Processed Point Cloud')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')

        plt.tight_layout()
        plt.show()

# Demonstrate the pipeline
pipeline = PointCloudPipeline()

# Generate sample data
points, colors = pipeline.generate_sample_point_cloud()
print(f"Generated {len(points)} points")

# Process the point cloud
downsampled_points, downsampled_colors = pipeline.downsample(voxel_size=0.05)
print(f"After downsampling: {len(downsampled_points)} points")

filtered_points, filtered_colors = pipeline.remove_outliers()
print(f"After outlier removal: {len(filtered_points)} points")

# Segment planes
planes = pipeline.segment_planes()
print(f"Detected {len(planes)} planes")

# Segment objects
objects = pipeline.segment_objects()
print(f"Detected {len(objects)} objects")

# Visualize results
pipeline.visualize()

# Print object information
for i, obj in enumerate(objects):
    print(f"Object {i}: Center={obj['center']}, Dimensions={obj['dimensions']}")
```

## Exercise 2: 3D Object Recognition System

Now let's implement a complete 3D object recognition system:

```python
class ObjectRecognitionSystem:
    def __init__(self):
        self.known_objects = {}
        self.feature_extractor = PointCloudPipeline()  # Reusing for simplicity

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

        # Size features
        bbox_min = np.min(points, axis=0)
        bbox_max = np.max(points, axis=0)
        dimensions = bbox_max - bbox_min
        features['volume'] = np.prod(dimensions)
        features['dimensions'] = dimensions

        # Density features
        try:
            from scipy.spatial import ConvexHull
            hull = ConvexHull(points)
            convex_volume = hull.volume
            features['compactness'] = features['volume'] / (convex_volume + 1e-6)
        except:
            # Fallback for collinear points
            features['compactness'] = 0.0

        return features

    def register_object(self, object_name, point_cloud):
        """Register a known object with its features"""
        features = self.extract_geometric_features(point_cloud)
        self.known_objects[object_name] = features
        print(f"Registered object: {object_name}")

    def recognize_object(self, point_cloud, threshold=0.3):
        """Recognize object by comparing features with registered objects"""
        if not self.known_objects:
            return None, 0.0

        query_features = self.extract_geometric_features(point_cloud)

        best_match = None
        best_score = 0.0

        for obj_name, ref_features in self.known_objects.items():
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

    def generate_test_objects(self):
        """Generate test objects for recognition"""
        # Create a cube
        cube_points = []
        for _ in range(500):
            x = np.random.uniform(-0.5, 0.5)
            y = np.random.uniform(-0.5, 0.5)
            z = np.random.uniform(-0.5, 0.5)
            cube_points.append([x, y, z])
        cube_points = np.array(cube_points)

        # Create a cylinder
        cylinder_points = []
        for _ in range(500):
            angle = np.random.uniform(0, 2*np.pi)
            radius = np.random.uniform(0, 0.5)
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = np.random.uniform(-0.5, 0.5)
            cylinder_points.append([x, y, z])
        cylinder_points = np.array(cylinder_points)

        # Create a sphere
        sphere_points = []
        for _ in range(500):
            vec = np.random.normal(0, 1, 3)
            vec = vec / np.linalg.norm(vec) * np.random.uniform(0, 0.5)
            sphere_points.append(vec)
        sphere_points = np.array(sphere_points)

        return {
            'cube': cube_points,
            'cylinder': cylinder_points,
            'sphere': sphere_points
        }

# Demonstrate object recognition
recognition_system = ObjectRecognitionSystem()

# Generate test objects
test_objects = recognition_system.generate_test_objects()

# Register known objects
for obj_name, points in test_objects.items():
    recognition_system.register_object(obj_name, points)

# Test recognition
print("\nTesting object recognition:")
for obj_name, points in test_objects.items():
    # Add some noise to test object
    noisy_points = points + np.random.normal(0, 0.05, points.shape)

    recognized_obj, confidence = recognition_system.recognize_object(noisy_points)
    print(f"Test {obj_name}: Recognized as '{recognized_obj}' with confidence {confidence:.3f}")
```

## Exercise 3: Occupancy Grid Mapping

Implement a complete occupancy grid mapping system:

```python
class OccupancyGridMapper:
    def __init__(self, width=10, height=10, resolution=0.1):
        self.width = width  # meters
        self.height = height  # meters
        self.resolution = resolution  # meters per cell
        self.grid_size = (int(width / resolution), int(height / resolution))

        # Initialize grid with unknown (0.5) occupancy
        self.occupancy_grid = np.full(self.grid_size, 0.5)

        # Log-odds representation for mathematical convenience
        self.log_odds = np.zeros(self.grid_size)

        # Robot position in grid coordinates (center of map)
        self.robot_x = self.grid_size[0] // 2
        self.robot_y = self.grid_size[1] // 2

        # Sensor parameters
        self.max_range = 5.0  # meters
        self.prob_hit = 0.7   # Probability of hit
        self.prob_miss = 0.4  # Probability of miss

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
        """Ray tracing using Bresenham's algorithm"""
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
        for range_reading, angle in zip(ranges, angles):
            if range_reading < 0.1 or range_reading > self.max_range:
                continue  # Invalid reading

            # Calculate end point of beam in world coordinates
            beam_angle = robot_theta + angle
            end_x = robot_x + range_reading * np.cos(beam_angle)
            end_y = robot_y + range_reading * np.sin(beam_angle)

            # Convert to grid coordinates
            end_grid_x, end_grid_y = self.world_to_grid(end_x, end_y)

            # Ensure endpoints are within grid bounds
            end_grid_x = np.clip(end_grid_x, 0, self.grid_size[0] - 1)
            end_grid_y = np.clip(end_grid_y, 0, self.grid_size[1] - 1)

            # Ray trace to update free space
            free_cells = self.ray_trace(robot_grid_x, robot_grid_y,
                                      end_grid_x, end_grid_y)

            # Update free space (all cells except the last one)
            for x, y in free_cells[:-1]:
                if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
                    # Convert probability to log odds and update
                    old_log_odds = self.log_odds[x, y]
                    new_log_odds = old_log_odds + self.log_probability_update(self.prob_miss)
                    self.log_odds[x, y] = np.clip(new_log_odds, -10, 10)  # Limit range

            # Update the endpoint with obstacle information (if not max range)
            if range_reading < self.max_range:
                if 0 <= end_grid_x < self.grid_size[0] and 0 <= end_grid_y < self.grid_size[1]:
                    old_log_odds = self.log_odds[end_grid_x, end_grid_y]
                    new_log_odds = old_log_odds + self.log_probability_update(self.prob_hit)
                    self.log_odds[end_grid_x, end_grid_y] = np.clip(new_log_odds, -10, 10)

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

    def get_map_as_image(self):
        """Return occupancy grid as an image array"""
        # Normalize to 0-255 for image representation
        img = (self.occupancy_grid * 255).astype(np.uint8)
        return img

    def visualize_map(self):
        """Visualize the occupancy grid"""
        plt.figure(figsize=(10, 10))
        plt.imshow(self.occupancy_grid, cmap='gray', origin='lower',
                  extent=[-self.width/2, self.width/2, -self.height/2, self.height/2])
        plt.colorbar(label='Occupancy Probability')
        plt.title('Occupancy Grid Map')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.grid(True, alpha=0.3)
        plt.show()

# Simulate robot movement and mapping
mapper = OccupancyGridMapper(width=10, height=10, resolution=0.1)

# Create a simple environment with obstacles
def simulate_laser_scan(robot_x, robot_y, robot_theta, map_func):
    """Simulate laser scan in a simple environment"""
    num_beams = 360
    angles = np.linspace(0, 2*np.pi, num_beams, endpoint=False)
    ranges = []

    for angle in angles:
        world_angle = robot_theta + angle
        max_range = 5.0  # Max sensor range

        # Ray march to find obstacle
        step_size = 0.1
        current_range = 0
        hit_obstacle = False

        while current_range < max_range:
            test_x = robot_x + current_range * np.cos(world_angle)
            test_y = robot_y + current_range * np.sin(world_angle)

            # Check if we hit an obstacle in our simulated environment
            # Simple environment: walls at boundaries and a few obstacles
            if (abs(test_x) > 4.5 or abs(test_y) > 4.5 or  # Walls
                (1 < test_x < 2 and -1 < test_y < 1) or   # Obstacle 1
                (-2 < test_x < -1 and 1 < test_y < 2) or  # Obstacle 2
                (0 < test_x < 1 and 2 < test_y < 3)):     # Obstacle 3
                ranges.append(current_range)
                hit_obstacle = True
                break

            current_range += step_size

        if not hit_obstacle:
            ranges.append(max_range)

    return ranges, angles

# Simulate robot exploring the environment
robot_path = []
for step in range(50):
    # Robot moves in a spiral pattern
    t = step * 0.5
    robot_x = 2 * np.cos(t * 0.3) * (1 - t * 0.02)
    robot_y = 2 * np.sin(t * 0.3) * (1 - t * 0.02)
    robot_theta = t * 0.3  # Robot orientation

    robot_path.append((robot_x, robot_y))

    # Get simulated laser scan
    ranges, angles = simulate_laser_scan(robot_x, robot_y, robot_theta, None)

    # Update map with laser scan
    mapper.update_with_laser_scan([robot_x, robot_y, robot_theta], ranges, angles)

    if step % 10 == 0:  # Visualize every 10 steps
        print(f"Step {step}: Updated map")

print("Mapping completed!")
mapper.visualize_map()

# Print statistics
occupied_cells = np.sum(mapper.occupancy_grid > 0.7)
free_cells = np.sum(mapper.occupancy_grid < 0.3)
unknown_cells = np.sum((mapper.occupancy_grid >= 0.3) & (mapper.occupancy_grid <= 0.7))

print(f"Occupied cells: {occupied_cells}")
print(f"Free cells: {free_cells}")
print(f"Unknown cells: {unknown_cells}")
```

## Exercise 4: Path Planning with A* Algorithm

Implement a complete path planning system:

```python
import heapq

class PathPlanner:
    def __init__(self, occupancy_grid, resolution=0.1):
        self.occupancy_grid = occupancy_grid
        self.resolution = resolution
        self.grid_height, self.grid_width = occupancy_grid.shape

    def a_star(self, start, goal, occupancy_threshold=0.7):
        """A* path planning algorithm"""
        start_x, start_y = start
        goal_x, goal_y = goal

        # Convert to grid coordinates
        start_grid = (int(start_x / self.resolution), int(start_y / self.resolution))
        goal_grid = (int(goal_x / self.resolution), int(goal_y / self.resolution))

        # Check if start and goal are valid
        if not self.is_valid_cell(start_grid[0], start_grid[1], occupancy_threshold) or \
           not self.is_valid_cell(goal_grid[0], goal_grid[1], occupancy_threshold):
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

            for neighbor in self.get_neighbors(current, occupancy_threshold):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

    def is_valid_cell(self, x, y, threshold=0.7):
        """Check if a grid cell is valid (not occupied)"""
        if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
            return self.occupancy_grid[y, x] < threshold
        return False

    def get_neighbors(self, cell, threshold=0.7):
        """Get valid neighboring cells"""
        x, y = cell
        neighbors = []
        # 8-connected neighborhood
        for dx, dy in [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]:
            nx, ny = x + dx, y + dy
            if self.is_valid_cell(nx, ny, threshold):
                neighbors.append((nx, ny))
        return neighbors

    def heuristic(self, a, b):
        """Heuristic function (Euclidean distance)"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def distance(self, a, b):
        """Distance between adjacent cells"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def visualize_path(self, start, goal, path):
        """Visualize the planned path"""
        plt.figure(figsize=(10, 10))
        plt.imshow(self.occupancy_grid, cmap='gray', origin='lower',
                  extent=[0, self.grid_width * self.resolution, 0, self.grid_height * self.resolution])

        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            plt.plot(path_x, path_y, 'r-', linewidth=2, label='Planned Path')

        plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
        plt.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

        plt.colorbar(label='Occupancy Probability')
        plt.title('A* Path Planning')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()

# Use the mapper from previous exercise
planner = PathPlanner(mapper.occupancy_grid, resolution=0.1)

# Plan a path
start = (0.0, 0.0)  # Robot start position
goal = (3.0, 3.0)   # Goal position

path = planner.a_star(start, goal)

if path:
    print(f"Path found with {len(path)} waypoints")
    planner.visualize_path(start, goal, path)

    # Print path details
    total_distance = 0
    for i in range(1, len(path)):
        total_distance += np.sqrt((path[i][0] - path[i-1][0])**2 +
                                 (path[i][1] - path[i-1][1])**2)
    print(f"Total path distance: {total_distance:.2f} m")
else:
    print("No path found!")

# Test with different start/goal positions
start2 = (-3.0, -3.0)
goal2 = (4.0, 4.0)
path2 = planner.a_star(start2, goal2)

if path2:
    print(f"Alternative path found with {len(path2)} waypoints")
    planner.visualize_path(start2, goal2, path2)
else:
    print("No path found for alternative route!")
```

## Exercise 5: Sensor Fusion for State Estimation

Implement a complete sensor fusion system:

```python
class SensorFusionSystem:
    def __init__(self):
        # State: [x, y, theta, vx, vy, omega] (position, orientation, velocities)
        self.state = np.array([0, 0, 0, 0, 0, 0], dtype=float)
        self.covariance = np.eye(6) * 0.1

        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.01, 0.5, 0.5, 0.1])

        # Measurement noise
        self.R_position = np.diag([0.05, 0.05])  # GPS/visual position noise
        self.R_orientation = np.array([[0.01]])  # IMU orientation noise
        self.R_velocity = np.diag([0.1, 0.1, 0.05])  # IMU velocity noise

    def predict(self, dt, control_input=None):
        """Prediction step using motion model"""
        x, y, theta, vx, vy, omega = self.state

        # Motion model: constant velocity with rotation
        # For simplicity, assume control input affects acceleration
        if control_input is not None:
            ax, ay = control_input
        else:
            ax, ay = 0, 0

        # Update state
        new_state = self.state.copy()
        new_state[0] += (vx * np.cos(theta) - vy * np.sin(theta)) * dt  # x
        new_state[1] += (vx * np.sin(theta) + vy * np.cos(theta)) * dt  # y
        new_state[2] += omega * dt  # theta
        new_state[3] += ax * dt  # vx
        new_state[4] += ay * dt  # vy
        # omega remains the same (constant angular velocity assumption)

        self.state = new_state

        # Jacobian of motion model
        F = np.eye(6)
        F[0, 2] = (-vx * np.sin(theta) - vy * np.cos(theta)) * dt  # dx/dtheta
        F[0, 3] = np.cos(theta) * dt  # dx/dvx
        F[0, 4] = -np.sin(theta) * dt  # dx/dvy
        F[1, 2] = (vx * np.cos(theta) - vy * np.sin(theta)) * dt  # dy/dtheta
        F[1, 3] = np.sin(theta) * dt  # dy/dvx
        F[1, 4] = np.cos(theta) * dt  # dy/dvy
        F[3, 5] = 0  # No direct coupling in this simple model
        F[4, 5] = 0

        # Update covariance
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update_position(self, measurement):
        """Update with position measurement (e.g., from visual odometry or GPS)"""
        # Measurement model: H maps state to measurement
        H = np.array([
            [1, 0, 0, 0, 0, 0],  # measure x
            [0, 1, 0, 0, 0, 0]   # measure y
        ])

        # Innovation
        innovation = measurement - H @ self.state

        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_position

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.state = self.state + K @ innovation
        self.covariance = (np.eye(6) - K @ H) @ self.covariance

    def update_orientation(self, measurement):
        """Update with orientation measurement (e.g., from IMU)"""
        # Measurement model for orientation
        H = np.array([[0, 0, 1, 0, 0, 0]])  # measure theta

        # Innovation (handle angle wrapping)
        innovation = measurement - self.state[2]
        # Normalize angle to [-pi, pi]
        innovation = np.arctan2(np.sin(innovation), np.cos(innovation))

        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_orientation

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.state[2] += K[2, 0] * innovation
        # Normalize orientation angle
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        self.covariance = (np.eye(6) - K @ H) @ self.covariance

    def update_velocity(self, measurement):
        """Update with velocity measurement (e.g., from wheel encoders or IMU)"""
        # Measurement model for velocity in robot frame
        theta = self.state[2]
        H = np.array([
            [0, 0, 0, np.cos(theta), -np.sin(theta), 0],  # v_x in world frame
            [0, 0, 0, np.sin(theta), np.cos(theta), 0],   # v_y in world frame
            [0, 0, 0, 0, 0, 1]                            # omega (angular velocity)
        ])

        # Innovation
        predicted_velocity = H @ self.state
        innovation = measurement - predicted_velocity

        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_velocity

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.state = self.state + K @ innovation
        self.covariance = (np.eye(6) - K @ H) @ self.covariance

    def get_state(self):
        """Get current state estimate"""
        return self.state.copy()

    def get_covariance(self):
        """Get current covariance estimate"""
        return self.covariance.copy()

# Demonstrate sensor fusion
fusion_system = SensorFusionSystem()

# Simulate robot movement with sensor measurements
true_positions = []
estimated_positions = []
measurements = []

for step in range(100):
    dt = 0.1  # 10Hz

    # True motion (simulated)
    true_control = [0.1 * np.sin(step * 0.1), 0.05 * np.cos(step * 0.1)]  # Oscillating motion

    # Predict step
    fusion_system.predict(dt, true_control)

    # Simulate measurements with noise
    true_pos = fusion_system.state[0:2] + np.random.normal(0, 0.1, 2)  # Position with noise
    true_orientation = fusion_system.state[2] + np.random.normal(0, 0.01)  # Orientation with noise
    true_velocity = np.array([1.0, 0.1, 0.05]) + np.random.normal(0, 0.05, 3)  # Velocity with noise

    # Update with different sensor measurements
    fusion_system.update_position(true_pos)
    fusion_system.update_orientation(true_orientation)
    fusion_system.update_velocity(true_velocity)

    # Store for visualization
    true_positions.append([step * dt * 0.5, step * dt * 0.2])  # Simulated true trajectory
    estimated_positions.append(fusion_system.get_state()[0:2].copy())
    measurements.append(true_pos.copy())

true_positions = np.array(true_positions)
estimated_positions = np.array(estimated_positions)
measurements = np.array(measurements)

# Visualize results
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(true_positions[:, 0], true_positions[:, 1], 'g-', label='True Trajectory', linewidth=2)
plt.plot(estimated_positions[:, 0], estimated_positions[:, 1], 'b-', label='Estimated Trajectory', linewidth=2)
plt.scatter(measurements[:, 0], measurements[:, 1], c='r', s=10, alpha=0.5, label='Measurements')
plt.title('Sensor Fusion: Trajectory Estimation')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.grid(True, alpha=0.3)

plt.subplot(1, 2, 2)
position_error = np.linalg.norm(estimated_positions - true_positions, axis=1)
plt.plot(position_error, 'r-', linewidth=2)
plt.title('Position Estimation Error')
plt.xlabel('Time Step')
plt.ylabel('Error (m)')
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Print final statistics
final_error = np.linalg.norm(estimated_positions[-1] - true_positions[-1])
print(f"Final position error: {final_error:.3f} m")
print(f"Average position error: {np.mean(position_error):.3f} m")
print(f"RMS position error: {np.sqrt(np.mean(position_error**2)):.3f} m")
```

## Exercise 6: Complete Environmental Perception Pipeline

Let's create a complete environmental perception system that integrates all components:

```python
class CompletePerceptionSystem:
    def __init__(self):
        # Initialize all components
        self.point_cloud_processor = PointCloudPipeline()
        self.object_recognizer = ObjectRecognitionSystem()
        self.occupancy_mapper = OccupancyGridMapper(width=20, height=20, resolution=0.1)
        self.path_planner = PathPlanner(self.occupancy_mapper.occupancy_grid)
        self.sensor_fusion = SensorFusionSystem()

        # System state
        self.robot_pose = np.array([0, 0, 0])  # x, y, theta
        self.detected_objects = []
        self.map_updated = False

    def process_point_cloud(self, points, colors=None):
        """Process point cloud data"""
        self.point_cloud_processor.points = points
        self.point_cloud_processor.colors = colors

        # Apply processing pipeline
        self.point_cloud_processor.downsample(voxel_size=0.05)
        self.point_cloud_processor.remove_outliers()

        # Segment objects
        objects = self.point_cloud_processor.segment_objects(eps=0.1, min_samples=30)
        self.detected_objects = objects

        return objects

    def update_map_with_scan(self, robot_pose, ranges, angles):
        """Update occupancy grid with laser scan"""
        self.occupancy_mapper.update_with_laser_scan(robot_pose, ranges, angles)
        self.map_updated = True

        # Update path planner with new map
        self.path_planner = PathPlanner(self.occupancy_mapper.occupancy_grid,
                                      resolution=self.occupancy_mapper.resolution)

    def plan_path(self, start, goal):
        """Plan path using current map"""
        if not self.map_updated:
            print("Warning: Map not updated, using empty map for planning")

        path = self.path_planner.a_star(start, goal)
        return path

    def get_environment_summary(self):
        """Get summary of current environment understanding"""
        summary = {
            'robot_pose': self.robot_pose,
            'num_detected_objects': len(self.detected_objects),
            'object_types': [],
            'map_coverage': np.sum(self.occupancy_mapper.occupancy_grid != 0.5) / self.occupancy_mapper.occupancy_grid.size,
            'free_space_ratio': np.sum(self.occupancy_mapper.occupancy_grid < 0.3) / self.occupancy_mapper.occupancy_grid.size
        }

        for obj in self.detected_objects:
            # Estimate object type based on dimensions
            dims = obj['dimensions']
            max_dim = np.max(dims)
            min_dim = np.min(dims)

            if max_dim / min_dim > 5:  # Very elongated
                obj_type = 'rod'
            elif 2 < max_dim / min_dim < 5:  # Moderately elongated
                obj_type = 'cylinder'
            else:  # More cube-like
                obj_type = 'box'

            summary['object_types'].append(obj_type)

        return summary

    def visualize_environment(self):
        """Visualize the complete environment understanding"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))

        # Plot 1: Occupancy grid
        im1 = axes[0, 0].imshow(self.occupancy_mapper.occupancy_grid, cmap='gray', origin='lower')
        axes[0, 0].set_title('Occupancy Grid Map')
        axes[0, 0].set_xlabel('X (m)')
        axes[0, 0].set_ylabel('Y (m)')
        plt.colorbar(im1, ax=axes[0, 0])

        # Plot 2: Detected objects
        axes[0, 1].imshow(self.occupancy_mapper.occupancy_grid, cmap='gray', origin='lower',
                         extent=[-self.occupancy_mapper.width/2, self.occupancy_mapper.width/2,
                                -self.occupancy_mapper.height/2, self.occupancy_mapper.height/2])

        for obj in self.detected_objects:
            axes[0, 1].scatter(obj['center'][0], obj['center'][1], c='red', s=100, marker='s', label='Objects' if obj==self.detected_objects[0] else "")
            # Draw bounding box
            bbox_min, bbox_max = obj['bbox_min'], obj['bbox_max']
            rect = plt.Rectangle((bbox_min[0], bbox_min[1]),
                               bbox_max[0]-bbox_min[0], bbox_max[1]-bbox_min[1],
                               linewidth=1, edgecolor='red', facecolor='none')
            axes[0, 1].add_patch(rect)

        axes[0, 1].set_title('Detected Objects')
        axes[0, 1].set_xlabel('X (m)')
        axes[0, 1].set_ylabel('Y (m)')
        axes[0, 1].legend()

        # Plot 3: Robot trajectory (if available)
        # For this example, we'll simulate a trajectory
        t = np.linspace(0, 10, 100)
        robot_traj_x = 2 * np.sin(0.3 * t)
        robot_traj_y = 2 * np.cos(0.3 * t)
        axes[1, 0].plot(robot_traj_x, robot_traj_y, 'b-', linewidth=2, label='Robot Trajectory')
        axes[1, 0].scatter([robot_traj_x[0]], [robot_traj_y[0]], c='green', s=100, label='Start', zorder=5)
        axes[1, 0].scatter([robot_traj_x[-1]], [robot_traj_y[-1]], c='red', s=100, label='End', zorder=5)
        axes[1, 0].set_title('Robot Trajectory')
        axes[1, 0].set_xlabel('X (m)')
        axes[1, 0].set_ylabel('Y (m)')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        # Plot 4: Environment summary
        summary = self.get_environment_summary()
        axes[1, 1].axis('off')  # Turn off axis for text display

        summary_text = f"""Environment Summary:

Robot Position: ({summary['robot_pose'][0]:.2f}, {summary['robot_pose'][1]:.2f})
Number of Objects: {summary['num_detected_objects']}
Object Types: {', '.join(summary['object_types'][:5])}{'...' if len(summary['object_types']) > 5 else ''}
Map Coverage: {summary['map_coverage']:.1%}
Free Space: {summary['free_space_ratio']:.1%}

Processing Status:
- Point Cloud Processing: Complete
- Object Detection: {len(self.detected_objects)} objects
- Map Update: {'Complete' if self.map_updated else 'Pending'}
- Path Planning: Ready"""

        axes[1, 1].text(0.1, 0.5, summary_text, fontsize=12, verticalalignment='center',
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))

        plt.tight_layout()
        plt.show()

# Demonstrate the complete system
perception_system = CompletePerceptionSystem()

# Generate sample point cloud data
points, colors = perception_system.point_cloud_processor.generate_sample_point_cloud(2000)

# Process the point cloud
objects = perception_system.process_point_cloud(points, colors)
print(f"Detected {len(objects)} objects")

# Simulate laser scan and update map
robot_pose = [0, 0, 0]
ranges = [4.5] * 360  # Simulated ranges (mostly free space)
angles = np.linspace(0, 2*np.pi, 360, endpoint=False)
perception_system.update_map_with_scan(robot_pose, ranges, angles)

# Plan a path
path = perception_system.plan_path((0, 0), (5, 5))
if path:
    print(f"Planned path with {len(path)} waypoints")

# Get environment summary
summary = perception_system.get_environment_summary()
print("\nEnvironment Summary:")
for key, value in summary.items():
    print(f"{key}: {value}")

# Visualize the complete system
perception_system.visualize_environment()
```

## Exercise 7: Real-time Perception Simulation

Create a simulation that demonstrates real-time perception processing:

```python
import time
from collections import deque

class RealTimePerceptionSimulator:
    def __init__(self):
        self.perception_system = CompletePerceptionSystem()
        self.simulation_time = 0
        self.time_step = 0.1  # 10 Hz
        self.robot_trajectory = []
        self.processing_times = deque(maxlen=50)  # Keep last 50 processing times

    def simulate_robot_movement(self):
        """Simulate robot moving in environment"""
        # Robot follows a circular path
        x = 3 * np.cos(self.simulation_time * 0.2)
        y = 3 * np.sin(self.simulation_time * 0.2)
        theta = self.simulation_time * 0.2  # Robot orientation
        return np.array([x, y, theta])

    def simulate_sensor_data(self, robot_pose):
        """Simulate sensor data for current robot pose"""
        # Simulate point cloud around robot
        num_points = 500
        points = []

        # Add some objects in the environment
        for _ in range(num_points):
            # Sensor range: 5m
            range_val = np.random.uniform(0.5, 5.0)
            angle = np.random.uniform(0, 2*np.pi)

            # Convert to robot coordinates
            local_x = range_val * np.cos(angle)
            local_y = range_val * np.sin(angle)

            # Transform to world coordinates
            cos_th = np.cos(robot_pose[2])
            sin_th = np.sin(robot_pose[2])

            world_x = robot_pose[0] + local_x * cos_th - local_y * sin_th
            world_y = robot_pose[1] + local_x * sin_th + local_y * cos_th
            world_z = np.random.uniform(-0.5, 0.5)  # Ground level with variation

            points.append([world_x, world_y, world_z])

        return np.array(points)

    def simulate_laser_scan(self, robot_pose):
        """Simulate laser scan data"""
        num_beams = 360
        ranges = []
        angles = np.linspace(0, 2*np.pi, num_beams, endpoint=False)

        # Simulate scan with some obstacles
        for angle in angles:
            world_angle = robot_pose[2] + angle
            # Simulate obstacles at specific locations
            obstacle_distances = [
                np.linalg.norm([robot_pose[0] + 2 - (robot_pose[0] + 5*np.cos(world_angle)),
                              robot_pose[1] + 1 - (robot_pose[1] + 5*np.sin(world_angle))]),
                np.linalg.norm([robot_pose[0] - 1 - (robot_pose[0] + 5*np.cos(world_angle)),
                              robot_pose[1] + 2 - (robot_pose[1] + 5*np.sin(world_angle))])
            ]

            min_dist = min(obstacle_distances + [5.0])  # Min of obstacles or max range
            ranges.append(min(min_dist, 5.0))

        return np.array(ranges), angles

    def run_simulation_step(self):
        """Run one step of the simulation"""
        start_time = time.time()

        # Update simulation time
        self.simulation_time += self.time_step

        # Get robot pose
        robot_pose = self.simulate_robot_movement()
        self.robot_trajectory.append(robot_pose.copy())

        # Simulate sensor data
        point_cloud = self.simulate_sensor_data(robot_pose)
        laser_ranges, laser_angles = self.simulate_laser_scan(robot_pose)

        # Process perception
        self.perception_system.process_point_cloud(point_cloud)
        self.perception_system.update_map_with_scan(robot_pose, laser_ranges, laser_angles)

        # Record processing time
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)

        return processing_time

    def run_simulation(self, duration=10.0):
        """Run the complete simulation"""
        print("Starting real-time perception simulation...")
        print(f"Simulation duration: {duration}s at {1/self.time_step}Hz")

        num_steps = int(duration / self.time_step)

        for step in range(num_steps):
            processing_time = self.run_simulation_step()

            if step % 20 == 0:  # Print status every 20 steps
                avg_processing_time = np.mean(self.processing_times) if self.processing_times else 0
                print(f"Step {step}/{num_steps}, "
                      f"Processing time: {processing_time*1000:.1f}ms, "
                      f"Average: {avg_processing_time*1000:.1f}ms, "
                      f"Objects detected: {len(self.perception_system.detected_objects)}")

        print("Simulation completed!")

        # Print final statistics
        if self.processing_times:
            avg_time = np.mean(self.processing_times) * 1000  # Convert to ms
            max_time = np.max(self.processing_times) * 1000
            min_time = np.min(self.processing_times) * 1000

            print(f"\nPerformance Statistics:")
            print(f"Average processing time: {avg_time:.2f} ms")
            print(f"Min processing time: {min_time:.2f} ms")
            print(f"Max processing time: {max_time:.2f} ms")
            print(f"Required frequency: {1/self.time_step:.1f} Hz")
            print(f"Required processing time: {self.time_step*1000:.1f} ms")
            print(f"Real-time capable: {'Yes' if avg_time < self.time_step*1000 else 'No'}")

    def visualize_simulation_results(self):
        """Visualize simulation results"""
        if not self.robot_trajectory:
            print("No trajectory data to visualize")
            return

        traj = np.array(self.robot_trajectory)

        fig, axes = plt.subplots(2, 2, figsize=(15, 12))

        # Robot trajectory
        axes[0, 0].plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2, label='Trajectory')
        axes[0, 0].scatter(traj[0, 0], traj[0, 1], c='green', s=100, label='Start', zorder=5)
        axes[0, 0].scatter(traj[-1, 0], traj[-1, 1], c='red', s=100, label='End', zorder=5)
        axes[0, 0].set_title('Robot Trajectory')
        axes[0, 0].set_xlabel('X (m)')
        axes[0, 0].set_ylabel('Y (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)

        # Processing time over time
        if self.processing_times:
            times_ms = [t*1000 for t in list(self.processing_times)[-100:]]  # Last 100 samples
            time_steps = range(len(times_ms))
            axes[0, 1].plot(time_steps, times_ms, 'r-', linewidth=1)
            axes[0, 1].axhline(y=1000/10, color='g', linestyle='--', label='Required (10Hz)')
            axes[0, 1].set_title('Processing Time')
            axes[0, 1].set_xlabel('Step')
            axes[0, 1].set_ylabel('Time (ms)')
            axes[0, 1].legend()
            axes[0, 1].grid(True, alpha=0.3)

        # Number of objects detected over time (simulated)
        num_objects = [len(self.perception_system.detected_objects) for _ in range(len(traj))]
        axes[1, 0].plot(range(len(num_objects)), num_objects, 'g-', linewidth=2)
        axes[1, 0].set_title('Objects Detected Over Time')
        axes[1, 0].set_xlabel('Step')
        axes[1, 0].set_ylabel('Number of Objects')
        axes[1, 0].grid(True, alpha=0.3)

        # Final environment state
        final_summary = self.perception_system.get_environment_summary()
        axes[1, 1].axis('off')
        summary_text = f"""Final Simulation State:

Simulation Time: {self.simulation_time:.1f}s
Total Steps: {len(traj)}
Final Robot Pose: ({final_summary['robot_pose'][0]:.2f}, {final_summary['robot_pose'][1]:.2f}, {final_summary['robot_pose'][2]:.2f})
Total Objects Detected: {final_summary['num_detected_objects']}
Map Coverage: {final_summary['map_coverage']:.1%}
Free Space Ratio: {final_summary['free_space_ratio']:.1%}

Performance:
Average Processing Time: {np.mean(self.processing_times)*1000:.2f}ms
Required Processing Time: {self.time_step*1000:.2f}ms
Real-time Capable: {'Yes' if np.mean(self.processing_times) < self.time_step else 'No'}"""

        axes[1, 1].text(0.1, 0.5, summary_text, fontsize=11, verticalalignment='center',
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))

        plt.tight_layout()
        plt.show()

# Run the real-time simulation
simulator = RealTimePerceptionSimulator()
simulator.run_simulation(duration=5.0)  # Run for 5 seconds
simulator.visualize_simulation_results()
```

## Conclusion

These practical examples demonstrate the implementation of key environmental perception concepts for humanoid robots:

1. **Point Cloud Processing**: Techniques for filtering, segmenting, and analyzing 3D point cloud data
2. **Object Recognition**: Methods for identifying and classifying objects in the environment
3. **Mapping**: Creating occupancy grid maps from sensor data
4. **Path Planning**: Algorithms for finding safe and efficient paths through the environment
5. **Sensor Fusion**: Combining data from multiple sensors to create coherent state estimates
6. **Complete Systems**: Integration of multiple perception components into functional systems
7. **Real-time Processing**: Considerations for real-time performance and system integration

Students should experiment with these examples, modify parameters, and observe the effects on system behavior. Understanding both the theoretical concepts and their practical implementation is crucial for developing effective environmental perception systems for humanoid robots.