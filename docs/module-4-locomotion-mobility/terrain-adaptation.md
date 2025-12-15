# Terrain Adaptation and Footstep Planning

## Introduction: Navigating Complex Environments

Terrain adaptation is crucial for humanoid robots to operate effectively in real-world environments. Unlike structured indoor spaces, outdoor and human environments present diverse challenges including uneven surfaces, obstacles, slopes, stairs, and dynamic conditions. This section explores the algorithms and techniques that enable robots to perceive terrain, plan appropriate footstep sequences, and adapt their locomotion patterns in real-time.

### The Challenge of Terrain Adaptation

Terrain adaptation involves multiple interconnected challenges:
- **Perception**: Understanding the 3D structure and properties of the terrain
- **Planning**: Generating stable and efficient footstep sequences
- **Control**: Adapting gait patterns to match terrain characteristics
- **Real-time Response**: Reacting to unexpected terrain features

## Terrain Perception and Classification

### 3D Terrain Mapping

Creating accurate representations of the terrain environment:

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN

class TerrainMapper:
    def __init__(self, resolution=0.05, map_size=10.0):
        self.resolution = resolution
        self.map_size = map_size
        self.grid_size = int(map_size / resolution)
        self.terrain_map = np.full((self.grid_size, self.grid_size), np.nan)
        self.height_map = np.full((self.grid_size, self.grid_size), np.nan)
        self.normal_map = np.full((self.grid_size, self.grid_size, 3), np.nan)
        self.roughness_map = np.full((self.grid_size, self.grid_size), 0.0)
        self.traversability_map = np.ones((self.grid_size, self.grid_size))  # 1 = traversable, 0 = not

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x + self.map_size/2) / self.resolution)
        grid_y = int((y + self.map_size/2) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        x = grid_x * self.resolution - self.map_size/2
        y = grid_y * self.resolution - self.map_size/2
        return x, y

    def update_with_point_cloud(self, points, colors=None):
        """Update terrain map with new point cloud data"""
        for point in points:
            x, y, z = point
            grid_x, grid_y = self.world_to_grid(x, y)

            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                # Update height map
                if np.isnan(self.height_map[grid_x, grid_y]):
                    self.height_map[grid_x, grid_y] = z
                else:
                    # Average multiple measurements
                    self.height_map[grid_x, grid_y] = 0.7 * self.height_map[grid_x, grid_y] + 0.3 * z

        # Calculate terrain properties
        self.calculate_terrain_properties()

    def calculate_terrain_properties(self):
        """Calculate terrain properties like slope, roughness, etc."""
        for i in range(1, self.grid_size - 1):
            for j in range(1, self.grid_size - 1):
                if not np.isnan(self.height_map[i, j]):
                    # Calculate local slope (normal vector)
                    neighbors = [
                        self.height_map[i-1, j],
                        self.height_map[i+1, j],
                        self.height_map[i, j-1],
                        self.height_map[i, j+1]
                    ]

                    valid_neighbors = [h for h in neighbors if not np.isnan(h)]
                    if len(valid_neighbors) >= 3:
                        # Calculate gradients
                        dz_dx = (self.height_map[i+1, j] - self.height_map[i-1, j]) / (2 * self.resolution)
                        dz_dy = (self.height_map[i, j+1] - self.height_map[i, j-1]) / (2 * self.resolution)

                        # Normal vector (pointing upward)
                        normal = np.array([-dz_dx, -dz_dy, 1.0])
                        normal = normal / np.linalg.norm(normal)
                        self.normal_map[i, j] = normal

                        # Calculate roughness (local height variance)
                        height_values = [self.height_map[i+di, j+dj]
                                       for di in [-1,0,1] for dj in [-1,0,1]
                                       if 0 <= i+di < self.grid_size and 0 <= j+dj < self.grid_size
                                       and not np.isnan(self.height_map[i+di, j+dj])]

                        if len(height_values) > 1:
                            self.roughness_map[i, j] = np.std(height_values)

    def classify_terrain_type(self, x, y):
        """Classify terrain type at given location"""
        grid_x, grid_y = self.world_to_grid(x, y)

        if not (0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size):
            return 'unknown'

        height = self.height_map[grid_x, grid_y]
        if np.isnan(height):
            return 'unknown'

        # Calculate local properties
        slope = np.arccos(self.normal_map[grid_x, grid_y, 2]) if not np.isnan(self.normal_map[grid_x, grid_y, 2]) else 0
        roughness = self.roughness_map[grid_x, grid_y]

        # Classify based on properties
        if slope > np.radians(30):  # Very steep
            return 'steep_slope'
        elif slope > np.radians(15):  # Moderate slope
            return 'slope'
        elif roughness > 0.05:  # Rough terrain
            return 'rough'
        elif roughness > 0.02:  # Slightly rough
            return 'uneven'
        else:
            return 'flat'

    def get_traversability_cost(self, x, y, robot_params=None):
        """Get traversability cost for given location"""
        if robot_params is None:
            robot_params = {'max_step_height': 0.1, 'max_slope': np.radians(20)}

        grid_x, grid_y = self.world_to_grid(x, y)

        if not (0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size):
            return float('inf')  # Outside map

        # Check if point is valid
        if np.isnan(self.height_map[grid_x, grid_y]):
            return float('inf')  # No data

        # Calculate local slope
        slope = np.arccos(self.normal_map[grid_x, grid_y, 2]) if not np.isnan(self.normal_map[grid_x, grid_y, 2]) else 0
        roughness = self.roughness_map[grid_x, grid_y]

        # Calculate cost based on terrain properties
        cost = 1.0  # Base cost

        # Slope cost
        if slope > robot_params['max_slope']:
            cost += 10.0  # High penalty for steep slopes
        else:
            cost += slope / robot_params['max_slope'] * 2.0  # Graduated penalty

        # Roughness cost
        cost += roughness * 10.0

        return cost

    def visualize_terrain(self):
        """Visualize the terrain map"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))

        # Height map
        im1 = axes[0, 0].imshow(self.height_map, cmap='terrain', origin='lower',
                               extent=[-self.map_size/2, self.map_size/2,
                                     -self.map_size/2, self.map_size/2])
        axes[0, 0].set_title('Height Map')
        axes[0, 0].set_xlabel('X (m)')
        axes[0, 0].set_ylabel('Y (m)')
        plt.colorbar(im1, ax=axes[0, 0])

        # Slope map (from normal vectors)
        slope_map = np.zeros((self.grid_size, self.grid_size))
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                if not np.isnan(self.normal_map[i, j, 2]):
                    slope_map[i, j] = np.arccos(self.normal_map[i, j, 2])

        im2 = axes[0, 1].imshow(slope_map, cmap='hot', origin='lower',
                               extent=[-self.map_size/2, self.map_size/2,
                                     -self.map_size/2, self.map_size/2])
        axes[0, 1].set_title('Slope Map (radians)')
        axes[0, 1].set_xlabel('X (m)')
        axes[0, 1].set_ylabel('Y (m)')
        plt.colorbar(im2, ax=axes[0, 1])

        # Roughness map
        im3 = axes[1, 0].imshow(self.roughness_map, cmap='viridis', origin='lower',
                               extent=[-self.map_size/2, self.map_size/2,
                                     -self.map_size/2, self.map_size/2])
        axes[1, 0].set_title('Roughness Map')
        axes[1, 0].set_xlabel('X (m)')
        axes[1, 0].set_ylabel('Y (m)')
        plt.colorbar(im3, ax=axes[1, 0])

        # Traversability map
        im4 = axes[1, 1].imshow(self.traversability_map, cmap='RdYlGn_r', origin='lower',
                               extent=[-self.map_size/2, self.map_size/2,
                                     -self.map_size/2, self.map_size/2])
        axes[1, 1].set_title('Traversability Map')
        axes[1, 1].set_xlabel('X (m)')
        axes[1, 1].set_ylabel('Y (m)')
        plt.colorbar(im4, ax=axes[1, 1])

        plt.tight_layout()
        plt.show()

# Example: Create and update terrain map
terrain_mapper = TerrainMapper(resolution=0.1, map_size=5.0)

# Generate sample terrain with different features
np.random.seed(42)
x = np.linspace(-2, 2, 100)
y = np.linspace(-2, 2, 100)
X, Y = np.meshgrid(x, y)

# Create terrain with flat area, slope, and rough patch
Z = np.zeros_like(X)
Z[X > 0.5] = 0.1  # Small step
Z[(X > -0.5) & (X < 0.5) & (Y > 0.5)] = 0.2  # Raised platform
Z[(X > -1) & (X < 0) & (Y > -1) & (Y < 0)] += 0.1 * np.random.random((50, 50))[:50, :50]  # Rough patch

# Add a slope
Z += 0.3 * X  # Gentle slope

# Flatten the terrain for point cloud generation
points = []
for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        points.append([X[i, j], Y[i, j], Z[i, j]])

terrain_mapper.update_with_point_cloud(points)

# Classify terrain at specific points
test_points = [(0, 0), (1, 0), (-0.5, 0.75), (-0.5, -0.5)]
for x, y in test_points:
    terrain_type = terrain_mapper.classify_terrain_type(x, y)
    cost = terrain_mapper.get_traversability_cost(x, y)
    print(f"Point ({x}, {y}): {terrain_type}, cost = {cost:.2f}")

# terrain_mapper.visualize_terrain()  # Uncomment to visualize
```

### Obstacle Detection and Classification

Identifying and categorizing obstacles in the environment:

```python
class ObstacleDetector:
    def __init__(self, min_obstacle_height=0.1, max_obstacle_height=1.0):
        self.min_obstacle_height = min_obstacle_height
        self.max_obstacle_height = max_obstacle_height
        self.obstacles = []
        self.ground_level = 0.0

    def detect_obstacles(self, point_cloud, ground_level=0.0):
        """Detect obstacles from point cloud data"""
        self.ground_level = ground_level
        self.obstacles = []

        # Group points that are close together (using DBSCAN clustering)
        clustering = DBSCAN(eps=0.2, min_samples=10).fit(point_cloud)
        labels = clustering.labels_

        for label in set(labels):
            if label == -1:  # Noise points
                continue

            # Get points in this cluster
            cluster_points = point_cloud[labels == label]

            # Calculate cluster properties
            center = np.mean(cluster_points, axis=0)
            bbox_min = np.min(cluster_points, axis=0)
            bbox_max = np.max(cluster_points, axis=0)
            dimensions = bbox_max - bbox_min

            # Check if it's a valid obstacle (above ground and within height limits)
            if (bbox_min[2] > ground_level + self.min_obstacle_height and
                bbox_max[2] < ground_level + self.max_obstacle_height and
                len(cluster_points) > 20):  # Minimum size threshold

                obstacle = {
                    'center': center,
                    'bbox_min': bbox_min,
                    'bbox_max': bbox_max,
                    'dimensions': dimensions,
                    'points': cluster_points,
                    'type': self.classify_obstacle_type(dimensions),
                    'passable': self.is_obstacle_passable(dimensions)
                }

                self.obstacles.append(obstacle)

        return self.obstacles

    def classify_obstacle_type(self, dimensions):
        """Classify obstacle type based on dimensions"""
        height = dimensions[2]
        width = dimensions[0]
        length = dimensions[1]

        if height < 0.1:  # Low obstacle
            if width < 0.2 and length < 0.2:
                return 'small_object'
            else:
                return 'step'
        elif height < 0.3:  # Medium obstacle
            return 'curb'
        elif height < 0.8:  # High obstacle
            return 'barrier'
        else:  # Very high obstacle
            return 'wall'

    def is_obstacle_passable(self, dimensions):
        """Determine if obstacle is passable by stepping over"""
        height = dimensions[2]

        # For humanoid robot, passable obstacles are typically < 15cm high
        return height < 0.15

    def get_obstacle_traversability(self, robot_params=None):
        """Get traversability information for each obstacle"""
        if robot_params is None:
            robot_params = {
                'leg_length': 0.9,
                'max_step_height': 0.15,
                'robot_width': 0.3
            }

        traversability_info = []
        for obstacle in self.obstacles:
            info = {
                'obstacle': obstacle,
                'can_step_over': obstacle['dimensions'][2] <= robot_params['max_step_height'],
                'can_walk_around': True,  # Assume walk-around is possible
                'navigation_strategy': self.determine_navigation_strategy(obstacle, robot_params)
            }
            traversability_info.append(info)

        return traversability_info

    def determine_navigation_strategy(self, obstacle, robot_params):
        """Determine best navigation strategy for obstacle"""
        height = obstacle['dimensions'][2]

        if height <= robot_params['max_step_height']:
            return 'step_over'
        elif height <= 0.4:  # Crawlable height
            return 'step_approach'  # Approach and step up
        else:
            return 'go_around'  # Navigate around obstacle

    def get_clear_path_around_obstacles(self, start, goal, robot_radius=0.2):
        """Get path that avoids obstacles"""
        # This would implement path planning around obstacles
        # For this example, return a simple path
        return [start, goal]  # Placeholder

# Example: Obstacle detection
obstacle_detector = ObstacleDetector()

# Create sample point cloud with obstacles
sample_points = []
# Ground plane
for i in range(20):
    for j in range(20):
        sample_points.append([i*0.2 - 2, j*0.2 - 2, 0.0])

# Add some obstacles
# Small box
for i in range(3):
    for j in range(3):
        for k in range(5):
            sample_points.append([-0.5 + i*0.1, 0.5 + j*0.1, k*0.02])

# Step
for i in range(10):
    for j in range(3):
        sample_points.append([0.5 + i*0.1, -0.5 + j*0.1, 0.1 if i > 5 else 0.0])

point_cloud = np.array(sample_points)

# Detect obstacles
obstacles = obstacle_detector.detect_obstacles(point_cloud, ground_level=0.0)
print(f"Detected {len(obstacles)} obstacles")

for i, obstacle in enumerate(obstacles):
    print(f"Obstacle {i+1}: {obstacle['type']}, dimensions = {obstacle['dimensions']}, "
          f"passable = {obstacle['passable']}")
```

## Footstep Planning Algorithms

### A* Footstep Planner

Planning optimal footstep sequences using A* algorithm:

```python
import heapq

class FootstepPlanner:
    def __init__(self, terrain_mapper, step_limits=None):
        self.terrain_mapper = terrain_mapper
        if step_limits is None:
            step_limits = {
                'max_forward': 0.3,    # Maximum forward step
                'max_lateral': 0.2,    # Maximum lateral step
                'max_rotation': np.pi/4,  # Maximum rotation
                'min_forward': 0.05    # Minimum forward step
            }
        self.step_limits = step_limits
        self.robot_foot_size = 0.15  # Foot size for collision checking

    def heuristic(self, pos1, pos2):
        """Heuristic function for A* (Euclidean distance)"""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def get_neighbors(self, current_pos, current_yaw):
        """Get possible next footstep positions"""
        neighbors = []

        # Generate possible step locations
        step_configs = [
            # Forward steps
            (self.step_limits['max_forward'], 0, 0),  # Forward
            (self.step_limits['max_forward']*0.7, 0, 0),  # Shorter forward
            (self.step_limits['max_forward']*0.5, 0, 0),  # Even shorter

            # Lateral steps
            (0, self.step_limits['max_lateral'], 0),  # Left
            (0, -self.step_limits['max_lateral'], 0),  # Right
            (0, self.step_limits['max_lateral']*0.5, 0),  # Small left
            (0, -self.step_limits['max_lateral']*0.5, 0),  # Small right

            # Diagonal steps
            (self.step_limits['max_forward']*0.7, self.step_limits['max_lateral']*0.5, 0),  # Forward-left
            (self.step_limits['max_forward']*0.7, -self.step_limits['max_lateral']*0.5, 0),  # Forward-right

            # Rotational steps
            (0, 0, self.step_limits['max_rotation']),  # Rotate left
            (0, 0, -self.step_limits['max_rotation']),  # Rotate right
        ]

        for dx, dy, dtheta in step_configs:
            # Transform step to world coordinates based on current orientation
            world_dx = dx * np.cos(current_yaw) - dy * np.sin(current_yaw)
            world_dy = dx * np.sin(current_yaw) + dy * np.cos(current_yaw)

            new_x = current_pos[0] + world_dx
            new_y = current_pos[1] + world_dy
            new_yaw = current_yaw + dtheta

            # Keep yaw in [-π, π]
            new_yaw = np.arctan2(np.sin(new_yaw), np.cos(new_yaw))

            neighbors.append(((new_x, new_y), new_yaw))

        return neighbors

    def is_valid_footstep(self, pos, yaw):
        """Check if footstep is valid (not in collision, stable, etc.)"""
        x, y = pos

        # Check terrain traversability
        cost = self.terrain_mapper.get_traversability_cost(x, y)
        if cost == float('inf'):
            return False

        # Check slope stability (too steep)
        grid_x, grid_y = self.terrain_mapper.world_to_grid(x, y)
        if (0 <= grid_x < self.terrain_mapper.grid_size and
            0 <= grid_y < self.terrain_mapper.grid_size):
            if not np.isnan(self.terrain_mapper.normal_map[grid_x, grid_y, 2]):
                slope = np.arccos(self.terrain_mapper.normal_map[grid_x, grid_y, 2])
                if slope > np.radians(30):  # Too steep
                    return False

        return True

    def plan_footsteps(self, start_pos, start_yaw, goal_pos, max_steps=50):
        """Plan sequence of footsteps from start to goal"""
        # A* algorithm for footstep planning
        open_set = [(0, (start_pos, start_yaw, []))]  # (f_score, (pos, yaw, path))
        closed_set = set()

        step_count = 0
        while open_set and step_count < max_steps:
            current_f, (current_pos, current_yaw, current_path) = heapq.heappop(open_set)

            # Check if we're close to goal
            if np.sqrt((current_pos[0] - goal_pos[0])**2 + (current_pos[1] - goal_pos[1])**2) < 0.2:
                return current_path + [(current_pos, current_yaw)]

            # Add current state to closed set
            state_key = (int(current_pos[0]/0.1), int(current_pos[1]/0.1), int(current_yaw/0.1))
            if state_key in closed_set:
                continue
            closed_set.add(state_key)

            # Get possible next steps
            neighbors = self.get_neighbors(current_pos, current_yaw)
            for next_pos, next_yaw in neighbors:
                if not self.is_valid_footstep(next_pos, next_yaw):
                    continue

                # Calculate cost
                g_score = len(current_path) + 1  # Step count cost
                h_score = self.heuristic(next_pos, goal_pos)  # Distance to goal
                f_score = g_score + h_score

                # Add to open set
                new_path = current_path + [(current_pos, current_yaw)]
                heapq.heappush(open_set, (f_score, (next_pos, next_yaw, new_path)))

            step_count += 1

        return None  # No path found

    def plan_alternating_feet(self, start_left, start_right, goal_pos, num_steps=10):
        """Plan footsteps alternating between left and right feet"""
        footsteps = []

        # Start with left foot
        current_left = start_left[:2]
        current_left_yaw = start_left[2] if len(start_left) > 2 else 0
        current_right = start_right[:2]
        current_right_yaw = start_right[2] if len(start_right) > 2 else 0

        # Plan steps alternating feet
        for i in range(num_steps):
            if i % 2 == 0:  # Left foot step
                # Plan to move left foot toward goal
                goal_for_left = (
                    goal_pos[0] - 0.1 + 0.2 * (i / num_steps),  # Adjust for alternating pattern
                    goal_pos[1] + (0.1 if i % 4 < 2 else -0.1)  # Slight lateral adjustment
                )

                path = self.plan_footsteps(current_left, current_left_yaw, goal_for_left, max_steps=5)
                if path and len(path) > 1:
                    current_left, current_left_yaw = path[1]  # Next step
                    footsteps.append(('left', current_left[0], current_left[1], current_left_yaw))
                else:
                    # Fallback: simple forward step
                    new_x = current_left[0] + 0.2 * np.cos(current_left_yaw)
                    new_y = current_left[1] + 0.2 * np.sin(current_left_yaw)
                    current_left = (new_x, new_y)
                    footsteps.append(('left', new_x, new_y, current_left_yaw))
            else:  # Right foot step
                # Plan to move right foot toward goal
                goal_for_right = (
                    goal_pos[0] - 0.1 + 0.2 * (i / num_steps),
                    goal_pos[1] + (0.1 if (i+1) % 4 < 2 else -0.1)
                )

                path = self.plan_footsteps(current_right, current_right_yaw, goal_for_right, max_steps=5)
                if path and len(path) > 1:
                    current_right, current_right_yaw = path[1]
                    footsteps.append(('right', current_right[0], current_right[1], current_right_yaw))
                else:
                    # Fallback: simple forward step
                    new_x = current_right[0] + 0.2 * np.cos(current_right_yaw)
                    new_y = current_right[1] + 0.2 * np.sin(current_right_yaw)
                    current_right = (new_x, new_y)
                    footsteps.append(('right', new_x, new_y, current_right_yaw))

        return footsteps

# Example: Footstep planning
footstep_planner = FootstepPlanner(terrain_mapper)

# Plan footsteps from start to goal
start_pos = (0.0, 0.0)
start_yaw = 0.0
goal_pos = (2.0, 0.0)

footsteps = footstep_planner.plan_alternating_feet(
    (0.0, 0.1, 0.0),  # Start left foot
    (0.0, -0.1, 0.0),  # Start right foot
    goal_pos,
    num_steps=8
)

print(f"Planned {len(footsteps)} footsteps:")
for i, (foot, x, y, yaw) in enumerate(footsteps):
    print(f"  Step {i+1}: {foot} foot at ({x:.2f}, {y:.2f}), yaw={yaw:.2f}")
```

### Dynamic Footstep Adjustment

Adapting footstep plans based on real-time perception:

```python
class AdaptiveFootstepAdjuster:
    def __init__(self, base_planner):
        self.base_planner = base_planner
        self.current_footsteps = []
        self.executed_steps = []
        self.adjustment_threshold = 0.05  # 5cm threshold for adjustment
        self.lookahead_distance = 1.0  # Look ahead 1m

    def update_terrain_map(self, new_point_cloud, robot_pos):
        """Update terrain map with new sensor data"""
        self.base_planner.terrain_mapper.update_with_point_cloud(new_point_cloud)

    def detect_terrain_changes(self, current_pos, current_yaw):
        """Detect changes in terrain that require footstep adjustment"""
        changes_detected = []

        # Check upcoming footsteps against new terrain data
        for i, (foot, x, y, yaw) in enumerate(self.current_footsteps):
            if i < len(self.executed_steps):  # Already executed
                continue

            # Calculate expected terrain properties at foot position
            expected_cost = self.base_planner.terrain_mapper.get_traversability_cost(x, y)

            # Check if terrain has changed significantly
            # This would involve comparing with updated map
            if expected_cost > 5.0:  # High cost terrain detected
                changes_detected.append({
                    'step_index': i,
                    'position': (x, y),
                    'current_cost': expected_cost,
                    'adjustment_needed': True
                })

        return changes_detected

    def adjust_footsteps(self, detected_changes, robot_pos, robot_yaw):
        """Adjust footstep plan based on detected changes"""
        adjusted_footsteps = self.current_footsteps.copy()

        for change in detected_changes:
            step_idx = change['step_index']
            if step_idx >= len(adjusted_footsteps):
                continue

            original_foot, orig_x, orig_y, orig_yaw = adjusted_footsteps[step_idx]

            # Find alternative footstep location
            alternative_pos = self.find_alternative_footstep(
                (orig_x, orig_y), robot_pos, robot_yaw
            )

            if alternative_pos:
                adjusted_footsteps[step_idx] = (
                    original_foot, alternative_pos[0], alternative_pos[1], orig_yaw
                )

        return adjusted_footsteps

    def find_alternative_footstep(self, original_pos, robot_pos, robot_yaw):
        """Find alternative footstep location near original"""
        x, y = original_pos

        # Search in a local area around the original position
        search_radius = 0.3  # 30cm search radius
        search_resolution = 0.05  # 5cm resolution

        best_pos = None
        best_cost = float('inf')

        for dx in np.arange(-search_radius, search_radius, search_resolution):
            for dy in np.arange(-search_radius, search_radius, search_resolution):
                test_x = x + dx
                test_y = y + dy

                # Check if this position is valid
                if self.base_planner.is_valid_footstep((test_x, test_y), robot_yaw):
                    cost = self.base_planner.terrain_mapper.get_traversability_cost(test_x, test_y)
                    # Add distance penalty to stay close to original
                    distance_penalty = np.sqrt(dx**2 + dy**2) * 2.0
                    total_cost = cost + distance_penalty

                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_pos = (test_x, test_y)

        return best_pos

    def replan_around_obstacle(self, obstacle_pos, robot_pos, goal_pos):
        """Replan footsteps to go around an obstacle"""
        # This would implement local replanning around detected obstacles
        # For this example, we'll return a simple detour
        detour_offset = 0.3  # 30cm detour

        # Calculate detour direction (perpendicular to obstacle-robot vector)
        to_obstacle = np.array(obstacle_pos[:2]) - np.array(robot_pos[:2])
        detour_direction = np.array([-to_obstacle[1], to_obstacle[0]])  # Perpendicular
        detour_direction = detour_direction / np.linalg.norm(detour_direction)

        # New path goes around obstacle
        detour_point = np.array(obstacle_pos[:2]) + detour_direction * detour_offset
        return [(robot_pos[0], robot_pos[1]), detour_point, goal_pos]

    def execute_step_and_update(self, step_index, robot_pos, robot_yaw, sensor_data=None):
        """Execute a step and update plan based on new information"""
        if sensor_data:
            # Update terrain map with new sensor data
            self.update_terrain_map(sensor_data['point_cloud'], robot_pos)

        # Add executed step to history
        if step_index < len(self.current_footsteps):
            self.executed_steps.append(self.current_footsteps[step_index])

        # Check for terrain changes that require adjustment
        changes = self.detect_terrain_changes(robot_pos, robot_yaw)

        if changes:
            print(f"Detected {len(changes)} terrain changes, adjusting plan...")
            self.current_footsteps = self.adjust_footsteps(changes, robot_pos, robot_yaw)

        return self.current_footsteps

# Example: Adaptive footstep adjustment
adaptive_adjuster = AdaptiveFootstepAdjuster(footstep_planner)

# Set initial plan
adaptive_adjuster.current_footsteps = footsteps

# Simulate execution with some terrain changes
robot_pos = (0.0, 0.0, 0.85)  # x, y, z
robot_yaw = 0.0

for step_idx in range(min(3, len(footsteps))):  # Execute first 3 steps
    print(f"Executing step {step_idx + 1}")

    # Simulate sensor data (in real system, this would come from sensors)
    sensor_data = {
        'point_cloud': np.array([[robot_pos[0] + 0.1, robot_pos[1], robot_pos[2] - 0.85],
                                [robot_pos[0] + 0.2, robot_pos[1], robot_pos[2] - 0.85]])
    }

    # Execute step and update
    updated_plan = adaptive_adjuster.execute_step_and_update(
        step_idx, robot_pos, robot_yaw, sensor_data
    )

    # Update robot position (simplified)
    if step_idx < len(updated_plan):
        foot, x, y, yaw = updated_plan[step_idx]
        robot_pos = (x, y, robot_pos[2])
        robot_yaw = yaw

    print(f"Remaining footsteps: {len(updated_plan) - step_idx - 1}")
```

## Stair and Step Negotiation

### Stair Detection and Negotiation Planning

Handling stairs and discrete height changes:

```python
class StairNegotiationPlanner:
    def __init__(self):
        self.stair_parameters = {
            'typical_rise': 0.17,    # 17cm typical stair rise
            'typical_run': 0.28,    # 28cm typical stair run
            'max_rise': 0.25,       # Maximum safe rise
            'min_run': 0.15,        # Minimum safe run
            'nose_depth': 0.03      # Tread nose depth
        }
        self.stairs_detected = []
        self.current_step = 0

    def detect_stairs_from_point_cloud(self, point_cloud, ground_level=0.0):
        """Detect stairs from point cloud data"""
        self.stairs_detected = []

        # Group points by height to identify discrete levels
        z_values = point_cloud[:, 2]
        unique_heights = np.unique(np.round(z_values, decimals=2))

        # Look for regular height intervals (stair pattern)
        possible_rise_values = []
        for i in range(len(unique_heights)):
            for j in range(i + 1, len(unique_heights)):
                rise = unique_heights[j] - unique_heights[i]
                if self.stair_parameters['min_run'] < rise < self.stair_parameters['max_rise']:
                    possible_rise_values.append(rise)

        # Find the most common rise value (likely stair height)
        if possible_rise_values:
            unique_rises, counts = np.unique(possible_rise_values, return_counts=True)
            most_common_rise = unique_rises[np.argmax(counts)]

            # Identify stair locations based on this rise
            stair_heights = []
            for height in unique_heights:
                if any(abs(height - h - most_common_rise) < 0.02 for h in stair_heights) or len(stair_heights) == 0:
                    stair_heights.append(height)

            # Create stair structure
            if len(stair_heights) > 1:
                stair_heights = sorted(stair_heights)
                for i in range(len(stair_heights)):
                    self.stairs_detected.append({
                        'step_number': i,
                        'height': stair_heights[i],
                        'rise': most_common_rise if i > 0 else 0,
                        'tread_points': point_cloud[abs(point_cloud[:, 2] - stair_heights[i]) < 0.02]
                    })

        return self.stairs_detected

    def plan_stair_ascent(self, start_pos, num_steps):
        """Plan footstep sequence for stair ascent"""
        footsteps = []
        current_x, current_y = start_pos

        for i in range(num_steps):
            # Left foot on step i
            left_x = current_x + i * self.stair_parameters['typical_run']
            left_y = current_y + (0.1 if i % 2 == 0 else -0.1)  # Alternate lateral position
            left_z = self.stair_parameters['typical_rise'] * (i + 1)
            footsteps.append(('left', left_x, left_y, left_z, 0.0))

            # Right foot on step i (if not the last step)
            if i < num_steps - 1:
                right_x = current_x + i * self.stair_parameters['typical_run']
                right_y = current_y + (-0.1 if i % 2 == 0 else 0.1)
                right_z = self.stair_parameters['typical_rise'] * (i + 1)
                footsteps.append(('right', right_x, right_y, right_z, 0.0))

        return footsteps

    def plan_stair_descent(self, start_pos, num_steps):
        """Plan footstep sequence for stair descent"""
        footsteps = []
        current_x, current_y = start_pos

        # Start from top of stairs
        start_z = self.stair_parameters['typical_rise'] * num_steps

        for i in range(num_steps):
            step_level = num_steps - i - 1  # Descending
            step_z = self.stair_parameters['typical_rise'] * step_level

            # Left foot on step
            left_x = current_x + i * self.stair_parameters['typical_run']
            left_y = current_y + (0.1 if i % 2 == 0 else -0.1)
            footsteps.append(('left', left_x, left_y, step_z, 0.0))

            # Right foot on same step
            right_x = current_x + i * self.stair_parameters['typical_run']
            right_y = current_y + (-0.1 if i % 2 == 0 else 0.1)
            footsteps.append(('right', right_x, right_y, step_z, 0.0))

        return footsteps

    def validate_stair_footstep(self, foot_pos, stair_info):
        """Validate if footstep is safe on stairs"""
        x, y, z = foot_pos
        step_height = stair_info['height']

        # Check if foot is properly positioned on step
        height_error = abs(z - step_height)
        if height_error > 0.05:  # 5cm tolerance
            return False, f"Height error: {height_error:.3f}m"

        # Check if foot is within step boundaries
        # This would require more detailed step geometry
        return True, "Valid"

    def generate_stair_negotiation_pattern(self, stair_sequence, direction='up'):
        """Generate complete stair negotiation pattern"""
        pattern = {
            'approach': self.plan_approach_to_stairs(stair_sequence[0] if stair_sequence else None),
            'ascent': self.plan_stair_ascent_pattern(stair_sequence, direction),
            'departure': self.plan_departure_from_stairs(stair_sequence[-1] if stair_sequence else None)
        }
        return pattern

    def plan_approach_to_stairs(self, first_step_info):
        """Plan approach steps before reaching stairs"""
        if not first_step_info:
            return []

        approach_steps = []
        # Plan 2-3 approach steps leading to first stair
        for i in range(3):
            x = -0.3 * (3 - i)  # Approach from -0.9m
            y = 0.0
            z = 0.0  # Ground level
            approach_steps.append(('left' if i % 2 == 0 else 'right', x, y, z, 0.0))

        return approach_steps

    def plan_departure_from_stairs(self, last_step_info):
        """Plan departure steps after leaving stairs"""
        if not last_step_info:
            return []

        departure_steps = []
        # Plan 2-3 departure steps after last stair
        last_x = last_step_info.get('tread_points', np.array([[0, 0, 0]]))[-1, 0] if 'tread_points' in last_step_info else 0

        for i in range(3):
            x = last_x + 0.3 * (i + 1)
            y = 0.0
            z = last_step_info['height'] if 'height' in last_step_info else 0.0
            departure_steps.append(('left' if i % 2 == 0 else 'right', x, y, z, 0.0))

        return departure_steps

    def plan_stair_ascent_pattern(self, stair_sequence, direction):
        """Plan the actual stair negotiation pattern"""
        if direction == 'up':
            return self.plan_stair_ascent((0, 0), len(stair_sequence))
        else:
            return self.plan_stair_descent((0, 0), len(stair_sequence))

# Example: Stair negotiation
stair_planner = StairNegotiationPlanner()

# Create sample stair point cloud
stair_points = []
for step in range(5):  # 5 steps
    z_height = step * 0.17  # 17cm rise per step
    for x in np.arange(0, 0.3, 0.02):  # 30cm run
        for y in np.arange(-0.1, 0.1, 0.02):  # 20cm width
            stair_points.append([x + step * 0.3, y, z_height])

stair_point_cloud = np.array(stair_points)

# Detect stairs
detected_stairs = stair_planner.detect_stairs_from_point_cloud(stair_point_cloud, ground_level=0.0)
print(f"Detected {len(detected_stairs)} stairs")

# Plan stair ascent
ascent_plan = stair_planner.plan_stair_ascent((0, 0), len(detected_stairs))
print(f"Stair ascent plan: {len(ascent_plan)} footsteps")

for i, (foot, x, y, z, yaw) in enumerate(ascent_plan[:6]):  # Show first 6 steps
    print(f"  Step {i+1}: {foot} foot at ({x:.2f}, {y:.2f}, {z:.2f})")
```

## Rough Terrain Navigation

### Compliance and Adaptation Strategies

Handling uneven and compliant surfaces:

```python
class RoughTerrainNavigator:
    def __init__(self):
        self.foot_compliance = 0.02  # 2cm foot compliance for uneven surfaces
        self.adaptation_speed = 1.0  # Adaptation rate
        self.step_height_modulation = True
        self.ankle_adaptation = True

    def analyze_terrain_roughness(self, terrain_map, position, radius=0.2):
        """Analyze local terrain roughness around position"""
        x, y = position
        grid_x, grid_y = terrain_map.world_to_grid(x, y)

        # Get local neighborhood
        neighborhood_size = int(radius / terrain_map.resolution)
        roughness_values = []

        for i in range(-neighborhood_size, neighborhood_size + 1):
            for j in range(-neighborhood_size, neighborhood_size + 1):
                nx, ny = grid_x + i, grid_y + j
                if (0 <= nx < terrain_map.grid_size and
                    0 <= ny < terrain_map.grid_size and
                    not np.isnan(terrain_map.height_map[nx, ny])):
                    roughness_values.append(terrain_map.roughness_map[nx, ny])

        if roughness_values:
            avg_roughness = np.mean(roughness_values)
            max_roughness = np.max(roughness_values)
            return avg_roughness, max_roughness
        else:
            return 0.0, 0.0

    def adjust_step_parameters_for_roughness(self, base_params, avg_roughness, max_roughness):
        """Adjust step parameters based on terrain roughness"""
        adjusted_params = base_params.copy()

        # Reduce step length on rough terrain
        roughness_factor = 1.0 - 0.3 * min(1.0, avg_roughness / 0.05)  # Reduce up to 30%
        adjusted_params['step_length'] *= roughness_factor

        # Increase step height for rough terrain
        adjusted_params['swing_height'] += max(0, min(0.05, avg_roughness))

        # Reduce step frequency on very rough terrain
        if max_roughness > 0.08:
            adjusted_params['step_time'] *= 1.2  # Slower steps

        return adjusted_params

    def generate_ankle_adaptation_trajectory(self, foot_position, terrain_normal, step_phase):
        """Generate ankle trajectory to adapt to terrain slope"""
        # Calculate required ankle angles to match terrain slope
        # terrain_normal is the surface normal vector at foot contact point
        if np.linalg.norm(terrain_normal) > 0:
            # Normalize the normal vector
            normal = terrain_normal / np.linalg.norm(terrain_normal)

            # Calculate required roll and pitch angles
            # This is a simplified calculation
            roll_angle = np.arctan2(normal[1], normal[2])
            pitch_angle = np.arctan2(-normal[0], normal[2])

            # Add phase-dependent modulation for natural ankle motion
            phase_modulation = np.sin(step_phase * 2 * np.pi)
            roll_angle *= (1 + 0.3 * phase_modulation)
            pitch_angle *= (1 + 0.3 * phase_modulation)

            return roll_angle, pitch_angle
        else:
            return 0.0, 0.0

    def calculate_com_adaptation(self, terrain_roughness, base_com_trajectory):
        """Calculate CoM trajectory adaptation for rough terrain"""
        adapted_trajectory = base_com_trajectory.copy()

        # Add small lateral sway to help with balance on uneven terrain
        if terrain_roughness > 0.02:
            lateral_amplitude = min(0.03, terrain_roughness * 2)  # Max 3cm sway
            for i, (x, y, z) in enumerate(adapted_trajectory):
                phase = i * 0.2  # Walking phase
                adapted_trajectory[i][1] += lateral_amplitude * np.sin(phase)

        # Adjust CoM height slightly for better stability
        height_adjustment = max(-0.02, -terrain_roughness)  # Lower CoM on rough terrain
        for pos in adapted_trajectory:
            pos[2] += height_adjustment

        return adapted_trajectory

    def generate_compliant_foot_trajectory(self, nominal_trajectory, terrain_roughness):
        """Generate foot trajectory with compliance for uneven surfaces"""
        compliant_trajectory = []

        for point in nominal_trajectory:
            x, y, z = point

            # Add compliance-based height adjustment
            height_adjustment = np.random.normal(0, terrain_roughness * 0.5)
            z_adjusted = z + height_adjustment

            # Ensure foot doesn't go below ground
            grid_x, grid_y = self.terrain_mapper.world_to_grid(x, y) if hasattr(self, 'terrain_mapper') else (0, 0)
            if (0 <= grid_x < 50 and 0 <= grid_y < 50):  # Assuming 50x50 grid
                ground_height = 0  # Placeholder - would use actual terrain map
                z_adjusted = max(z_adjusted, ground_height + 0.01)  # Minimum 1cm above ground

            compliant_trajectory.append([x, y, z_adjusted])

        return np.array(compliant_trajectory)

    def plan_rough_terrain_footsteps(self, start_pos, goal_pos, terrain_map, max_roughness=0.05):
        """Plan footsteps specifically for rough terrain"""
        # Use base planner but with modified parameters
        base_footsteps = self.plan_rough_terrain_sequence(start_pos, goal_pos, max_roughness)

        # Add extra steps if terrain is very rough
        if max_roughness > 0.03:
            # Insert intermediate steps for better stability
            refined_footsteps = []
            for i in range(len(base_footsteps) - 1):
                refined_footsteps.append(base_footsteps[i])

                # Add intermediate step if distance is large
                if i % 2 == 0:  # Only for every other step to maintain alternation
                    pos1 = base_footsteps[i][1:3]  # x, y coordinates
                    pos2 = base_footsteps[i+1][1:3]
                    mid_pos = [(p1 + p2) / 2 for p1, p2 in zip(pos1, pos2)]

                    refined_footsteps.append((
                        base_footsteps[i][0],  # foot type
                        mid_pos[0], mid_pos[1], 0.0  # intermediate position
                    ))
            refined_footsteps.append(base_footsteps[-1])

            return refined_footsteps
        else:
            return base_footsteps

    def plan_rough_terrain_sequence(self, start_pos, goal_pos, max_roughness):
        """Plan a sequence of footsteps for rough terrain"""
        footsteps = []
        current_pos = np.array(start_pos)

        # Calculate direction to goal
        direction = np.array(goal_pos) - current_pos[:2]
        distance = np.linalg.norm(direction)
        direction = direction / distance if distance > 0 else np.array([1, 0])

        # Adjust step size based on roughness
        base_step_size = 0.3
        step_size = base_step_size * (1 - 0.5 * min(1.0, max_roughness / 0.02))

        num_steps = int(distance / step_size) + 1
        step_vector = direction * step_size

        for i in range(num_steps):
            x = current_pos[0] + step_vector[0] * i
            y = current_pos[1] + step_vector[1] * i

            # Add small random variations to help with uneven terrain
            x += np.random.normal(0, max_roughness * 2)
            y += np.random.normal(0, max_roughness * 2)

            foot_type = 'left' if i % 2 == 0 else 'right'
            footsteps.append((foot_type, x, y, 0.0))

        return footsteps

    def evaluate_rough_terrain_traversability(self, path, terrain_map):
        """Evaluate how well a path traverses rough terrain"""
        score = 0.0
        for i, (foot, x, y, z) in enumerate(path):
            # Get terrain properties at this point
            avg_rough, max_rough = self.analyze_terrain_roughness(terrain_map, (x, y))

            # Penalty for very rough terrain
            rough_penalty = max_rough * 100  # Higher roughness = higher penalty

            # Bonus for smoother sections
            smooth_bonus = max(0, 1 - avg_rough * 50)

            score += smooth_bonus - rough_penalty

        return score / len(path) if path else 0

# Example: Rough terrain navigation
rough_navigator = RoughTerrainNavigator()

# Using the terrain map from earlier examples
avg_roughness, max_roughness = rough_navigator.analyze_terrain_roughness(
    terrain_mapper, (0.5, 0.5), radius=0.3
)

print(f"Local terrain roughness at (0.5, 0.5): avg={avg_roughness:.4f}, max={max_roughness:.4f}")

# Adjust step parameters
base_params = {'step_length': 0.3, 'step_time': 0.8, 'swing_height': 0.05}
adjusted_params = rough_navigator.adjust_step_parameters_for_roughness(
    base_params, avg_roughness, max_roughness
)

print(f"Base params: {base_params}")
print(f"Adjusted params: {adjusted_params}")

# Plan footsteps for rough terrain
rough_footsteps = rough_navigator.plan_rough_terrain_footsteps(
    (0, 0), (2, 0), terrain_mapper, max_roughness=max_roughness
)
print(f"Rough terrain footsteps: {len(rough_footsteps)} steps")
```

## Integration with Control Systems

### ROS 2 Integration for Terrain Adaptation

Integrating terrain adaptation with ROS 2 for real-world deployment:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float64MultiArray, Bool
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker

class TerrainAdaptationNode(Node):
    def __init__(self):
        super().__init__('terrain_adaptation_node')

        # Publishers
        self.footstep_pub = self.create_publisher(Float64MultiArray, '/planned_footsteps', 10)
        self.terrain_map_pub = self.create_publisher(MarkerArray, '/terrain_map', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/detected_obstacles', 10)
        self.status_pub = self.create_publisher(Bool, '/terrain_adaptation_active', 10)

        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/points', self.pointcloud_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Point, '/imu_com', self.imu_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.goal_callback, 10
        )

        # Initialize components
        self.terrain_mapper = TerrainMapper(resolution=0.05, map_size=10.0)
        self.obstacle_detector = ObstacleDetector()
        self.footstep_planner = FootstepPlanner(self.terrain_mapper)
        self.adaptive_adjuster = AdaptiveFootstepAdjuster(self.footstep_planner)
        self.stair_planner = StairNegotiationPlanner()
        self.rough_navigator = RoughTerrainNavigator()

        # Robot state
        self.robot_position = np.array([0.0, 0.0, 0.85])
        self.robot_yaw = 0.0
        self.goal_position = None
        self.current_footsteps = []
        self.terrain_adaptation_active = False

        # Timer for terrain processing
        self.process_timer = self.create_timer(0.1, self.process_terrain_data)

        self.get_logger().info('Terrain adaptation node initialized')

    def pointcloud_callback(self, msg):
        """Process incoming point cloud data"""
        try:
            import sensor_msgs.point_cloud2 as pc2
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])

            if points:
                # Update terrain mapper
                self.terrain_mapper.update_with_point_cloud(np.array(points))

                # Detect obstacles
                obstacles = self.obstacle_detector.detect_obstacles(
                    np.array(points), ground_level=self.robot_position[2] - 0.85
                )

                # Update adaptive adjuster with new data
                sensor_data = {'point_cloud': np.array(points)}
                self.adaptive_adjuster.update_terrain_map(
                    sensor_data['point_cloud'], self.robot_position[:2]
                )

                self.get_logger().info(f'Processed point cloud: {len(points)} points, {len(obstacles)} obstacles')

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def imu_callback(self, msg):
        """Process IMU data for robot state"""
        self.robot_position[0] = msg.x
        self.robot_position[1] = msg.y
        self.robot_position[2] = msg.z

    def goal_callback(self, msg):
        """Process navigation goal"""
        self.goal_position = np.array([msg.pose.position.x, msg.pose.position.y])
        self.terrain_adaptation_active = True

        # Plan path to goal
        self.plan_path_to_goal()

    def plan_path_to_goal(self):
        """Plan path to goal considering terrain"""
        if self.goal_position is None:
            return

        # Classify terrain between start and goal
        start_pos = self.robot_position[:2]
        goal_pos = self.goal_position

        # Plan footsteps using the footstep planner
        footsteps = self.footstep_planner.plan_alternating_feet(
            (start_pos[0], start_pos[1] + 0.1, self.robot_yaw),
            (start_pos[0], start_pos[1] - 0.1, self.robot_yaw),
            goal_pos,
            num_steps=20
        )

        if footsteps:
            self.current_footsteps = footsteps
            self.adaptive_adjuster.current_footsteps = footsteps

            # Publish planned footsteps
            footstep_msg = Float64MultiArray()
            footstep_data = []
            for foot, x, y, yaw in footsteps:
                footstep_data.extend([x, y, yaw, 1.0 if foot == 'left' else 0.0])  # Add left/right indicator
            footstep_msg.data = footstep_data
            self.footstep_pub.publish(footstep_msg)

            self.get_logger().info(f'Planned {len(footsteps)} footsteps to goal')

    def process_terrain_data(self):
        """Periodic processing of terrain data"""
        if not self.terrain_adaptation_active:
            return

        # Update terrain classification
        if self.goal_position is not None:
            # Check for terrain changes along the path
            changes = self.adaptive_adjuster.detect_terrain_changes(
                self.robot_position[:2], self.robot_yaw
            )

            if changes:
                self.get_logger().info(f'Detected {len(changes)} terrain changes, adjusting plan')
                self.current_footsteps = self.adaptive_adjuster.adjust_footsteps(
                    changes, self.robot_position[:2], self.robot_yaw
                )

                # Republish updated footsteps
                footstep_msg = Float64MultiArray()
                footstep_data = []
                for foot, x, y, yaw in self.current_footsteps:
                    footstep_data.extend([x, y, yaw, 1.0 if foot == 'left' else 0.0])
                footstep_msg.data = footstep_data
                self.footstep_pub.publish(footstep_msg)

        # Publish terrain map visualization
        self.publish_terrain_visualization()

        # Publish status
        status_msg = Bool()
        status_msg.data = self.terrain_adaptation_active
        self.status_pub.publish(status_msg)

    def publish_terrain_visualization(self):
        """Publish terrain map for visualization"""
        marker_array = MarkerArray()

        # Create markers for terrain features
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'terrain_map'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.scale.x = self.terrain_mapper.resolution
        marker.scale.y = self.terrain_mapper.resolution
        marker.scale.z = 0.01

        # Add points for traversable areas
        for i in range(0, self.terrain_mapper.grid_size, 5):  # Downsample for performance
            for j in range(0, self.terrain_mapper.grid_size, 5):
                if not np.isnan(self.terrain_mapper.height_map[i, j]):
                    point = Point()
                    x, y = self.terrain_mapper.grid_to_world(i, j)
                    point.x = x
                    point.y = y
                    point.z = self.terrain_mapper.height_map[i, j]

                    # Color based on traversability
                    if self.terrain_mapper.traversability_map[i, j] > 0.7:
                        marker.color.r = 0.0  # Green for traversable
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                    else:
                        marker.color.r = 1.0  # Red for non-traversable
                        marker.color.g = 0.0
                        marker.color.b = 0.0

                    marker.color.a = 0.5
                    marker.points.append(point)

        marker_array.markers.append(marker)
        self.terrain_map_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    terrain_node = TerrainAdaptationNode()

    try:
        rclpy.spin(terrain_node)
    except KeyboardInterrupt:
        pass
    finally:
        terrain_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Evaluation and Optimization

### Terrain Adaptation Metrics

Evaluating the effectiveness of terrain adaptation systems:

```python
class TerrainAdaptationEvaluator:
    def __init__(self):
        self.metrics = {
            'path_efficiency': [],
            'energy_consumption': [],
            'stability_margin': [],
            'step_success_rate': [],
            'adaptation_response_time': []
        }

    def evaluate_path_efficiency(self, planned_path, executed_path, goal_position):
        """Evaluate path efficiency"""
        if len(executed_path) == 0:
            return 0.0

        # Calculate path length efficiency
        planned_length = sum(np.linalg.norm(np.array(executed_path[i+1]) - np.array(executed_path[i]))
                          for i in range(len(executed_path)-1))

        direct_distance = np.linalg.norm(np.array(executed_path[-1][:2]) - goal_position)

        efficiency = direct_distance / planned_length if planned_length > 0 else 0.0
        return efficiency

    def evaluate_energy_consumption(self, joint_trajectories, joint_velocities):
        """Evaluate energy consumption"""
        # Simplified energy calculation based on joint velocities
        energy = 0.0
        for vel_traj in joint_velocities:
            # Energy proportional to square of velocity
            energy += np.sum(vel_traj**2)

        return energy

    def evaluate_stability_margin(self, com_trajectory, foot_positions):
        """Evaluate stability based on ZMP and support polygon"""
        stability_scores = []

        for i, com_pos in enumerate(com_trajectory):
            # Calculate distance from CoM to nearest foot
            if i < len(foot_positions):
                foot_pos = foot_positions[i]
                distance = np.linalg.norm(com_pos[:2] - foot_pos[:2])
                # Stability is better when CoM is closer to support points
                stability_score = 1.0 / (1.0 + distance)  # Higher is better
                stability_scores.append(stability_score)

        return np.mean(stability_scores) if stability_scores else 0.0

    def evaluate_step_success_rate(self, planned_footsteps, actual_footsteps):
        """Evaluate percentage of successful steps"""
        if len(planned_footsteps) == 0:
            return 0.0

        successful_steps = 0
        for planned, actual in zip(planned_footsteps, actual_footsteps):
            # Check if step was completed within tolerance
            if np.linalg.norm(np.array(planned[1:3]) - np.array(actual[1:3])) < 0.05:  # 5cm tolerance
                successful_steps += 1

        return successful_steps / len(planned_footsteps)

    def evaluate_adaptation_response_time(self, terrain_change_times, adaptation_times):
        """Evaluate how quickly system adapts to terrain changes"""
        if len(terrain_change_times) == 0:
            return float('inf')

        response_times = []
        for change_time, adaptation_time in zip(terrain_change_times, adaptation_times):
            response_time = adaptation_time - change_time
            if response_time > 0:
                response_times.append(response_time)

        return np.mean(response_times) if response_times else float('inf')

    def calculate_comprehensive_score(self, weights=None):
        """Calculate weighted comprehensive score"""
        if weights is None:
            weights = {
                'path_efficiency': 0.25,
                'energy_efficiency': 0.2,
                'stability': 0.3,
                'success_rate': 0.15,
                'response_time': 0.1
            }

        # Normalize metrics to [0, 1] range
        path_eff = np.mean(self.metrics['path_efficiency']) if self.metrics['path_efficiency'] else 0.0
        energy_eff = 1.0 / (1.0 + np.mean(self.metrics['energy_consumption'])) if self.metrics['energy_consumption'] else 1.0
        stability = np.mean(self.metrics['stability_margin']) if self.metrics['stability_margin'] else 0.0
        success_rate = np.mean(self.metrics['step_success_rate']) if self.metrics['step_success_rate'] else 0.0
        response_time_score = 1.0 / (1.0 + np.mean(self.metrics['adaptation_response_time'])) if self.metrics['adaptation_response_time'] else 1.0

        comprehensive_score = (
            weights['path_efficiency'] * path_eff +
            weights['energy_efficiency'] * energy_eff +
            weights['stability'] * stability +
            weights['success_rate'] * success_rate +
            weights['response_time'] * response_time_score
        )

        return comprehensive_score

    def log_metric(self, metric_name, value):
        """Log a metric value"""
        if metric_name in self.metrics:
            self.metrics[metric_name].append(value)

    def get_performance_report(self):
        """Generate performance report"""
        report = {
            'path_efficiency': {
                'mean': np.mean(self.metrics['path_efficiency']) if self.metrics['path_efficiency'] else 0.0,
                'std': np.std(self.metrics['path_efficiency']) if self.metrics['path_efficiency'] else 0.0,
                'count': len(self.metrics['path_efficiency'])
            },
            'energy_consumption': {
                'mean': np.mean(self.metrics['energy_consumption']) if self.metrics['energy_consumption'] else 0.0,
                'std': np.std(self.metrics['energy_consumption']) if self.metrics['energy_consumption'] else 0.0,
                'count': len(self.metrics['energy_consumption'])
            },
            'stability_margin': {
                'mean': np.mean(self.metrics['stability_margin']) if self.metrics['stability_margin'] else 0.0,
                'std': np.std(self.metrics['stability_margin']) if self.metrics['stability_margin'] else 0.0,
                'count': len(self.metrics['stability_margin'])
            },
            'step_success_rate': {
                'mean': np.mean(self.metrics['step_success_rate']) if self.metrics['step_success_rate'] else 0.0,
                'std': np.std(self.metrics['step_success_rate']) if self.metrics['step_success_rate'] else 0.0,
                'count': len(self.metrics['step_success_rate'])
            },
            'adaptation_response_time': {
                'mean': np.mean(self.metrics['adaptation_response_time']) if self.metrics['adaptation_response_time'] else 0.0,
                'std': np.std(self.metrics['adaptation_response_time']) if self.metrics['adaptation_response_time'] else 0.0,
                'count': len(self.metrics['adaptation_response_time'])
            },
            'comprehensive_score': self.calculate_comprehensive_score()
        }

        return report

# Example: Performance evaluation
evaluator = TerrainAdaptationEvaluator()

# Simulate evaluation metrics
for i in range(10):
    # Simulate different terrain scenarios
    path_eff = np.random.uniform(0.6, 0.95)
    energy_cons = np.random.uniform(10, 50)
    stability = np.random.uniform(0.5, 0.9)
    success_rate = np.random.uniform(0.8, 1.0)
    response_time = np.random.uniform(0.1, 0.5)

    evaluator.log_metric('path_efficiency', path_eff)
    evaluator.log_metric('energy_consumption', energy_cons)
    evaluator.log_metric('stability_margin', stability)
    evaluator.log_metric('step_success_rate', success_rate)
    evaluator.log_metric('adaptation_response_time', response_time)

# Generate report
report = evaluator.get_performance_report()
print("Terrain Adaptation Performance Report:")
print(f"Path Efficiency: {report['path_efficiency']['mean']:.3f} ± {report['path_efficiency']['std']:.3f}")
print(f"Energy Consumption: {report['energy_consumption']['mean']:.1f} ± {report['energy_consumption']['std']:.1f}")
print(f"Stability Margin: {report['stability_margin']['mean']:.3f} ± {report['stability_margin']['std']:.3f}")
print(f"Step Success Rate: {report['step_success_rate']['mean']:.3f} ± {report['step_success_rate']['std']:.3f}")
print(f"Response Time: {report['adaptation_response_time']['mean']:.3f} ± {report['adaptation_response_time']['std']:.3f}")
print(f"Comprehensive Score: {report['comprehensive_score']:.3f}")
```

## Conclusion

Terrain adaptation and footstep planning are critical capabilities that enable humanoid robots to navigate complex, real-world environments. The integration of perception, planning, and control systems allows robots to respond to diverse terrain conditions including flat surfaces, slopes, stairs, obstacles, and rough terrain.

The key to successful terrain adaptation lies in the combination of:
- Accurate terrain perception and classification
- Robust footstep planning algorithms
- Real-time adaptation mechanisms
- Integration with balance and locomotion control systems

Modern approaches use a hierarchical strategy where global planning provides a general path, local planning handles immediate obstacles, and reactive control adapts to unexpected terrain features. The evaluation of these systems requires comprehensive metrics that consider efficiency, stability, energy consumption, and adaptability.

The next section will explore complex terrain navigation, including specialized techniques for challenging environments like stairs, narrow passages, and dynamic obstacles.