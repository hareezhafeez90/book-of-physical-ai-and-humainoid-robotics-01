---
sidebar_label: 'Practical Examples'
sidebar_position: 9
---

# Practical Examples: Hands-On Exercises for the Robotic Nervous System

## Learning Objectives

By completing these exercises, you will be able to:
- Implement complete ROS 2 systems with multiple interconnected nodes
- Apply the "Robotic Nervous System" concepts in practical scenarios
- Debug and troubleshoot common ROS 2 communication issues
- Create functional robot behaviors using the publish-subscribe and service patterns
- Integrate all concepts learned in this module into working systems

## Exercise 1: Building a Simple Robot Nervous System

### Overview
In this exercise, you'll create a complete "nervous system" for a simple robot that includes sensory input, processing, and motor output - all connected through ROS 2 communication patterns.

### Requirements
- Create a sensor node that publishes simulated sensor data
- Create a processing node that interprets sensor data and makes decisions
- Create an actuator node that executes commands
- Use the nervous system analogy throughout your implementation

### Step-by-Step Implementation

#### Step 1: Create the Sensor Node
Create a file `static/code/robot_sensor.py` that simulates a distance sensor:

```python
#!/usr/bin/env python3
# robot_sensor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class RobotSensor(Node):
    def __init__(self):
        super().__init__('robot_sensor')
        self.publisher_ = self.create_publisher(Float32, 'distance_sensor', 10)
        self.timer = self.create_timer(0.5, self.publish_sensor_data)
        self.get_logger().info('Distance Sensor Node (Sensory Neuron) started')

    def publish_sensor_data(self):
        msg = Float32()
        # Simulate distance measurement (in meters)
        msg.data = random.uniform(0.1, 3.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sensor reading: {msg.data:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    sensor_node = RobotSensor()

    try:
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        sensor_node.get_logger().info('Shutting down sensor node')
    finally:
        sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 2: Create the Processing Node
Create a file `static/code/robot_brain.py` that processes sensor data and makes decisions:

```python
#!/usr/bin/env python3
# robot_brain.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class RobotBrain(Node):
    def __init__(self):
        super().__init__('robot_brain')
        self.subscription = self.create_subscription(
            Float32,
            'distance_sensor',
            self.sensor_callback,
            10)

        self.motor_publisher = self.create_publisher(String, 'motor_commands', 10)
        self.get_logger().info('Robot Brain Node (Processing Center) started')

    def sensor_callback(self, msg):
        distance = msg.data
        self.get_logger().info(f'Processing distance: {distance:.2f}m')

        # Simple AI decision making
        if distance < 0.5:
            command = "STOP"  # Too close to obstacle
        elif distance < 1.0:
            command = "SLOW_DOWN"  # Approaching obstacle
        else:
            command = "MOVE_FORWARD"  # Clear path ahead

        # Publish motor command
        cmd_msg = String()
        cmd_msg.data = command
        self.motor_publisher.publish(cmd_msg)
        self.get_logger().info(f'Brain decision: {command}')

def main(args=None):
    rclpy.init(args=args)
    brain_node = RobotBrain()

    try:
        rclpy.spin(brain_node)
    except KeyboardInterrupt:
        brain_node.get_logger().info('Shutting down brain node')
    finally:
        brain_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 3: Create the Actuator Node
Create a file `static/code/robot_motors.py` that executes commands:

```python
#!/usr/bin/env python3
# robot_motors.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotMotors(Node):
    def __init__(self):
        super().__init__('robot_motors')
        self.subscription = self.create_subscription(
            String,
            'motor_commands',
            self.command_callback,
            10)
        self.get_logger().info('Robot Motors Node (Motor Neurons) started')

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Executing motor command: {command}')

        # Simulate motor execution
        if command == "MOVE_FORWARD":
            self.move_forward()
        elif command == "SLOW_DOWN":
            self.slow_down()
        elif command == "STOP":
            self.stop_motors()
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def move_forward(self):
        self.get_logger().info('Motors: Moving forward at full speed')

    def slow_down(self):
        self.get_logger().info('Motors: Reducing speed')

    def stop_motors(self):
        self.get_logger().info('Motors: Emergency stop')

def main(args=None):
    rclpy.init(args=args)
    motor_node = RobotMotors()

    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        motor_node.get_logger().info('Shutting down motor node')
    finally:
        motor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 4: Run the Complete System
Open three terminals and run each node:

**Terminal 1:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash  # if you have a workspace
python3 static/code/robot_sensor.py
```

**Terminal 2:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash  # if you have a workspace
python3 static/code/robot_brain.py
```

**Terminal 3:**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash  # if you have a workspace
python3 static/code/robot_motors.py
```

### Biological Analogy Connection
This system mimics a simple biological reflex arc:
- **Sensor Node** = Sensory neurons detecting stimuli
- **Processing Node** = Interneurons processing information and making decisions
- **Actuator Node** = Motor neurons executing responses
- **Topics** = Synapses carrying information between neurons

## Exercise 2: Implementing a Service-Based Safety System

### Overview
Create a safety service that can be called to check if robot operations are safe to proceed.

### Implementation

#### Step 1: Create Safety Service Server
Create `static/code/safety_service.py`:

```python
#!/usr/bin/env python3
# safety_service.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class SafetyService(Node):
    def __init__(self):
        super().__init__('safety_service')
        self.srv = self.create_service(Trigger, 'safety_check', self.safety_callback)
        self.get_logger().info('Safety Service (Reflex Safety System) started')

    def safety_callback(self, request, response):
        # Simulate safety check (in real system, this would check actual safety parameters)
        import random
        is_safe = random.choice([True, True, True, True, False])  # 80% safe, 20% unsafe

        if is_safe:
            response.success = True
            response.message = "Safety check passed - operations approved"
            self.get_logger().info('Safety check: PASSED')
        else:
            response.success = False
            response.message = "Safety check failed - operations blocked"
            self.get_logger().warn('Safety check: FAILED - STOPPING ALL OPERATIONS')

        return response

def main(args=None):
    rclpy.init(args=args)
    safety_service = SafetyService()

    try:
        rclpy.spin(safety_service)
    except KeyboardInterrupt:
        safety_service.get_logger().info('Shutting down safety service')
    finally:
        safety_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 2: Create Safety Service Client
Create `static/code/safety_client.py`:

```python
#!/usr/bin/env python3
# safety_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class SafetyClient(Node):
    def __init__(self):
        super().__init__('safety_client')
        self.cli = self.create_client(Trigger, 'safety_check')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for safety service...')

        # Check safety before performing operations
        self.check_safety_and_operate()

    def check_safety_and_operate(self):
        request = Trigger.Request()

        # Call safety service
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response.success:
            self.get_logger().info(f'PROCEEDING: {response.message}')
            self.perform_operation()
        else:
            self.get_logger().error(f'STOPPED: {response.message}')

    def perform_operation(self):
        self.get_logger().info('Performing robot operation - safety confirmed')
        # In a real system, this would be the actual robot operation

def main(args=None):
    rclpy.init(args=args)
    safety_client = SafetyClient()

    try:
        # Keep node alive to see logs
        rclpy.spin_once(safety_client, timeout_sec=1)
    except KeyboardInterrupt:
        safety_client.get_logger().info('Shutting down safety client')
    finally:
        safety_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 3: URDF Visualization and Analysis

### Overview
Analyze and visualize the provided URDF model to understand robot structure.

### Steps
1. Use RViz2 to visualize the URDF model:
```bash
# Install required tools if not already installed
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui

# Visualize the URDF
ros2 run rviz2 rviz2
# In RViz2, add a RobotModel display and set the robot description to the URDF file
```

2. Examine the `static/code/urdf-example.urdf` file and identify:
   - The root link of the robot
   - The number of joints and their types
   - Which joints are for legs vs. arms
   - The kinematic chains for each limb

3. Modify the URDF to add a simple sensor (like a camera) to the robot's head.

## Exercise 4: Creating Your Own Communication Pattern

### Overview
Design and implement a new communication pattern that combines multiple ROS 2 concepts.

### Requirements
1. Create a node that acts as both publisher and subscriber
2. Use services for configuration or state queries
3. Implement a simple action for a long-running task
4. Follow the nervous system analogy in your design

### Suggested Project: Environmental Monitoring System
- **Sensor Node**: Publishes environmental data (temperature, humidity)
- **Analysis Node**: Subscribes to sensor data and analyzes trends
- **Control Node**: Uses services to configure monitoring parameters
- **Action Server**: Performs long-term data collection and analysis

## Troubleshooting Common Issues

### Issue 1: Nodes Can't Communicate
**Symptoms**: Nodes don't see each other's topics/services
**Solutions**:
- Ensure all terminals have sourced the ROS 2 setup: `source /opt/ros/humble/setup.bash`
- Check that nodes are using the same ROS domain ID
- Verify topic names match exactly

### Issue 2: Python Import Errors
**Symptoms**: `ModuleNotFoundError` for rclpy or ROS messages
**Solutions**:
- Ensure ROS 2 is properly installed and sourced
- Check that you're using Python 3
- Verify that required message packages are installed

### Issue 3: Permission Issues
**Symptoms**: Can't run ROS commands or access nodes
**Solutions**:
- Don't use `sudo` with ROS commands
- Check file permissions on your workspace
- Ensure your user is properly configured

## Assessment Rubric

### Basic Implementation (Pass)
- [ ] All three nodes from Exercise 1 run without errors
- [ ] Nodes successfully communicate via topics
- [ ] Code follows Python and ROS 2 best practices
- [ ] Nervous system analogy is clearly applied

### Advanced Implementation (Credit)
- [ ] Service-based safety system from Exercise 2 works correctly
- [ ] URDF analysis completed with modifications
- [ ] Error handling implemented appropriately
- [ ] Code is well-documented

### Excellent Implementation (Distinction)
- [ ] Custom communication pattern from Exercise 4 implemented
- [ ] All exercises completed with additional features
- [ ] Deep understanding of ROS 2 concepts demonstrated
- [ ] Troubleshooting skills demonstrated

## Extension Activities

1. **Advanced Exercise**: Implement a multi-robot coordination system where multiple robots communicate with each other
2. **Research Exercise**: Investigate how real humanoid robots like Pepper or NAO implement similar nervous system concepts
3. **Integration Exercise**: Connect your ROS 2 nodes to a simulation environment like Gazebo

## Summary

These practical exercises have helped you apply the theoretical concepts of the robotic nervous system to real implementations. You've created working ROS 2 systems that demonstrate:

- The publish-subscribe pattern as the "nerves" of the system
- Services as "reflexes" for immediate responses
- The overall architecture as a "nervous system" connecting perception, processing, and action
- The bridge between AI algorithms and hardware through rclpy

## Next Steps

After completing these exercises, you should be able to:
- Design and implement complete ROS 2 systems
- Debug communication issues between nodes
- Apply the nervous system analogy to new robotic problems
- Prepare for more advanced robotics topics in subsequent modules

## Resources for Further Learning

- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [rclpy Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/URDF/Building-a-Visual-Robot-Model-with-URDF.html)
- [ROS 2 Design Papers](https://arxiv.org/abs/1811.09461)