# Quickstart Guide: Physical AI Textbook Module 1 - The Robotic Nervous System (ROS 2)

## Prerequisites

Before starting this module, you should have:
- Basic Python programming knowledge (Python 3.8+)
- Understanding of fundamental programming concepts (variables, functions, classes)
- Access to a computer capable of running ROS 2 (Ubuntu 22.04 recommended)

## Environment Setup

### 1. Install ROS 2 Humble Hawksbill

Follow the official installation guide for your operating system:
- [Ubuntu Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Windows Installation Guide](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)
- [macOS Installation Guide](https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html)

### 2. Verify Installation

Open a terminal and run:
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

You should see the ROS 2 version information.

### 3. Create a Workspace

```bash
mkdir -p ~/ros2_textbook_ws/src
cd ~/ros2_textbook_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Running the Examples

### 1. Simple Publisher/Subscriber Example

1. Create the publisher node:
```bash
cd ~/ros2_textbook_ws/src
ros2 pkg create --build-type ament_python simple_publisher_subscriber --dependencies rclpy std_msgs
```

2. Replace the content of `~/ros2_textbook_ws/src/simple_publisher_subscriber/simple_publisher_subscriber/publisher_member_function.py` with the publisher code from the module

3. Replace the content of `~/ros2_textbook_ws/src/simple_publisher_subscriber/simple_publisher_subscriber/subscriber_member_function.py` with the subscriber code

4. Build and run:
```bash
cd ~/ros2_textbook_ws
colcon build --packages-select simple_publisher_subscriber
source install/setup.bash
ros2 run simple_publisher_subscriber publisher_member_function
``

In another terminal:
```bash
cd ~/ros2_textbook_ws
source install/setup.bash
ros2 run simple_publisher_subscriber subscriber_member_function
```

### 2. Service Example

1. Create the service package:
```bash
cd ~/ros2_textbook_ws/src
ros2 pkg create --build-type ament_python simple_service --dependencies rclpy std_msgs example_interfaces
```

2. Build and run the service client and server following the same pattern as the publisher/subscriber example

## Understanding the Structure

The module content is organized as follows:

1. **Introduction**: Overview of ROS 2 as the "Robotic Nervous System"
2. **Core Components**: Detailed explanation of Nodes, Topics, Services, and Actions
3. **URDF Understanding**: Introduction to robot modeling with URDF
4. **Python Bridge**: How to use rclpy to connect Python code to ROS 2
5. **Practical Examples**: Hands-on exercises to reinforce concepts

## Key Concepts to Focus On

- **Nodes** are like "brain cells" - independent processes that perform specific functions
- **Topics** are like "nerves" - communication channels for asynchronous data streams
- **Services** are like "reflexes" - synchronous request/response communications
- **Actions** are like "complex tasks" - long-running operations with feedback

## Troubleshooting

### Common Issues

1. **Package not found**: Make sure you've sourced the ROS 2 setup file:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_textbook_ws/install/setup.bash
   ```

2. **Python import errors**: Ensure you're using the correct Python environment with ROS 2 packages installed

3. **Permission errors**: Make sure your Python files have execute permissions:
   ```bash
   chmod +x your_python_file.py
   ```

## Next Steps

After completing this module, you should be able to:
- Explain the core concepts of ROS 2 using the nervous system analogy
- Create simple ROS 2 nodes in Python
- Implement publisher/subscriber communication
- Understand the basics of URDF robot modeling
- Run and modify the provided code examples