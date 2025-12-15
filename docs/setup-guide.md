---
sidebar_label: 'ROS 2 Setup Guide'
sidebar_position: 8
---

# ROS 2 Setup Guide: Getting Started with Your Robotic Nervous System

## Overview

This guide will walk you through setting up ROS 2 (Robot Operating System 2) on your development machine. ROS 2 is the foundation of the robotic nervous system we'll be exploring throughout this textbook, so getting it properly installed is the first step in your journey.

## System Requirements

Before installing ROS 2, ensure your system meets the following requirements:

### Recommended System
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish) - This is the recommended OS for ROS 2 Humble Hawksbill
- **RAM**: 8 GB or more
- **Storage**: At least 20 GB of free disk space
- **Processor**: Multi-core processor (2+ cores recommended)

### Alternative Systems
While Ubuntu 22.04 is recommended, ROS 2 can also run on:
- Windows 10/11 with WSL2 (Windows Subsystem for Linux)
- macOS (with Docker or native installation)
- Other Linux distributions (with varying levels of support)

## Installing ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is the LTS (Long Term Support) version we'll be using throughout this textbook. It provides stability and extensive documentation for educational purposes.

### Step 1: Set Locale

Ensure your locale is set to UTF-8:

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Add ROS 2 Repository

First, add the ROS 2 GPG key and repository:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Packages

Update your package list and install the ROS 2 desktop package:

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

This installs the complete ROS 2 desktop environment, including all the tools and libraries you'll need for robotics development.

### Step 4: Install colcon and Other Tools

`colcon` is the build tool used in ROS 2:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

### Step 5: Source ROS 2 Environment

To use ROS 2, you need to source the setup script. Add this line to your `~/.bashrc` file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verifying Your Installation

### Check ROS 2 Installation

Verify that ROS 2 is properly installed:

```bash
ros2 --version
```

You should see output similar to: `ros2 humble <version_number>`

### Test with Demo Nodes

Run a simple demo to verify everything works:

```bash
# Terminal 1: Run a talker node
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2: Run a listener node
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

If you see messages being passed from the talker to the listener, your installation is working correctly!

## Creating Your First ROS 2 Workspace

A workspace is where you'll develop your own ROS 2 packages. Let's create one:

```bash
# Create the workspace directory
mkdir -p ~/ros2_textbook_ws/src
cd ~/ros2_textbook_ws

# Build the workspace (even though it's empty, this sets up the basic structure)
colcon build

# Source the workspace
source install/setup.bash
```

## Setting Up Python Development Environment

Since we'll be using Python extensively with rclpy (ROS 2 Python client library), ensure Python 3 is properly configured:

```bash
python3 --version  # Should be 3.8 or higher
pip3 --version     # Should be installed
```

Install additional Python packages that are commonly used with ROS 2:

```bash
pip3 install numpy matplotlib
```

## IDE Setup (Recommended)

For the best development experience, consider setting up one of these IDEs:

### VS Code with ROS Extension
1. Install VS Code from https://code.visualstudio.com/
2. Install the "ROS" extension by Microsoft
3. This provides syntax highlighting, debugging, and ROS-specific tools

### Using the Terminal
Many ROS 2 developers prefer working directly in the terminal. This is perfectly valid and often more efficient for ROS 2 development.

## Troubleshooting Common Issues

### Issue: "command 'ros2' not found"
**Solution**: Make sure you've sourced the ROS 2 setup script:
```bash
source /opt/ros/humble/setup.bash
```

### Issue: Permission denied errors
**Solution**: Don't use `sudo` with ROS 2 commands. If you encounter permission issues, check file permissions or run the problematic command without `sudo`.

### Issue: Python import errors
**Solution**: Ensure you're using Python 3 and that the ROS 2 Python packages are installed:
```bash
python3 -c "import rclpy; print('rclpy imported successfully')"
```

## Next Steps

Now that you have ROS 2 installed, you're ready to:

1. Explore the [Module 1: The Robotic Nervous System](./module-1-ros2/index.md) content
2. Try the code examples we've provided
3. Experiment with creating your own simple ROS 2 nodes

## Additional Resources

- [Official ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Conceptual Overview](https://docs.ros.org/en/humble/Concepts.html)

## Quick Reference

Here are the essential commands you'll use frequently:

```bash
# Source ROS 2 environment (do this in each new terminal)
source /opt/ros/humble/setup.bash

# Source your workspace (do this in each new terminal after building)
source ~/ros2_textbook_ws/install/setup.bash

# List available ROS 2 commands
ros2 --help

# List active nodes
ros2 node list

# List active topics
ros2 topic list
```

## Summary

With ROS 2 properly installed, you now have the foundation for building robotic systems. The ROS 2 environment provides the "nervous system" infrastructure that allows different components of your robot to communicate and coordinate effectively.

In the next sections, you'll learn how to create your own nodes, topics, services, and actions - the building blocks of any robotic system!