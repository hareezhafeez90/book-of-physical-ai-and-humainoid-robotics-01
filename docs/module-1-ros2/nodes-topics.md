---
sidebar_label: 'Nodes and Topics'
sidebar_position: 4
---

# Nodes and Topics: The Brain Cells and Nerves of the Robotic Nervous System

## Learning Objectives

By the end of this section, you will be able to:
- Define what a ROS 2 Node is and how it functions as a "brain cell"
- Explain how Topics serve as "nerves" carrying asynchronous data streams
- Create and run simple publisher and subscriber nodes using Python and rclpy
- Understand the publish-subscribe communication pattern
- Implement basic message passing between nodes

## Nodes: The Brain Cells of the Robotic Nervous System

In the robotic nervous system, **Nodes** serve as the equivalent of brain cells or neurons. Just as neurons are the basic computational units of the biological nervous system, Nodes are the fundamental execution units of the ROS 2 system.

### What is a Node?

A Node is a process that performs computation. In the context of robotics, a Node typically handles a specific task or function within the larger robotic system. Examples include:

- A camera driver Node that captures images
- A sensor processing Node that interprets sensor data
- A control Node that calculates motor commands
- A perception Node that identifies objects in the environment

### Characteristics of Nodes

Like biological neurons, ROS 2 Nodes share several key characteristics:

1. **Specialized Function**: Each Node typically performs one specific function, similar to how neurons in different parts of the brain specialize in different tasks.

2. **Communication Interface**: Nodes communicate with other Nodes through well-defined interfaces (topics, services, actions).

3. **Independent Operation**: Nodes run independently and can be started, stopped, or restarted without affecting other Nodes (except for the functionality they provide).

4. **Resource Management**: Each Node manages its own resources and computational load.

### Node Implementation in Python

In Python, Nodes are implemented using the `rclpy` library, which provides the Python client interface for ROS 2. Here's a basic Node structure:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal Node has been created')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    try:
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: The Nerves of the Robotic Nervous System

**Topics** serve as the "nerves" of the robotic nervous system, carrying asynchronous data streams between Nodes. Just as biological nerves transmit signals between neurons, Topics allow Nodes to share information without requiring direct connections.

### How Topics Work

The Topic communication system follows a **publish-subscribe** pattern:

- **Publishers** send messages to a Topic
- **Subscribers** receive messages from a Topic
- Multiple Publishers can publish to the same Topic
- Multiple Subscribers can subscribe to the same Topic
- Communication is asynchronous - Publishers don't wait for responses

This is analogous to how neurons communicate: one neuron releases neurotransmitters into a synapse (publishes), and other neurons receive these signals (subscribes).

### Topic Communication Pattern

```
Publisher Node A ----> [Topic: sensor_data] ----> Subscriber Node X
Publisher Node B ----> [Topic: sensor_data] ----> Subscriber Node Y
                        (asynchronous flow)      Subscriber Node Z
```

## Implementing Publishers and Subscribers

Let's implement a simple publisher and subscriber to demonstrate the nervous system analogy in action.

### Simple Publisher: The "Sensory Neuron"

First, let's create a publisher that simulates a sensor sending data (like a sensory neuron):

```python
# File: static/code/simple_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'nerve_impulse', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Sensory Neuron (Publisher) started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Nerve impulse #{self.i} - Sensory information transmitted at {time.strftime("%H:%M:%S")}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        simple_publisher.get_logger().info('Stopping publisher...')
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Simple Subscriber: The "Motor Neuron"

Now, let's create a subscriber that receives and processes the "nerve impulses":

```python
# File: static/code/simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'nerve_impulse',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Motor Neuron (Subscriber) started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received nerve impulse: "{msg.data}"')
        # In a real robot, this might trigger a motor action
        self.process_nerve_impulse(msg.data)

    def process_nerve_impulse(self, impulse_data):
        # Simulate processing of the nerve impulse
        self.get_logger().info('Processing nerve impulse...')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        simple_subscriber.get_logger().info('Stopping subscriber...')
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## The Biological Analogy in Detail

Let's explore how the ROS 2 concepts map to biological nervous system components:

### Nodes as Neurons

| Biological Component | ROS 2 Component | Function |
|---------------------|-----------------|----------|
| Neuron Cell Body | Node | Performs computation and processing |
| Dendrites | Subscribers | Receive input signals from other neurons |
| Axon | Publisher | Sends output signals to other neurons |
| Synapse | Topic | Communication channel between neurons |

### Topics as Nerve Pathways

Just as biological nerves carry signals between different parts of the body, ROS 2 Topics carry messages between different Nodes:

- **Sensory Pathways**: Similar to how sensory neurons carry information from sensors to the brain
- **Motor Pathways**: Similar to how motor neurons carry commands from the brain to muscles
- **Interneuron Pathways**: Similar to how nodes can relay information between other nodes

## Quality of Service (QoS) in Topic Communication

In biological nervous systems, different types of signals have different requirements for reliability and timing. Similarly, ROS 2 Topics support Quality of Service (QoS) profiles that allow you to specify communication requirements:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Example: Reliable communication for critical sensor data
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,  # Ensure all messages are delivered
    durability=DurabilityPolicy.VOLATILE,    # Don't keep messages for late joiners
)
```

## Practical Exercise: Building a Neural Network

Let's create a more complex example that demonstrates multiple Nodes communicating over Topics, similar to a simple neural network:

```
[Sensor Node] -----> [Processing Node] -----> [Actuator Node]
     |                    |                       |
     v                    v                       v
[Data Stream]      [Processed Data]        [Control Commands]
```

This demonstrates how Nodes can form chains of communication, just like neurons in a biological nervous system.

## Best Practices for Node and Topic Design

When designing your robotic nervous system:

1. **Single Responsibility**: Each Node should have a clear, single purpose
2. **Appropriate Topic Names**: Use descriptive, consistent naming conventions
3. **Message Types**: Choose appropriate message types for your data
4. **Error Handling**: Implement proper error handling and logging
5. **Resource Management**: Properly clean up resources when Nodes are destroyed

## Summary

Nodes and Topics form the fundamental communication infrastructure of the robotic nervous system. Nodes act as the "brain cells" performing specific computations, while Topics serve as the "nerves" carrying asynchronous information between them. This publish-subscribe pattern enables flexible, decoupled communication that scales well for complex robotic systems.

The biological analogy helps us understand how these components work together: just as neurons communicate through synapses to coordinate biological functions, ROS 2 Nodes communicate through Topics to coordinate robotic behaviors.

## Key Terms

- **Node**: An execution unit performing specific functions (the "brain cell")
- **Topic**: A communication channel for asynchronous data streams (the "nerve")
- **Publisher**: A Node that sends messages to a Topic
- **Subscriber**: A Node that receives messages from a Topic
- **Publish-Subscribe Pattern**: Asynchronous communication where publishers don't wait for responses
- **Quality of Service (QoS)**: Settings that control communication behavior

## Review Questions

1. How does the publish-subscribe pattern in ROS 2 compare to neural communication in biological systems?
2. What are the advantages of asynchronous communication in robotic systems?
3. Why is it important for Nodes to have single responsibility?