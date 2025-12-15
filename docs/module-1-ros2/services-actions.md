---
sidebar_label: 'Services and Actions'
sidebar_position: 5
---

# Services and Actions: The Reflexes and Complex Tasks of the Robotic Nervous System

## Learning Objectives

By the end of this section, you will be able to:
- Distinguish between Services and Actions in ROS 2
- Explain how Services function as "reflexes" for immediate responses
- Understand how Actions serve as "complex tasks" requiring extended execution
- Implement both Services and Actions using Python and rclpy
- Choose the appropriate communication pattern for different robotic tasks

## Services: The Reflexes of the Robotic Nervous System

In the robotic nervous system, **Services** function as "reflexes" - providing synchronous request/response communication for immediate responses. Just as biological reflexes provide quick, automatic responses to stimuli (like pulling your hand away from a hot surface), ROS 2 Services provide immediate responses to specific requests.

### Understanding Service Communication

Service communication follows a **request/response** pattern:

1. A client Node sends a request to a Service
2. The Service processes the request
3. The Service sends a response back to the client
4. The client waits for the response (synchronous)

This is fundamentally different from the asynchronous publish/subscribe pattern of Topics. With Services, the client "blocks" (waits) until it receives a response.

### Service Characteristics

Like biological reflexes, ROS 2 Services have specific characteristics:

1. **Synchronous**: The client waits for a response before continuing
2. **Immediate**: Designed for quick, single-result operations
3. **Request-Response**: One request generates one response
4. **Stateless**: Each request is independent of others

### Service Implementation in Python

Here's how to implement a simple service server:

```python
# File: static/code/service_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Mathematical Reflex Server started')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Reflex triggered: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        minimal_service.get_logger().info('Stopping service server...')
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And here's the corresponding service client:

```python
# File: static/code/service_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()
        self.get_logger().info('Reflex Client started')

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()

    try:
        response = minimal_client.send_request(2, 3)
        minimal_client.get_logger().info(f'Reflex response: 2 + 3 = {response.sum}')
    except Exception as e:
        minimal_client.get_logger().error(f'Service call failed: {e}')
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: The Complex Tasks of the Robotic Nervous System

While Services handle immediate "reflexes," **Actions** manage "complex tasks" that require extended execution time. Actions are like complex behaviors that require ongoing monitoring, feedback, and the possibility of cancellation - similar to how complex motor tasks require continuous adjustment and feedback.

### Understanding Action Communication

Actions follow a more complex pattern with three components:

1. **Goal**: The desired outcome (sent by the client)
2. **Feedback**: Ongoing status updates during execution (sent by the server)
3. **Result**: The final outcome when the task completes (sent by the server)

This is perfect for tasks like navigation ("go to location X"), manipulation ("pick up object Y"), or calibration ("calibrate sensor Z").

### Action Characteristics

Actions have specific characteristics that make them suitable for complex tasks:

1. **Long-Running**: Designed for operations that take time to complete
2. **Feedback**: Provides ongoing status updates during execution
3. **Cancellable**: Clients can cancel actions before completion
4. **Goal-Result Pattern**: Clear start (goal) and end (result) states

### Action Implementation in Python

Here's a simplified example of an action server (in practice, actions use more complex message types):

```python
# Example of action-like behavior using services with feedback
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci  # Standard action example

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Complex Task Server started')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Goal succeeded with result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()

    try:
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
        fibonacci_action_server.get_logger().info('Stopping action server...')
    finally:
        fibonacci_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## The Biological Analogy in Detail

### Services as Reflexes

| Biological Reflex | ROS 2 Service | Function |
|-------------------|---------------|----------|
| Knee-jerk reflex | Simple service call | Immediate, automatic response |
| Pupillary reflex | Status query service | Quick adjustment response |
| Withdrawal reflex | Safety service | Immediate protective response |

### Actions as Complex Tasks

| Biological Complex Task | ROS 2 Action | Function |
|------------------------|--------------|----------|
| Walking | Navigation action | Extended task with continuous feedback |
| Grasping an object | Manipulation action | Multi-step task with monitoring |
| Learning a skill | Training action | Long-term process with progress feedback |

## When to Use Services vs Actions vs Topics

Choosing the right communication pattern is crucial for effective robotic system design:

### Use Services (Reflexes) When:
- You need an immediate response to a request
- The operation is relatively quick (under a few seconds)
- You need a single result for a single request
- The operation is stateless

### Use Actions (Complex Tasks) When:
- The operation takes a long time to complete
- You need ongoing feedback during execution
- The operation might need to be canceled
- You need to track progress toward a goal

### Use Topics (Nerves) When:
- You need continuous data streaming
- Communication should be asynchronous
- Multiple subscribers need the same information
- Real-time performance is critical

## Practical Example: A Robotic Arm Control System

Let's consider how these communication patterns work together in a robotic arm system:

```
[GUI Node] --(Service)--> [Motion Planner] --(Topics)--> [Joint Controllers]
    |                          |                            |
    v                          v                            v
(Service: Move Arm)    (Action: Execute Trajectory)    (Topics: Joint States)
```

1. **Service Call**: GUI requests the arm to move to a specific position (reflex-like immediate command)
2. **Action Execution**: Motion planner executes the complex trajectory with ongoing feedback (complex task)
3. **Topic Streaming**: Joint controllers continuously publish joint state information (continuous monitoring)

## Quality of Service for Services and Actions

Just as with Topics, Services and Actions support Quality of Service (QoS) configurations:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# For services requiring guaranteed delivery
service_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE
)
```

## Best Practices for Service and Action Design

When implementing Services and Actions:

1. **Keep Services Fast**: Services should respond quickly; long operations should use Actions
2. **Provide Clear Feedback**: Actions should provide meaningful feedback during execution
3. **Handle Cancellation**: Actions should properly handle cancellation requests
4. **Error Handling**: Implement proper error responses for both Services and Actions
5. **Timeout Management**: Set appropriate timeouts for service calls and action goals

## Summary

Services and Actions provide two essential communication patterns in the robotic nervous system. Services function as "reflexes" for immediate, synchronous request/response communication, while Actions serve as "complex tasks" for long-running operations that require feedback and cancellation capabilities.

The biological analogy helps us understand when to use each pattern: use Services for quick, reflexive responses and Actions for complex behaviors that require ongoing attention and feedback.

## Key Terms

- **Service**: Synchronous request/response communication (the "reflex")
- **Action**: Long-running task with feedback and goal management (the "complex task")
- **Request/Response**: Synchronous communication pattern where client waits for response
- **Goal/Feedback/Result**: Action communication pattern with ongoing status updates
- **Client**: Node that requests service or action execution
- **Server**: Node that provides service or executes actions

## Review Questions

1. What is the main difference between Services and Actions in ROS 2?
2. Why might you choose an Action over a Service for a navigation task?
3. How do Quality of Service settings affect Service and Action communication?