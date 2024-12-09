# ROS2-Workshop

The basics of ROS2 can be found here
https://ieeexplore.ieee.org/document/7743223
# ROS Installation link
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# Start
How to create a workspace
```
mkdir -p hello_ws/src
```

Creating a ros2 package, 

a) CPP Package
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
```
b) Python Package
```
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
```
Now we focus on cpp package, create nodes
```
cd src
gedit publisher.cpp
```
Now complete the code
```

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```
Open Package.xml and add the following
```
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

Open CMakeLsts.txt
```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```
Add executables
```
add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```
```
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```
create a subscriber code as per the similar process, the code is given below
```
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```
and complete the similar process
Come to workspace and type the following
```
colcon build
```
# Running a Node
```
source /opt/ros/humble/setup.bash
ros2 run package_name talker
```
# Python Package
```
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
```
Now go to the package and a same folder with package name will be there, go the folder and open a python code , publisher.py
```
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```
Add the following lines to packages.xml
```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
Then open setup.py
```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```
similar to publisher, create a subscriber node
```

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
and do the same process for displaying the subscriber code.

# Turtlesim
Now we can familarize the concept of topics, services and actions
Go to the turtlesim simulator
```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```
The excersizes can be completed as per the guidelines
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
Move the turtle
```
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
Python code for turtlesim
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.move_turtle)
        self.get_logger().info('Turtle controller node has been started.')

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = 2.0  # Move forward with a linear velocity of 2.0
        msg.angular.z = 1.0  # Rotate with an angular velocity of 1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)

    # Destroy the node explicitly
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

Now move turtle using following cpp code
```
#include <functional>
#include <memory>
#include <sstream>
#include <string>



#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtleRotate : public rclcpp::Node
{
public:
    TurtleRotate() : Node("turtle_rotate")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtleRotate::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.5; 
        message.angular.z = 0.5;  // Rotate with a constant angular velocity
        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleRotate>());
    rclcpp::shutdown();
    return 0;
}
```
You have to complete the package building procedure first and complete the experiments

## Launch file
Python launch files are a significant change in ros2
Use the following code in a tex file inside a launch folder 
```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        Node(
            package='tfexample',
            executable='move',
            name='move',
            
          
        ),
    ])
```
Now build your package and run the code.

## Calling a service using a python code
```
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class TurtlesimSpawner(Node):
    def __init__(self):
        super().__init__('turtlesim_spawner')
        self.client = self.create_client(Spawn, 'spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Spawn.Request()

    def spawn_turtle(self, x, y, theta, name):
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.req.name = name
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Spawned turtle with name: %s' % self.future.result().name)
        else:
            self.get_logger().error('Failed to spawn turtle')


def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimSpawner()
    node.spawn_turtle(2.0, 2.0, 0.0, 'new_turtle')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Challenge?? Move two turtles in a screen in circular shape


