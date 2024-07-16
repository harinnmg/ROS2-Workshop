# ROS2-Workshop-Saintgits

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



