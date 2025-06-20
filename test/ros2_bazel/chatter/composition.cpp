// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/task_interface.hpp"
#include "std_msgs/msg/string.hpp"

class MockaPublisher : public sert::core::TaskInterface

{
  public:
    MockaPublisher(const std::string& name, rclcpp::NodeOptions options)
        : TaskInterface(name, options)
    {
        RegisterPublisher<std_msgs::msg::String>("output_topic", 10);
    }

  protected:
    void ExecuteStep() override
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello from MockaPublisher!";
        RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
        auto publisher = GetPublisher<decltype(message)>();
        publisher->publish(message);
    }
};

class MockaSubscriber : public sert::core::TaskInterface
{
  public:
    MockaSubscriber(const std::string& name, rclcpp::NodeOptions options)
        : TaskInterface(name, options)
    {
        RegisterSubscriber<std_msgs::msg::String>(
            "output_topic",
            [this](std_msgs::msg::String::UniquePtr msg)
            { RCLCPP_INFO(get_logger(), "Received: '%s'", msg->data.c_str()); });
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto publisher_node = std::make_shared<MockaPublisher>("porcodeddiooo", options);
    auto subscriber_node = std::make_shared<MockaSubscriber>("diooodeeppoorco", options);
    exec.add_node(publisher_node);
    exec.add_node(subscriber_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}