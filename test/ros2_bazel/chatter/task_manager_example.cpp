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

#include "core/tasks_manager.hpp"
#include "std_msgs/msg/string.hpp"
#include "support/lookup_table.hpp"

class MockaPublisher : public sert::core::TaskInterface

{
  public:
    MockaPublisher(const std::string& name, rclcpp::NodeOptions options)
        : TaskInterface(name, options)
    {
        RegisterPublisher<std_msgs::msg::String>("topic", 10);
    }

  protected:
    void ExecuteStep() override
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello from MockaPublisher!";
        RCLCPP_INFO(
            rclcpp::get_logger("MockaPublisher"),
            "Publishing: '%s'",
            message.data.c_str());
        auto publisher = GetPublisher<decltype(message)>("topic");
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
            "topic",
            [](std_msgs::msg::String::UniquePtr msg)
            {
                RCLCPP_INFO(
                    rclcpp::get_logger("MockaSubscriber"),
                    "Received: '%s'",
                    msg->data.c_str());
            });
    }
};

using ExampleConfig = sert::support::LookupTable<
    sert::support::TableItem<MockaPublisher, sert::support::UnusedValue>,
    sert::support::TableItem<MockaSubscriber, sert::support::UnusedValue>>;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto task_manager = sert::core::BuildTasksManager<ExampleConfig>();
    task_manager->Execute();
    rclcpp::shutdown();
    return 0;
}