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

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MockaTask : public rclcpp::Node
{
  public:
    MockaTask(const std::string& name, rclcpp::NodeOptions options) : Node(name, options)
    {
        declare_parameter("cycle_time_ms", 500);
        const auto cycle_time_ms = get_parameter("cycle_time_ms").as_int();
        RCLCPP_INFO(get_logger(), "MockaTask created with name: %s", name.c_str());
        execution_timer_ = create_wall_timer(
            std::chrono::milliseconds(cycle_time_ms),
            [this]() -> void
            {
                RCLCPP_INFO(get_logger(), "Executing MockaTask step");
                ExecuteStep();
            });
    }

    template <typename MockaKitOutput>
    void RegisterPublisher(const std::string topic_id, const int queue_size)
    {
        auto publisher = create_publisher<MockaKitOutput>(topic_id, queue_size);
        publishers_.insert_or_assign(typeid(MockaKitOutput), std::move(publisher));
        RCLCPP_INFO(get_logger(), "Publisher created for topic: output_topic");
    }

    template <typename MockaKitInput>
    void RegisterSubscriber(
        const std::string topic_id,
        std::function<void(typename MockaKitInput::UniquePtr)> callback,
        const uint8_t QoS = 10)
    {
        auto subscription = create_subscription<MockaKitInput>(topic_id, QoS, callback);
        subscribers_.push_back(std::move(subscription));
        RCLCPP_INFO(get_logger(), "Subscriber created for topic: %s", topic_id.c_str());
    }

    template <typename MockaKitOutput>
    typename rclcpp::Publisher<MockaKitOutput>::SharedPtr GetPublisher()
    {
        const std::type_info& type_info = typeid(MockaKitOutput);
        auto it = publishers_.find(type_info);
        if (it != publishers_.end())
        {
            return std::dynamic_pointer_cast<rclcpp::Publisher<MockaKitOutput>>(
                it->second);
        }
        RCLCPP_ERROR(get_logger(), "Publisher for type %s not found", type_info.name());
        return nullptr;
    }

  protected:
    virtual void ExecuteStep()
    {
        RCLCPP_INFO(get_logger(), "Executing step in MockaTask");
        // This method should be overridden by derived classes to implement specific
        // behavior
    }

  private:
    rclcpp::TimerBase::SharedPtr execution_timer_;
    std::map<std::type_index, rclcpp::PublisherBase::SharedPtr> publishers_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
};

class MockaPublisher : public MockaTask

{
  public:
    MockaPublisher(const std::string& name, rclcpp::NodeOptions options)
        : MockaTask(name, options)
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

class MockaSubscriber : public MockaTask
{
  public:
    MockaSubscriber(const std::string& name, rclcpp::NodeOptions options)
        : MockaTask(name, options)
    {
        RegisterSubscriber<std_msgs::msg::String>(
            "input_topic",
            [this](std_msgs::msg::String::UniquePtr msg)
            { RCLCPP_INFO(get_logger(), "Received: '%s'", msg->data.c_str()); });
    }

  protected:
    void ExecuteStep() override
    {
        RCLCPP_INFO(get_logger(), "Executing step in MockaSubscriber");
        // This method can be overridden to implement specific behavior
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