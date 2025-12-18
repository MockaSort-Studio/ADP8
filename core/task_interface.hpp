#ifndef CORE_TASK_INTERFACE
#define CORE_TASK_INTERFACE

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace sert::core {

class TaskInterface : public rclcpp::Node
{
  public:
    TaskInterface(const std::string& name, rclcpp::NodeOptions options)
        : Node(name, options)
    {
        const auto cycle_time_param_name = "cycle_time_ms";
        declare_parameter(cycle_time_param_name, 500);
        const auto cycle_time_ms =
            std::chrono::milliseconds(get_parameter(cycle_time_param_name).as_int());
        RCLCPP_INFO(get_logger(), "TaskInterface created with name: %s", name.c_str());
        execution_timer_ = create_wall_timer(
            cycle_time_ms,
            [this]() -> void
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Executing TaskInterface step - I am a change that triggers my CI "
                    "test!");
                ExecuteStep();
            });
    }

    template <typename MessageType>
    void RegisterPublisher(const std::string& topic_id, const int queue_size)
    {
        auto publisher = create_publisher<MessageType>(topic_id, queue_size);
        publishers_.insert_or_assign(topic_id, std::move(publisher));
        RCLCPP_INFO(get_logger(), "Publisher created for topic: %s", topic_id.c_str());
    }

    template <typename MessageType>
    void RegisterSubscriber(
        const std::string topic_id,
        std::function<void(typename MessageType::UniquePtr)> callback,
        const uint8_t QoS = 10)
    {
        auto subscription = create_subscription<MessageType>(topic_id, QoS, callback);
        subscribers_.push_back(std::move(subscription));
        RCLCPP_INFO(get_logger(), "Subscriber created for topic: %s", topic_id.c_str());
    }

    template <typename MessageType>
    typename rclcpp::Publisher<MessageType>::SharedPtr GetPublisher(
        const std::string& topic_id)
    {
        auto it = publishers_.find(topic_id);
        if (it != publishers_.end())
        {
            return std::dynamic_pointer_cast<rclcpp::Publisher<MessageType>>(it->second);
        }
        RCLCPP_ERROR(get_logger(), "Publisher for %s not found", topic_id.c_str());
        return nullptr;
    }

  protected:
    virtual void ExecuteStep() {}

  private:
    rclcpp::TimerBase::SharedPtr execution_timer_;
    std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
};
}  // namespace sert::core
#endif  // CORE_TASK_INTERFACE
