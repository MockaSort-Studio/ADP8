
#include "car_msgs/msg/car_state.hpp"

#include <cmath>

#include "applications/common_utils.hpp"
#include "car_msgs/msg/car_control.hpp"
#include "core/tasks_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simulation/sim_param.hpp"
#include "simulation/state_update.hpp"
#include "support/lookup_table.hpp"

class CarStateNode : public sert::core::TaskInterface
{
  public:
    CarStateNode(const std::string& name, rclcpp::NodeOptions options, int cy)
        : TaskInterface(name, options)
    {
        RegisterPublisher<car_msgs::msg::CarState>("car_state", T_CARSTATE_PUB);
        RegisterSubscriber<car_msgs::msg::CarControl>(
            "car_control",
            [this](car_msgs::msg::CarControl::UniquePtr msg)
            {
                cmd_msg_.v = msg->v;
                cmd_msg_.d = msg->d;
            });
    }

  protected:
    void ExecuteStep() override
    {
        state_msg_ = state_update(state_msg_, cmd_msg_);
        auto publisher = GetPublisher<decltype(state_msg_)>("car_state");
        publisher->publish(state_msg_);
    }

  private:
    car_msgs::msg::CarState state_msg_;
    car_msgs::msg::CarControl cmd_msg_;
};

using CarStateConfig = sert::support::LookupTable<
    sert::support::TableItem<CarStateNode, sert::support::UnusedValue>>;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto task_manager = sert::core::BuildTasksManager<CarStateConfig>();
    task_manager->Execute();
    rclcpp::shutdown();
    return 0;
}
