#include <algorithm>
#include <cmath>

// applications
#include "applications/applications_param.hpp"
#include "applications/command_interfaces/map_control2actuators.hpp"
#include "applications/common_utils.hpp"
// messages
#include "car_msgs/msg/actuator_commands.hpp"
#include "car_msgs/msg/car_control.hpp"
// ROS2
#include "core/tasks_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "support/lookup_table.hpp"

class ActuationInterfaceNode : public sert::core::TaskInterface
{
  public:
    ActuationInterfaceNode(const std::string& name, rclcpp::NodeOptions options)
        : TaskInterface(
              name,
              options.append_parameter_override(
                  "cycle_time_ms", T_ACTUATIONINTERFACE_PUB))
    {
        RegisterPublisher<car_msgs::msg::ActuatorCommands>(
            "actuator_cmd", QS_ACTUATIONINTERFACE_PUB);

        RegisterSubscriber<car_msgs::msg::CarControl>(
            "car_cmd",
            [this](car_msgs::msg::CarControl::UniquePtr msg)
            {
                ctrl_msg_ = *msg;
                ctrl_received_ = true;
            });
    }

  protected:
    void ExecuteStep() override
    {
        if (!ctrl_received_)
        {
            return;
        }

        act_cmd_msg_ = mapControlToActuators(ctrl_msg_);
        publishActuators();
    }

  private:
    car_msgs::msg::CarControl ctrl_msg_;
    car_msgs::msg::ActuatorCommands act_cmd_msg_;

    bool ctrl_received_ = false;

    void publishActuators()
    {
        GetPublisher<car_msgs::msg::ActuatorCommands>("actuator_cmd")
            ->publish(act_cmd_msg_);
    }
};

using ActuationInterfaceNodeConfig = sert::support::LookupTable<
    sert::support::TableItem<ActuationInterfaceNode, sert::support::UnusedValue>>;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto task_manager = sert::core::BuildTasksManager<ActuationInterfaceNodeConfig>();
    task_manager->Execute();
    rclcpp::shutdown();
    return 0;
}
