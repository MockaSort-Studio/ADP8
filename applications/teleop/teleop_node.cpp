//
#include <cerrno>
#include <cstring>
#include <unordered_set>

#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

#include "applications/applications_param.hpp"
#include "applications/common_utils.hpp"
#include "car_msgs/msg/car_control.hpp"
#include "core/tasks_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "support/lookup_table.hpp"

class TeleOpNode : public sert::core::TaskInterface
{
  public:
    TeleOpNode(const std::string& name, rclcpp::NodeOptions options)
        : TaskInterface(
              name, options.append_parameter_override("cycle_time_ms", T_TELEOP_PUB))
    {
        RegisterPublisher<car_msgs::msg::CarControl>("car_cmd", QS_CARCONTROL_PUB);

        // CHANGE THIS to your keyboard device
        input_fd_ = open("/dev/input/event6", O_RDONLY | O_NONBLOCK);

        if (input_fd_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open /dev/input/event6: %s",
                strerror(errno));
            throw std::runtime_error("Failed to open keyboard device");
        }

        RCLCPP_INFO(this->get_logger(), "Teleop (evdev) started");
    }

    ~TeleOpNode()
    {
        if (input_fd_ >= 0)
            close(input_fd_);
    }

  protected:
    void ExecuteStep() override
    {
        readKeyboardEvents();
        publishCommand();
    }

  private:
    void readKeyboardEvents()
    {
        input_event ev;
        while (read(input_fd_, &ev, sizeof(ev)) == sizeof(ev))
        {
            if (ev.type != EV_KEY)
                continue;

            bool pressed = (ev.value == 1);
            bool released = (ev.value == 0);

            switch (ev.code)
            {
                case KEY_W:
                    if (pressed)
                        cmd_.v = V_MAX;
                    if (released)
                        cmd_.v = 0.0f;
                    break;

                case KEY_S:
                    if (pressed)
                        cmd_.v = -V_MAX;
                    if (released)
                        cmd_.v = 0.0f;
                    break;

                case KEY_A:
                    if (pressed)
                        cmd_.d = D_MAX;
                    if (released)
                        cmd_.d = 0.0f;
                    break;

                case KEY_D:
                    if (pressed)
                        cmd_.d = -D_MAX;
                    if (released)
                        cmd_.d = 0.0f;
                    break;

                case KEY_SPACE:
                    if (pressed)
                    {
                        cmd_.v = 0.0f;
                        cmd_.d = 0.0f;
                    }
                    break;

                case KEY_Q:
                    if (pressed)
                    {
                        RCLCPP_INFO(this->get_logger(), "Teleop exiting.");
                        rclcpp::shutdown();
                    }
                    break;

                default:
                    break;
            }
        }
    }

    void publishCommand()
    {
        GetPublisher<car_msgs::msg::CarControl>("car_cmd")->publish(cmd_);
    }
    int input_fd_ {-1};
    car_msgs::msg::CarControl cmd_;
};

using TeleOpNodeConfig = sert::support::LookupTable<
    sert::support::TableItem<TeleOpNode, sert::support::UnusedValue>>;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto task_manager = sert::core::BuildTasksManager<TeleOpNodeConfig>();
    task_manager->Execute();
    rclcpp::shutdown();
    return 0;
}
