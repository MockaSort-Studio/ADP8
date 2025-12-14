
//
#include <cmath>
#include <cstdio>

// #include <sys/select.h>
#include <termios.h>
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

        setupTerminal();

        RCLCPP_INFO(this->get_logger(), "TELEOP STARTED!");
        printHelp();
    }

    ~TeleOpNode() { restoreTerminal(); }

  protected:
    void ExecuteStep() override
    {
        if (keyAvailable())
        {
            int key = readKey();
            handleKey(key);
        }
        // implementation
        // cmd_msg_ = read_keys(something)

        // dummy implementation
        // cmd_msg_.v = 2;
        // cmd_msg_.d = 0.2;

        GetPublisher<car_msgs::msg::CarControl>("car_cmd")->publish(cmd_msg_);
    }

  private:
    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &orig_term_);
        termios raw = orig_term_;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    void restoreTerminal() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_); }

    bool keyAvailable()
    {
        timeval tv {0, 0};
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
        return FD_ISSET(STDIN_FILENO, &fds);
    }

    int readKey() { return getchar(); }

    void handleKey(int key)
    {
        switch (key)
        {
            case 'w':
                cmd_msg_.v = V_MAX;
                break;

            case 's':
                cmd_msg_.v = 0.0f;
                break;

            case 'a':
                cmd_msg_.d = D_MAX;
                break;

            case 'd':
                cmd_msg_.d = -D_MAX;
                break;

            case 'c':  // center steering
                cmd_msg_.d = 0.0f;
                break;

            case 'x':  // emergency stop
                cmd_msg_.v = 0.0f;
                cmd_msg_.d = 0.0f;
                break;

            case 'q':
                RCLCPP_INFO(this->get_logger(), "Teleop exiting.");
                rclcpp::shutdown();
                break;

            default:
                break;
        }
    }

    void printHelp()
    {
        printf("\nTeleop controls:\n");
        printf("  w/s : increase/decrease speed\n");
        printf("  a/d : steer left/right\n");
        printf("  c   : center steering\n");
        printf("  x : emergency stop\n");
        printf("  q   : quit\n\n");
    }

    // ros2 interfaces
    car_msgs::msg::CarControl cmd_msg_;
    termios orig_term_;
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
