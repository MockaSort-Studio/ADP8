#include <iostream>

#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

int main()
{
    const char* device = "/dev/input/event6";  // Replace with your keyboard device
    int input_fd = open(device, O_RDONLY | O_NONBLOCK);
    if (input_fd == -1)
    {
        perror("Failed to open device");
        return 1;
    }

    input_event ev;
    while (true)
    {
        ssize_t n = read(input_fd, &ev, sizeof(ev));
        if (n == sizeof(ev))
        {
            if (ev.type == EV_KEY)
            {  // Only handle key events
                std::cout << "Key code: " << ev.code << " Value: " << ev.value
                          << std::endl;

                // Example switch-case for key codes
                switch (ev.code)
                {
                    case KEY_A:
                        if (ev.value == 1)  // 1 = key press, 0 = release
                            std::cout << "A pressed!" << std::endl;
                        break;
                    case KEY_B:
                        if (ev.value == 1)
                            std::cout << "B pressed!" << std::endl;
                        break;
                        // Add more keys as needed
                }
            }
        } else
        {
            // Non-blocking: sleep briefly if no data
            usleep(1000);
        }
    }

    close(input_fd);
    return 0;
}
