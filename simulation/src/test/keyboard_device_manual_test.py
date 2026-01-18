import time
from evdev import list_devices

from simulation.src.keyboard_device import (
    KeyboardDevice,
)


def main():
    # List all available input devices
    print("Available input devices:")
    for dev_path in list_devices():
        print(dev_path)

    # Replace this with your actual keyboard device path
    device_path = "/dev/input/event6"

    try:
        kb = KeyboardDevice(device_path)
    except FileNotFoundError:
        print(f"Device {device_path} not found!")
        exit(1)

    kb.start()
    print(f"Listening on {device_path}... Press ESC to quit.")

    try:
        while True:
            pressed_keys = kb.get_cmd()
            if pressed_keys:
                print(f"Keys currently pressed: {pressed_keys}")

            if "KEY_ESC" in pressed_keys:
                print("ESC pressed. Exiting.")
                break

            time.sleep(0.05)  # small delay to avoid busy-waiting
    finally:
        kb.stop()


if __name__ == "__main__":
    main()
