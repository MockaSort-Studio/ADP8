from evdev import InputDevice, categorize, ecodes
import threading
import os


class KeyboardDevice:
    def __init__(self, device_path="/dev/input/event6"):
        if not os.path.exists(device_path):
            raise FileNotFoundError(f"Keyboard device {device_path} does not exist")

        self.device = InputDevice(device_path)
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def _listen(self):
        for event in self.device.read_loop():
            if not self.running:
                break

            # Only handle key events
            if event.type != ecodes.EV_KEY:
                continue

            key_event = categorize(event)
            key_code = key_event.keycode

            # Sometimes keycode is a list (like ['KEY_W']), normalize to string
            if isinstance(key_code, list):
                key_code = key_code[0]

            with self.lock:
                if key_event.keystate == key_event.key_down:
                    self.pressed_keys.add(key_code)
                elif key_event.keystate == key_event.key_up:
                    self.pressed_keys.discard(key_code)

    def get_cmd(self):
        with self.lock:
            return set(self.pressed_keys)  # return a copy
