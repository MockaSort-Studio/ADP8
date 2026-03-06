"""
Python node — counterpart to cpp_node.

Subscribes on channel_a (where cpp_node publishes),
publishes replies on channel_b (where cpp_node subscribes).

Run alongside cpp_node:
    bazel run //simulation/py_dds_talker/test:cpp_node
    bazel run //simulation/py_dds_talker/test:py_node
"""

import os
import signal
import time

# Must be set before any FastDDS participant is created.
# BUILD_WORKSPACE_DIRECTORY is provided by bazel run.
if _ws := os.environ.get("BUILD_WORKSPACE_DIRECTORY"):
    os.environ.setdefault(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        os.path.join(_ws, "simulation/py_dds_talker/test/fastdds_sim.xml"),
    )

print("=" * 10)
print("check FASTRTPS_DEFAULT_PROFILES_FILE env")
print(os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "NOT SET"))
print("+" * 10)

import channel_message_py as cm  # noqa: E402
import py_dds_bridge  # noqa: E402

_FREQ_HZ: float = 10.0
_PERIOD: float = 1.0 / _FREQ_HZ


def main() -> None:
    bridge = py_dds_bridge.PyDDSBridge("py_node")
    bridge.init()

    sub = cm.Subscriber()
    pub = cm.Publisher()
    bridge.register_input("channel_a", sub)
    bridge.register_output("channel_b", pub)

    counter: int = 0
    running: bool = True

    def _stop(*_: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print("py_node started")
    while running:
        samples = bridge.get_inputs("channel_a")

        if not samples:
            print(f"py_node/execute step — no message (matched={sub.is_matched()})")
        else:
            for sample in samples:
                print(f"[Py  <- channel_a] {sample.content} #{sample.counter}")

                reply = cm.ChannelMessage()
                reply.content = f"Python ack #{sample.counter}"
                reply.counter = counter
                counter += 1

                bridge.push_output("channel_b", reply)
                print(f"[Py  -> channel_b] {reply.content} #{reply.counter}")

        time.sleep(_PERIOD)


if __name__ == "__main__":
    main()
