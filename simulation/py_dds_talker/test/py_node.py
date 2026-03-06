"""
Python node — counterpart to cpp_node.

Subscribes on channel_a (where cpp_node publishes),
publishes replies on channel_b (where cpp_node subscribes).

Run alongside cpp_node:
    bazel run //simulation/py_dds_talker/test:cpp_node
    bazel run //simulation/py_dds_talker/test:py_node
"""

import signal
import time

# in the future will be something like:
#   import py_dds_lib
#   import channel_autogen_lib
import py_mock_channel as dds_lib

_FREQ_HZ: float = 2.0
_PERIOD: float = 1.0 / _FREQ_HZ


def main() -> None:
    bridge = dds_lib.PyDDSBridge("py_node")

    counter: int = 0
    running: bool = True

    def _stop(*_: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print("py_node started")
    while running:
        bridge.fill_inputs()
        samples = bridge.get_inputs()

        if not samples:
            print("py_node/execute step — no message")
        else:
            for sample in samples:
                print(f"[Py  <- channel_a] {sample.content} #{sample.counter}")

                reply = dds_lib.ChannelMessage()
                reply.content = f"Python ack #{sample.counter}"
                reply.counter = counter
                counter += 1

                bridge.push_output(reply)
                print(f"[Py  -> channel_b] {reply.content} #{reply.counter}")

        bridge.flush_outputs()
        time.sleep(_PERIOD)


if __name__ == "__main__":
    main()
