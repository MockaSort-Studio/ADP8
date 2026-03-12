"""
Python node — counterpart to cpp_node.

Subscribes on channel_a (where cpp_node publishes),
publishes replies on channel_b (where cpp_node subscribes).

Run alongside cpp_node:
    bazel run //core/examples/simple_chatter_pybinds:cpp_node
    bazel run //core/examples/simple_chatter_pybinds:py_node
"""

import signal
import time

import channel_message_py as cm
import chatter_bridge_py as bridge_lib

_FREQ_HZ: float = 2.0
_PERIOD: float = 1.0 / _FREQ_HZ


def main() -> None:
    bridge = bridge_lib.PyDDSBridge("py_node")

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
        samples = bridge.get_channel_a()

        if not samples:
            print("py_node/execute step — no message")
        else:
            for sample in samples:
                print(f"[Py  <- channel_a] {sample.content} #{sample.counter}")

                reply = cm.ChannelMessage()
                reply.content = f"Python ack #{sample.counter}"
                reply.counter = counter
                counter += 1

                bridge.push_channel_b(reply)
                print(f"[Py  -> channel_b] {reply.content} #{reply.counter}")

        bridge.flush_outputs()
        time.sleep(_PERIOD)


if __name__ == "__main__":
    main()
