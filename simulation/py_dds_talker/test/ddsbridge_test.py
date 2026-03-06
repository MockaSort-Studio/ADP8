import sys

import channel_message_py as cm
import chatter_bridge_py as bridge_lib
import pytest


def test_participant_name() -> None:
    name = "py_bridge_test"
    bridge = bridge_lib.PyDDSBridge(name)
    assert bridge.participant_name() == name


def test_channel_message_fields() -> None:
    msg = cm.ChannelMessage()
    msg.content = "hello from Python"
    msg.counter = 7

    assert msg.content == "hello from Python"
    assert msg.counter == 7


def test_channel_message_repr() -> None:
    msg = cm.ChannelMessage()
    msg.content = "test"
    msg.counter = 3

    assert "test" in repr(msg)
    assert "3" in repr(msg)


def test_bridge_get_inputs_no_peer() -> None:
    # No cpp_node peer running — fill_inputs should not crash and get_inputs
    # should return an empty list.
    bridge = bridge_lib.PyDDSBridge("py_bridge_no_peer_test")

    bridge.fill_inputs()
    samples = bridge.get_channel_a()

    assert isinstance(samples, list)
    assert len(samples) == 0


def test_bridge_flush_outputs_no_peer() -> None:
    # No peer — push_output + flush_outputs should not crash.
    bridge = bridge_lib.PyDDSBridge("py_bridge_flush_test")

    msg = cm.ChannelMessage()
    msg.content = "orphan"
    msg.counter = 0

    bridge.push_channel_b(msg)
    bridge.flush_outputs()  # unmatched write — should silently succeed


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
