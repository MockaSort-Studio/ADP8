import sys
import time

import channel_message_py as cm
import py_dds_bridge
import pytest


def test_participant() -> None:
    name = "py_bridge_test"
    bridge = py_dds_bridge.PyDDSBridge(name)
    bridge.init()

    actual = bridge.participant_name()
    assert actual == name, f"expected '{name}', got '{actual}'"


def test_channel_message() -> None:
    msg = cm.ChannelMessage()
    msg.content = "hello from Python"
    msg.counter = 7

    assert msg.content == "hello from Python"
    assert msg.counter == 7


def test_pub_sub_registration() -> None:
    bridge = py_dds_bridge.PyDDSBridge("py_bridge_pubsub_test")
    bridge.init()

    sub = cm.Subscriber()
    pub = cm.Publisher()

    bridge.register_input("channel_a", sub)
    bridge.register_output("channel_b", pub)

    # No peer running — unmatched is the expected state.
    assert not sub.is_matched()
    assert not pub.is_matched()


def test_intraprocess_loopback() -> None:
    """
    Publisher and Subscriber on the same topic within the same process.
    Both live in channel_message_py.so — one DDSContextProvider singleton,
    one DomainParticipant.

    Passes  → binding works end-to-end; two-participant split is not a blocker.
    Fails on is_matched() → FastDDS intraprocess delivery disabled or discovery
                            too slow; raise the sleep or revisit participant setup.
    Fails on drain()      → matching works but the data path is broken.
    """
    sub = cm.Subscriber()
    pub = cm.Publisher()

    sub.start("loopback_test")
    pub.start("loopback_test")

    # SPDP/SEDP matching is async even for intraprocess endpoints.
    time.sleep(0.5)

    assert sub.is_matched(), "intraprocess sub should match the intraprocess pub"
    assert pub.is_matched(), "intraprocess pub should match the intraprocess sub"

    msg = cm.ChannelMessage()
    msg.content = "ping"
    msg.counter = 42
    pub.publish(msg)

    time.sleep(0.1)

    samples = sub.drain()
    assert len(samples) == 1, f"expected 1 sample, got {len(samples)}"
    assert samples[0].content == "ping"
    assert samples[0].counter == 42


def test_bridge_get_inputs_push_output() -> None:
    """
    Exercises get_inputs() and push_output() end-to-end through the bridge
    using an intraprocess loopback: the bridge's output feeds its own input.
    """
    bridge = py_dds_bridge.PyDDSBridge("py_bridge_io_test")
    bridge.init()

    bridge.register_input("io_test", cm.Subscriber())
    bridge.register_output("io_test", cm.Publisher())

    time.sleep(0.5)

    msg = cm.ChannelMessage()
    msg.content = "bridge_ping"
    msg.counter = 1
    bridge.push_output("io_test", msg)

    time.sleep(0.1)

    samples = bridge.get_inputs("io_test")
    assert len(samples) == 1, f"expected 1 sample, got {len(samples)}"
    assert samples[0].content == "bridge_ping"
    assert samples[0].counter == 1


def test_bridge_key_error_on_unknown_topic() -> None:
    bridge = py_dds_bridge.PyDDSBridge("py_bridge_err_test")
    bridge.init()

    with pytest.raises(KeyError):
        bridge.get_inputs("nonexistent")

    with pytest.raises(KeyError):
        bridge.push_output("nonexistent", cm.ChannelMessage())


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
