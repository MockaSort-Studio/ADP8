import channel_message_py as cm
import py_dds_bridge


def test_participant():
    name = "py_bridge_test"
    bridge = py_dds_bridge.PyDDSBridge(name)
    bridge.init()

    actual = bridge.participant_name()
    assert actual == name, f"expected '{name}', got '{actual}'"
    print(f"DDS participant created: '{actual}'")


def test_channel_message():
    msg = cm.ChannelMessage()
    msg.content = "hello from Python"
    msg.counter = 7

    assert msg.content == "hello from Python"
    assert msg.counter == 7
    print(f"ChannelMessage: {msg}")


def test_pub_sub_registration():
    bridge = py_dds_bridge.PyDDSBridge("py_bridge_pubsub_test")
    bridge.init()

    sub = cm.Subscriber()
    pub = cm.Publisher()

    bridge.register_input("channel_a", sub)
    bridge.register_output("channel_b", pub)

    # No peer running — unmatched is expected, but entities must exist without error.
    assert not sub.is_matched(), "expected unmatched (no peer)"
    assert not pub.is_matched(), "expected unmatched (no peer)"
    print("Publisher and Subscriber registered — unmatched (no peer, expected).")


if __name__ == "__main__":
    test_participant()
    test_channel_message()
    test_pub_sub_registration()
