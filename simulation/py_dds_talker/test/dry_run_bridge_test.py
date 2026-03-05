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


if __name__ == "__main__":
    test_participant()
    test_channel_message()
