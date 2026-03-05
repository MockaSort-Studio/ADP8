import py_dds_bridge


def main():
    name = "py_bridge_test"
    bridge = py_dds_bridge.PyDDSBridge(name)
    bridge.init()

    actual = bridge.participant_name()
    assert actual == name, f"expected '{name}', got '{actual}'"
    print(f"DDS participant created: '{actual}'")


if __name__ == "__main__":
    main()
