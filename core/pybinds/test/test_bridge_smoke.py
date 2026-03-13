"""Smoke tests for the generated pybind11 PyDDSBridge.

No peer node running — tests cover construction, empty reads, and unmatched
writes. Everything must complete without crashing.
"""

import sys

import ping_bridge_py as bridge_lib
import ping_message_py as pm
import pytest


# DDSContextProvider is a singleton: the first bridge created sets the
# participant name for the whole process. Run participant_name first.
def test_participant_name() -> None:
    bridge = bridge_lib.PyDDSBridge("py_pybinds_test")
    assert bridge.participant_name() == "py_pybinds_test"


def test_fill_inputs_no_peer() -> None:
    bridge = bridge_lib.PyDDSBridge("py_pybinds_test")
    bridge.fill_inputs()  # must not crash


def test_get_ping_in_returns_empty_without_peer() -> None:
    bridge = bridge_lib.PyDDSBridge("py_pybinds_test")
    bridge.fill_inputs()

    samples = bridge.get_ping_in()

    assert isinstance(samples, list)
    assert len(samples) == 0


def test_push_and_flush_no_peer() -> None:
    bridge = bridge_lib.PyDDSBridge("py_pybinds_test")

    msg = pm.PingMessage()
    msg.content = "orphan"
    msg.id = 0

    bridge.push_ping_out(msg)
    bridge.flush_outputs()  # unmatched write — must not crash


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
