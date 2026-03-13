"""Tests for the generated pybind11 type binding (no DDS required)."""

import sys

import ping_message_py as pm
import pytest


def test_default_construction() -> None:
    msg = pm.PingMessage()
    assert msg.content == ""
    assert msg.id == 0


def test_field_assignment() -> None:
    msg = pm.PingMessage()
    msg.content = "hello"
    msg.id = 42

    assert msg.content == "hello"
    assert msg.id == 42


def test_repr_contains_fields() -> None:
    msg = pm.PingMessage()
    msg.content = "ping"
    msg.id = 7

    r = repr(msg)
    assert "ping" in r
    assert "7" in r


def test_fields_are_independent() -> None:
    a = pm.PingMessage()
    b = pm.PingMessage()
    a.content = "a"
    b.content = "b"
    a.id = 1
    b.id = 2

    assert a.content == "a"
    assert b.content == "b"
    assert a.id == 1
    assert b.id == 2


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
