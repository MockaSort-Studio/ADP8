import sys
from typing import Any, Dict

import pytest

from core.communication.generators.dds_gen_data_models import (
    IdlTypeHeader,
    Ports,
    TopicIdHeader,
    remove_bazel_prefix_path,
)


def test_bazel_path_stripping():
    path = "bazel-out/k8-fastbuild/bin/core/communication/my_header.hpp"
    assert remove_bazel_prefix_path(path) == "my_header.hpp"


@pytest.fixture
def raw_ports_data():
    """Represents the raw YAML-like input before normalization."""
    return {
        "publications": [{"engine_status": {"type": "EngineStatus", "queue_size": 10}}],
        "subscriptions": [{"battery_level": {"type": "Battery"}}],
    }


def test_ports_normalization(raw_ports_data: Dict[str, Any]) -> None:
    """Verify that 'engine_status' becomes 'EngineStatusTopic' and types are suffixed."""
    ports = Ports.model_validate(raw_ports_data)

    pub = ports.publications[0]
    assert pub.topic_id.name == "EngineStatusTopic"
    assert pub.topic_id.c_var_name == "kEngineStatusTopicName"
    assert pub.type == "EngineStatusPubSubType"
    assert pub.queue_size == 10

    sub = ports.subscriptions[0]
    assert sub.topic_id.name == "BatteryLevelTopic"
    assert sub.queue_size == 1


def test_idl_header_include_generation(raw_ports_data: Dict[str, Any]) -> None:
    """Verify IdlTypeHeader automatically finds PubSubTypes from ports."""
    ports = Ports.model_validate(raw_ports_data)

    header_data = {
        "output_file_path": "gen/dds_types.hpp",
        "subscriptions": [s.model_dump() for s in ports.subscriptions],
        "publications": [p.model_dump() for p in ports.publications],
    }

    header = IdlTypeHeader.model_validate(header_data)

    assert "EngineStatusPubSubTypes.hpp" in header.includes
    assert "BatteryPubSubTypes.hpp" in header.includes
    assert header.header_guard == "DDS_TYPES"


def test_topic_id_header_guard_calculation():
    """Verify header guard strips path and extensions."""
    data = {
        "output_file_path": "long/path/to/my_ports_ids.hpp",
        "topic_ids": [{"name": "T", "c_var_name": "kT"}],
        "namespace": "custom_ns",
    }
    header = TopicIdHeader.model_validate(data)

    assert header.topic_ids[0].c_var_name == "kT"
    assert header.topic_ids[0].name == "T"
    assert header.header_guard == "MY_PORTS_IDS"
    assert header.namespace == "custom_ns"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
