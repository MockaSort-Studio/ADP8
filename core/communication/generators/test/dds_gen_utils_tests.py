import sys

import pytest
import yaml

from core.communication.generators.dds_gen_utils import (
    dds_ports_from_yaml,
    dds_types_header_model,
)


@pytest.fixture
def mock_idl_types():
    """Simulates types found in an IDL file."""
    return ["Position", "BatteryStatus"]


@pytest.fixture
def valid_yaml_file(tmp_path):
    """Creates a temporary YAML file with one valid and one invalid type."""
    content = {
        "publications": [{"gps_pos": {"type": "Position"}}],
        "subscriptions": [
            {"imu_data": {"type": "ImuRaw"}},
            {"power": {"type": "BatteryStatus"}},
        ],
    }
    yaml_path = tmp_path / "config.yaml"
    with open(yaml_path, "w") as f:
        yaml.dump(content, f)
    return str(yaml_path)


def test_dds_ports_filtering(valid_yaml_file, mock_idl_types):
    """
    Verify that ports with types not present in IDL are discarded.
    This is the most critical logic in your parser.
    """
    ports = dds_ports_from_yaml(valid_yaml_file, mock_idl_types)

    assert len(ports.publications) == 1
    assert len(ports.subscriptions) == 1

    sub_names = [s.topic_id.name for s in ports.subscriptions]
    assert "PowerTopic" in sub_names
    assert "ImuDataTopic" not in sub_names


def test_dds_types_header_generation(valid_yaml_file, mock_idl_types):
    """Verify that the IdlTypeHeader correctly inherits from Ports."""
    ports = dds_ports_from_yaml(valid_yaml_file, mock_idl_types)
    output_path = "bazel-out/bin/dds_types.hpp"

    header_model = dds_types_header_model(ports, output_path)

    assert any("PositionPubSubTypes.hpp" in inc for inc in header_model.includes)
    assert header_model.header_guard == "DDS_TYPES"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
