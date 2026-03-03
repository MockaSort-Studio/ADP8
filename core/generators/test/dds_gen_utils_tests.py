import sys

import pytest
import yaml

from core.generators.gen_utils import (
    dds_ports_from_yaml,
    dds_types_header_model,
)


@pytest.fixture
def mock_idl_types():
    """Simulates types found in an IDL file."""
    return ["Position", "BatteryStatus"]


@pytest.fixture
def yaml_with_unknown_type(tmp_path):
    """YAML referencing a type not present in the IDL list."""
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


@pytest.fixture
def yaml_all_valid_types(tmp_path):
    """YAML where all types are present in the IDL list."""
    content = {
        "publications": [{"gps_pos": {"type": "Position"}}],
        "subscriptions": [{"power": {"type": "BatteryStatus"}}],
    }
    yaml_path = tmp_path / "config.yaml"
    with open(yaml_path, "w") as f:
        yaml.dump(content, f)
    return str(yaml_path)


def test_dds_ports_unknown_type_raises(yaml_with_unknown_type, mock_idl_types):
    """
    Unknown IDL types must raise — silent filtering produces broken output with no signal.
    """
    with pytest.raises(RuntimeError, match="ImuRaw"):
        dds_ports_from_yaml(yaml_with_unknown_type, mock_idl_types)


def test_dds_types_header_generation(yaml_all_valid_types, mock_idl_types):
    """Verify that the IdlTypeHeader correctly inherits from Ports."""
    ports = dds_ports_from_yaml(yaml_all_valid_types, mock_idl_types)
    output_path = "bazel-out/bin/dds_types.hpp"

    header_model = dds_types_header_model(ports, output_path)

    assert any("PositionPubSubTypes.hpp" in inc for inc in header_model.includes)
    assert header_model.header_guard == "DDS_TYPES"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__]))
