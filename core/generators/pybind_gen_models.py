"""Pydantic models for the pybind11 DDS binding code generator."""

from typing import List

from pydantic import BaseModel


# IDL primitive type → C++ parameter type for pybind11 property setters.
IDL_TO_CPP_TYPE: dict[str, str] = {
    "string": "std::string",
    "unsigned long": "uint32_t",
    "long": "int32_t",
    "unsigned short": "uint16_t",
    "short": "int16_t",
    "unsigned long long": "uint64_t",
    "long long": "int64_t",
    "float": "float",
    "double": "double",
    "boolean": "bool",
    "octet": "uint8_t",
    "char": "char",
}

STRING_IDL_TYPES = {"string"}


class FieldModel(BaseModel):
    """One field of a DDS IDL struct, with metadata needed for pybind11 binding."""

    name: str
    cpp_setter_type: str
    is_string: bool  # True → quoted in __repr__; False → std::to_string


class TypeBindingModel(BaseModel):
    """Data needed to render a pybind11 type binding .cpp for one IDL struct."""

    type_name: str       # e.g. "ChannelMessage"
    module_name: str     # e.g. "channel_message_py"
    idl_filename: str    # e.g. "ChannelMessage.idl"  (for the header comment)
    fields: List[FieldModel]
    output_file_path: str


class PortBindingEntry(BaseModel):
    """One topic entry in a bridge: method suffix, DDS topic constant, C++ type."""

    py_method_suffix: str   # e.g. "channel_a"
    topic_const: str        # e.g. "kChannelATopicName"
    type_name: str          # e.g. "ChannelMessage"
    type_module: str        # e.g. "channel_message_py"


class BridgeBindingModel(BaseModel):
    """Data needed to render a pybind11 PyDDSBridge binding .cpp."""

    module_name: str                       # e.g. "chatter_bridge_py"
    namespace: str                         # e.g. "cpp"
    ports_name: str                        # e.g. "chatter_ports" (→ include basename)
    subscriptions: List[PortBindingEntry]  # Python subscribes (cpp publishes)
    publications: List[PortBindingEntry]   # Python publishes (cpp subscribes)
    type_modules: List[str]                # unique, ordered (for module_::import)
    type_names: List[str]                  # unique C++ type names (for #includes)
    yaml_filename: str                     # e.g. "node_cpp_ports.yml"
    output_file_path: str
