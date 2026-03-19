load("@pip_tools//:requirements.bzl", tool_requirement = "requirement")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//core/generators:defs.bzl", "cc_dds_components", "pybind_dds_bridge", "pybind_dds_type")

cc_dds_components(
    name = "ping",
    idls = glob(["messages/*.idl"]),
    namespace = "ping",
    ports_yaml = "node_ports.yml",
)

### pybinds

pybind_dds_type(
    name = "ping_message_py",
    idl = "messages/PingMessage.idl",
    type_deps = [":ping_types"],
)

pybind_dds_bridge(
    name = "ping_bridge_py",
    bridge_deps = [
        ":ping_ports",
        "//core/pybinds:py_dds_bridge",
    ],
    idls = glob(["messages/*.idl"]),
    namespace = "ping",
    ports_name = "ping_ports",
    ports_yaml = "node_ports.yml",
    type_lib = ":ping_message_py_lib",
)

py_test(
    name = "test_type_binding",
    srcs = ["test_type_binding.py"],
    deps = [
        ":ping_message_py_lib",
        tool_requirement("pytest"),
    ],
)

py_test(
    name = "test_bridge_smoke",
    srcs = ["test_bridge_smoke.py"],
    deps = [
        ":ping_bridge_py_lib",
        tool_requirement("pytest"),
    ],
)

### cpp

cc_test(
    name = "py_dds_bridge_test",
    srcs = ["py_dds_bridge_test.cpp"],
    deps = [
        ":ping_ports",
        "//core/pybinds:py_dds_bridge",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
