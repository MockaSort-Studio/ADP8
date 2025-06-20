load("@pip_requirements//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

py_test(
    name = "requirements_test",
    srcs = [
        "requirements_test.py",
    ],
    deps = [
        requirement("lightning"),
        requirement("gymnasium"),
        requirement("torch"),
        requirement("torchvision"),
        requirement("transformers"),
        requirement("torchmetrics"),
        requirement("torchaudio"),
        requirement("tensorboard"),
        requirement("comet_ml"),
        requirement("datasets"),
        requirement("numpy"),
        requirement("pandas"),
        requirement("matplotlib"),
        requirement("jsonargparse"),
        requirement("mlxtend"),
    ],
)