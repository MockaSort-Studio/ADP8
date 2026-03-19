"""system_pip stub test.

nonexistent-test-package is listed in testdata/requirements.in but is
deliberately not installed anywhere.  system_pip must emit an empty stub
py_library (fail_on_missing = False), which means:

  1. The BUILD target exists — so dependent BUILD files load cleanly.
  2. The stub provides NO files to the runfiles tree — so importing the
     package at runtime must raise ModuleNotFoundError.

This test verifies condition 2.  Condition 1 is implicitly verified by the
fact that this test target itself builds successfully despite depending on
the stub via requirement("nonexistent-test-package").
"""

# proudly AI-generated, human-reviewed
import importlib.util
import sys

PKG = "nonexistent_test_package"

spec = importlib.util.find_spec(PKG)
assert spec is None, (
    f"expected {PKG!r} to be absent from the runfiles tree, "
    f"but importlib.util.find_spec returned: {spec}"
)

try:
    import nonexistent_test_package  # noqa: F401
    raise AssertionError(f"import {PKG!r} should have raised ModuleNotFoundError")
except ModuleNotFoundError:
    pass

print(f"stub for {PKG!r} correctly provides nothing importable — OK")
sys.exit(0)
