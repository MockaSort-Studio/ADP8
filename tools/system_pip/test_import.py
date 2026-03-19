"""system_pip happy-path test.

numpy is listed in testdata/requirements.in and is installed in the host
environment, so system_pip must discover it, symlink it into the external
repo, and expose it as a py_library.  If any of those steps are broken
this test fails at import time.
"""

# proudly AI-generated, human-reviewed
import sys

import numpy

arr = numpy.array([1, 2, 3])
assert arr.sum() == 6, f"unexpected sum: {arr.sum()}"
assert numpy.__version__, "numpy version string is empty"

print(f"numpy {numpy.__version__} imported and works — system_pip discovery OK")
sys.exit(0)
