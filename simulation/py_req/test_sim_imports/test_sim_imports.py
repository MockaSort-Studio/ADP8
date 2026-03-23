"""Smoke test: every package in requirements.in must be importable.

Package names are read at runtime from requirements.in — no hardcoded list.
Top-level import names are resolved via importlib.metadata, mirroring the
logic system_pip uses during discovery, so if system_pip can see it this
test can import it.

Packages that are not installed in the host environment (stubs) are skipped
with a warning rather than a failure, consistent with fail_on_missing=False.
"""

# proudly AI-generated, human-reviewed
import importlib
import importlib.metadata
import os
import pathlib
import sys


def _parse_pkg_names(content: str) -> list[str]:
    names = []
    for line in content.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("--"):
            continue
        name = line.split("==")[0].split(">=")[0].split("<=")[0]
        name = name.split("[")[0].strip()
        if name:
            names.append(name)
    return names


def _top_level_modules(pkg_name: str) -> list[str] | None:
    """Return importable top-level names for pkg_name, or None if not installed."""
    try:
        dist = importlib.metadata.distribution(pkg_name)
    except importlib.metadata.PackageNotFoundError:
        return None

    site = pathlib.Path(str(dist.locate_file(".")))

    top_txt = dist.read_text("top_level.txt")
    if top_txt:
        declared = [l.strip() for l in top_txt.splitlines() if l.strip()]
    else:
        seen: set[str] = set()
        for f in dist.files or []:
            parts = pathlib.PurePosixPath(str(f)).parts
            if (
                parts
                and not parts[0].endswith(".dist-info")
                and not parts[0].endswith(".data")
                and "." not in parts[0]
            ):
                seen.add(parts[0])
        declared = list(seen)

    # Only names that actually exist on disk — top_level.txt can be stale.
    tops = [
        t
        for t in declared
        if (site / t).is_dir() or (site / (t + ".py")).is_file()
    ]
    return tops or None


def main() -> int:
    # Locate requirements.in via Bazel's runfiles layout.
    # In Bzlmod the main workspace is always "_main".
    srcdir = os.environ.get("TEST_SRCDIR", "")
    workspace = os.environ.get("TEST_WORKSPACE", "_main")
    req_path = pathlib.Path(srcdir) / workspace / "simulation/py_req/requirements.in"

    if not req_path.exists():
        print(f"ERROR: requirements.in not found at {req_path}", file=sys.stderr)
        return 1

    packages = _parse_pkg_names(req_path.read_text())

    failures: list[str] = []
    skipped: list[str] = []

    for pkg in packages:
        tops = _top_level_modules(pkg)
        if tops is None:
            skipped.append(pkg)
            continue

        for mod in tops:
            try:
                importlib.import_module(mod)
            except Exception as exc:
                failures.append(f"  {pkg} → import {mod}: {type(exc).__name__}: {exc}")

    if skipped:
        print(f"skipped (not installed / no metadata): {', '.join(skipped)}")

    if failures:
        print(f"FAILED ({len(failures)} import error(s)):")
        for line in failures:
            print(line)
        return 1

    installed = len(packages) - len(skipped)
    print(f"OK — {installed}/{len(packages)} packages imported successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())
