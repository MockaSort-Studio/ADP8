"""system_pip — Bazel module extension that exposes host-installed Python packages.

Mirrors the pip.parse API so BUILD files can use requirement() exactly as they
would with a rules_python pip hub, sourcing packages from the host Python
environment instead of downloading from PyPI.

Designed for heavy simulation packages (genesis, torch) where Bazel-managed pip
is wasteful and the environment is controlled by the container image anyway.

Usage in MODULE.bazel:

    sim_pip = use_extension("//tools/system_pip:extensions.bzl", "system_pip")
    sim_pip.parse(
        hub_name        = "sim_py_req",
        requirements_lock = "//simulation/py_req:requirements.in",
        fail_on_missing = False,
    )
    use_repo(sim_pip, "sim_py_req")

Usage in BUILD files (drop-in replacement for the pip hub):

    load("@sim_py_req//:requirements.bzl", "requirement")
    deps = [requirement("genesis-world")]

# proudly AI-generated, human-reviewed
"""

# ---------------------------------------------------------------------------
# Discovery script — run via rctx.execute against the host Python interpreter.
# Double-braced {{ }} are literal Python braces after Starlark .format() runs.
# ---------------------------------------------------------------------------
_LOCATE_SCRIPT = """
import importlib.metadata, json, pathlib, sys

pkg = {pkg_literal}
try:
    dist = importlib.metadata.distribution(pkg)
except importlib.metadata.PackageNotFoundError:
    print(json.dumps({{"error": "not_found"}}))
    sys.exit(0)

# locate_file(".") returns the installation root (typically site-packages).
site_packages = pathlib.Path(str(dist.locate_file(".")))

top_txt = dist.read_text("top_level.txt")
if top_txt:
    declared = [l.strip() for l in top_txt.splitlines() if l.strip()]
else:
    seen = set()
    for f in (dist.files or []):
        parts = pathlib.PurePosixPath(str(f)).parts
        if (parts
                and not parts[0].endswith(".dist-info")
                and not parts[0].endswith(".data")
                and "." not in parts[0]):
            seen.add(parts[0])
    declared = list(seen)

# Classify as package directories or single-file modules (e.g. typing_extensions.py).
top_dirs = [t for t in declared if (site_packages / t).is_dir()]
top_files = [t for t in declared
             if t not in top_dirs and (site_packages / (t + ".py")).is_file()]

# Catch single-file modules absent from top_level.txt and the fallback scan.
for f in (dist.files or []):
    parts = pathlib.PurePosixPath(str(f)).parts
    if (len(parts) == 1
            and parts[0].endswith(".py")
            and not parts[0].startswith("_")):
        mod = parts[0][:-3]
        if mod not in top_dirs and mod not in top_files:
            if (site_packages / parts[0]).is_file():
                top_files.append(mod)

# C extension-only modules: a bare .so/.pyd at the top level with no .py wrapper.
# e.g. OpenEXR.cpython-312-x86_64-linux-gnu.so  ->  module name "OpenEXR"
import importlib.machinery
_ext_suffixes = importlib.machinery.EXTENSION_SUFFIXES
top_so = []  # list of actual filenames (e.g. "OpenEXR.cpython-312-....so")
for f in (dist.files or []):
    parts = pathlib.PurePosixPath(str(f)).parts
    if len(parts) == 1:
        fname = parts[0]
        for suffix in _ext_suffixes:
            if fname.endswith(suffix):
                mod = fname[:-len(suffix)]
                if ("." not in mod
                        and not mod.startswith("_")
                        and mod not in top_dirs
                        and mod not in top_files):
                    if (site_packages / fname).is_file():
                        top_so.append(fname)
                break

# manylinux wheels bundle native .so deps in <pkg>.libs/ sibling directories.
for f in (dist.files or []):
    parts = pathlib.PurePosixPath(str(f)).parts
    if parts and parts[0].endswith(".libs") and (site_packages / parts[0]).is_dir():
        if parts[0] not in top_dirs:
            top_dirs.append(parts[0])

# Parse direct requirements, skipping extras-conditional deps.
# "foo>=1.0; extra == 'cuda'" is optional and intentionally skipped so we
# don't accidentally pull in CUDA/dev/test variants as hard deps.
requires = []
for req_str in (dist.requires or []):
    if ";" in req_str:
        _, marker = req_str.split(";", 1)
        if "extra ==" in marker or "extra==" in marker:
            continue
    name = req_str.split(";")[0].strip()
    for sep in ["==", ">=", "<=", "!=", "~=", ">", "<"]:
        name = name.split(sep)[0]
    # Also handle old-style parenthesised version specs: "marshmallow (>=3.18.0)".
    name = name.split("[")[0].split("(")[0].strip()
    if name:
        requires.append(name)

print(json.dumps({{
    "top_dirs": top_dirs,
    "top_files": top_files,
    "top_so": top_so,
    "location": str(site_packages),
    "requires": requires,
}}))
"""

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_pkg_names(content):
    """Extracts bare package names from a requirements.in-style file."""
    names = []
    for line in content.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("--"):
            continue
        name = line.split("==")[0].split(">=")[0].split("<=")[0]
        name = name.split("[")[0].split("\\")[0].strip()
        if name:
            names.append(name)
    return names

def _normalize(name):
    """PEP 503-ish normalization used for deduplication."""
    return name.lower().replace("-", "_").replace(".", "_")

def _target_name(pkg_name):
    """Normalises a package name to a valid Bazel target name."""
    return "_" + _normalize(pkg_name)

def _gen_stub(tgt, pkg):
    return """py_library(
    name = "{tgt}",
    # system_pip stub: '{pkg}' was not found in host Python at analysis time.
    visibility = ["//visibility:public"],
)""".format(tgt = tgt, pkg = pkg)

def _gen_lib(tgt, dir_tops, file_tops, so_files, deps):
    """Generates a py_library over symlinked top-level package directories/files."""
    srcs_parts = []
    data_parts = []
    for t in dir_tops:
        srcs_parts.append('glob(["{t}/**/*.py"], allow_empty = True)'.format(t = t))
        data_parts.append(
            'glob(["{t}/**/*"], exclude = ["{t}/**/*.py"], allow_empty = True)'.format(t = t),
        )
    for f in file_tops:
        srcs_parts.append('["{f}.py"]'.format(f = f))
    for so in so_files:
        # C extension-only module: no .py, the .so IS the module.
        data_parts.append('["{so}"]'.format(so = so))

    srcs = " + ".join(srcs_parts) if srcs_parts else "[]"
    data = " + ".join(data_parts) if data_parts else "[]"
    deps_str = "[" + ", ".join(['":{}"'.format(d) for d in deps]) + "]"

    return """py_library(
    name = "{tgt}",
    # proudly AI-generated, human-reviewed
    srcs = {srcs},
    data = {data},
    deps = {deps},
    visibility = ["//visibility:public"],
)""".format(tgt = tgt, srcs = srcs, data = data, deps = deps_str)

def _gen_requirements_bzl(apparent_hub_name, req_map):
    # apparent_hub_name is the user-visible name (e.g. "sim_py_req"), NOT
    # rctx.name which is the canonical name (e.g. "+system_pip+sim_py_req").
    # Labels in generated files must use apparent names so they resolve
    # through the root module's repo mapping.
    entries = "\n".join([
        '    "{}": "@{}//:{}",'.format(k, apparent_hub_name, v)
        for k, v in req_map.items()
    ])
    return (
        '# Auto-generated by system_pip. Do not edit.\n' +
        '_REQUIREMENTS = {\n' +
        entries + "\n" +
        '}\n\n' +
        'def requirement(name):\n' +
        '    normed = name.lower().replace("-", "_").replace(".", "_")\n' +
        '    label = _REQUIREMENTS.get(name) or _REQUIREMENTS.get(normed)\n' +
        '    if not label:\n' +
        '        fail("system_pip: unknown package \'{}\'.".format(name))\n' +
        '    return label\n'
    )

# ---------------------------------------------------------------------------
# Repository rule
# ---------------------------------------------------------------------------

def _system_pip_hub_impl(rctx):
    python = rctx.attr.python_interpreter
    seed_pkgs = _parse_pkg_names(rctx.read(rctx.attr.requirements_lock))

    # Transitive discovery via BFS.
    # Starlark has no 'while'; iterate over a generous range and break when done.
    # Iterating over range() (not queue) means queue.append() inside is allowed.
    # normed_to_orig: normalized name -> first-seen original name (stable target names)
    # pkg_info: original name -> discovery result dict, or None for stubs
    normed_to_orig = {}
    pkg_info = {}
    queue = list(seed_pkgs)
    cursor = 0

    for _i in range(100000):
        if cursor >= len(queue):
            break
        pkg = queue[cursor]
        cursor += 1

        normed = _normalize(pkg)
        if normed in normed_to_orig:
            continue
        normed_to_orig[normed] = pkg

        result = rctx.execute([python, "-c", _LOCATE_SCRIPT.format(
            pkg_literal = '"' + pkg + '"',
        )])

        if result.return_code != 0 or '"error"' in result.stdout:
            if rctx.attr.fail_on_missing and pkg in seed_pkgs:
                fail("system_pip: '{}' not found in host Python.".format(pkg))
            pkg_info[pkg] = None  # stub — emit empty py_library
            continue

        info = json.decode(result.stdout.strip())
        pkg_info[pkg] = info

        # Enqueue undiscovered transitive requirements.
        for req in info["requires"]:
            if _normalize(req) not in normed_to_orig:
                queue.append(req)

    # Symlink top-level modules into the external repo.
    # First package to claim a name wins; conflicts handled gracefully.
    dir_owner = {}   # directory name -> pkg
    file_owner = {}  # single-file module name -> pkg
    so_owner = {}    # .so filename -> pkg

    for pkg, info in pkg_info.items():
        if info == None:
            continue
        for d in info["top_dirs"]:
            if d not in dir_owner:
                rctx.symlink(info["location"] + "/" + d, d)
                dir_owner[d] = pkg
        for f in info["top_files"]:
            if f not in file_owner:
                rctx.symlink(info["location"] + "/" + f + ".py", f + ".py")
                file_owner[f] = pkg
        for so in info["top_so"]:
            if so not in so_owner:
                rctx.symlink(info["location"] + "/" + so, so)
                so_owner[so] = pkg

    # Generate py_library targets.
    build_parts = []
    for pkg, info in pkg_info.items():
        tgt = _target_name(pkg)

        if info == None:
            build_parts.append(_gen_stub(tgt, pkg))
            continue

        owned_dirs = [d for d in info["top_dirs"] if dir_owner.get(d) == pkg]
        owned_files = [f for f in info["top_files"] if file_owner.get(f) == pkg]
        owned_so = [so for so in info["top_so"] if so_owner.get(so) == pkg]

        # Resolve dep target names from the requirements list (deduplicated).
        dep_tgts = []
        dep_seen = {}
        for req in info["requires"]:
            req_orig = normed_to_orig.get(_normalize(req))
            if req_orig:
                tgt_dep = _target_name(req_orig)
                if tgt_dep not in dep_seen:
                    dep_seen[tgt_dep] = True
                    dep_tgts.append(tgt_dep)

        build_parts.append(_gen_lib(tgt, owned_dirs, owned_files, owned_so, dep_tgts))

    # requirements.bzl exposes only the seed packages; transitives are
    # implementation details reached through the dep graph.
    req_map = {pkg: _target_name(pkg) for pkg in seed_pkgs}

    rctx.file("BUILD.bazel", "\n\n".join(build_parts) + "\n")
    # rctx.name is the canonical name (e.g. "+system_pip+sim_py_req").
    # Use the hub_name attribute (apparent name) for labels in requirements.bzl.
    rctx.file("requirements.bzl", _gen_requirements_bzl(rctx.attr.hub_name, req_map))

_system_pip_hub = repository_rule(
    implementation = _system_pip_hub_impl,
    attrs = {
        "hub_name": attr.string(mandatory = True),
        "python_interpreter": attr.string(default = "python3"),
        "requirements_lock": attr.label(allow_single_file = True),
        "fail_on_missing": attr.bool(default = False),
    },
)

# ---------------------------------------------------------------------------
# Module extension
# ---------------------------------------------------------------------------

def _system_pip_impl(mctx):
    for mod in mctx.modules:
        for tag in mod.tags.parse:
            _system_pip_hub(
                name = tag.hub_name,
                hub_name = tag.hub_name,
                python_interpreter = tag.python_interpreter,
                requirements_lock = tag.requirements_lock,
                fail_on_missing = tag.fail_on_missing,
            )

_parse_tag = tag_class(attrs = {
    "hub_name": attr.string(mandatory = True),
    "python_interpreter": attr.string(default = "python3"),
    "requirements_lock": attr.label(mandatory = True),
    "fail_on_missing": attr.bool(default = False),
})

system_pip = module_extension(
    implementation = _system_pip_impl,
    tag_classes = {"parse": _parse_tag},
)
