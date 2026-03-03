# Contributing to ADP8

All submissions require review. We use GitHub Pull Requests. Describe your changes clearly
and link any relevant issues. See [GitHub Help](https://help.github.com/articles/about-pull-requests/)
if you're unfamiliar with the process.

This project is governed by the [Code of Conduct](CODE_OF_CONDUCT.md).

---

## C++ Style Guide

**Baseline**: [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
Configuration is in `.clang-format` at the repo root. Run `bazel run //:format` before pushing —
the formatter is in the devcontainer.

### Naming

| Thing | Convention | Example |
|-------|-----------|---------|
| Classes / Structs | `PascalCase` | `ExecutionEngine` |
| Methods / Free functions | `PascalCase` | `ExecuteStep()` |
| Local variables | `snake_case` | `task_count` |
| Member variables | `snake_case_` | `name_`, `running_` |
| Constants / `static constexpr` | `kPascalCase` | `kFrequency`, `kHash` |
| Namespaces | `lower::snake` | `core::lifecycle` |
| Template parameters | `PascalCase` or single cap | `TTask`, `T` |
| Header guards | `COMPONENT_SUBSYSTEM_FILENAME` | `CORE_LIFECYCLE_TASK_INTERFACE` |

### Compile-Time First

This codebase pays costs at compile time, not at runtime. When writing new code, lean into
the same patterns already in use:

- **`if constexpr`** over runtime branching inside templates.
- **Fold expressions** over explicit loops where the operands are a parameter pack.
  `(... || expr)`, `(... && expr)`, `(f(args), ...)` — prefer these over recursive TMP.
- **`constexpr` functions** for anything that can be evaluated at compile time.
  FNV-1a hashing, index lookups, type dispatch — all resolved before the binary exists.
- **Tag dispatch and `std::index_sequence`** for heterogeneous tuple traversal.
  Zero runtime cost, intent visible at the call site.
- **`auto`** for verbose iterator and template types. Not as a substitute for knowing
  what the type is.

When something can be a template specialization rather than a runtime branch, it should be.

### RAII

Resources have owners. Owners initialize in constructors, clean up in destructors.
No `init()` / `deinit()` pairs. No manual resource management.

If you're about to write a `Cleanup()` method, reconsider the ownership model instead.

### Design Rules

- **No `using namespace` in headers.** Ever.
- **No `assert` in production paths.** If a condition must hold, enforce it through types
  and design. `assert` is stripped in release builds.
- **No `dynamic_cast` or `typeid`.** If you need runtime type dispatch, the type hierarchy
  is wrong. Design it out.
- **No exceptions in lifecycle or communication hot paths.** Design for error states.
- **No preemptive synchronization.** If concurrent access isn't a current, concrete
  requirement, the mutex doesn't exist yet. Add it when the use case arrives, not before.
- **No owning raw pointers.** `unique_ptr`, `shared_ptr`, or stack allocation.

### Comments

Comments explain *why*, not *what*. If the what needs explaining, rename things first.

`// proudly AI-generated, human-reviewed` — not a disclaimer. Dry humor earns its place.

---

## Python Style Guide

**Formatter + linter**: `ruff` — handles both. Config in `pyproject.toml`. Line length: **90**.

**Type checker**: `ty`.

**Standard**: Python 3.12+.

### Rules

- **Type annotations** on all function signatures. `Any` is acceptable when the type is
  genuinely dynamic — don't annotate everything `Any` to silence the checker.
- **f-strings** over `.format()` and `%`.
- **No mutable default arguments.**
- **No bare `except:`** — catch specific exceptions and let everything else propagate.
- **No debug `print` left in production paths.**
- **Pydantic** for structured data that crosses module boundaries or requires validation.
  `NamedTuple` or `dataclass` for internal grouping with no validation needed.

### Docstrings

Document what's worth documenting. A function that reads like prose doesn't need a docstring.
A Bazel macro with non-obvious arguments does (use the Starlark `Args:` block format).
A pydantic model with non-obvious field semantics does. A three-line helper does not.

Quality over quantity. A bad docstring is worse than no docstring.

---

## Starlark (Bazel)

- Macro public args are positional. Private/internal rule attrs use `_` prefix.
- Document macro args with the Starlark `Args:` docstring format.