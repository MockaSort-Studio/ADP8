# Generators

`core/generators/` — Python codegen from YAML configuration to C++ headers.

Boilerplate is not written by hand. Typed pub/sub specs and parameter tables are declared in YAML and generated into human-readable headers. The generator output is committed — nothing is hidden.

---

## What gets generated

**Ports** — `cc_dds_components(name="my_ports", ...)` produces two sub-targets:

`my_ports_types` — FastDDS C++ type support, compiled from IDL by `fastddsgen`:

- One `PubSubType` class per IDL struct (e.g. `ImuRawPubSubType`)

`my_ports_ports` — Javelina-RT topic spec headers, generated from YAML:

- `my_ports_dds_types.hpp` — includes for all generated PubSubType headers
- `my_ports_sub_ids.hpp` — `constexpr char[]` topic name constants (subscriptions)
- `my_ports_subscriptions.hpp` — `TopicList<...>` type alias for subscription specs
- `my_ports_pub_ids.hpp` — `constexpr char[]` topic name constants (publications)
- `my_ports_publications.hpp` — `TopicList<...>` type alias for publication specs

**Parameters** — `cc_parameters(name="my_params", ...)` produces:

- `my_params_parameters.hpp` — tag structs, `LookupTable` typedef, default initializer struct, `ParametersProvider` typedef

---

## Input YAML formats

### Ports YAML

```yaml
subscriptions:
  - imu_data:
      type: ImuRaw
      queue_size: 5
  - power:
      type: BatteryStatus

publications:
  - control_cmd:
      type: ControlCommand
```

`queue_size` defaults to 1 if omitted. `type` must match a struct name in the provided IDL files.

### Parameters YAML

```yaml
my_controller:
  - gain:
      type: float
      value: 1.5
  - offsets:
      type: float[3]
      value: [0.1, 0.2, 0.3]
  - enabled:
      type: bool
      value: true
```

Array types use `Type[N]` syntax. The generator expands `float[3]` to `float, float, float` — matching the variadic `TableItem<Tag, float, float, float>` form.

---

## Pipeline

```
YAML + IDL files
     │
     ▼
Pydantic models (gen_data_models.py)
  ├── Ports → subscriptions[], publications[]
  │     └── TopicSpec: type, topic_id, queue_size
  └── ParameterSet → name, params[]
        └── ParameterEntry: name, type, value
     │
     ▼
Model builders (gen_utils.py)
  ├── dds_types_header_model()
  ├── dds_topic_ids_pub_sub_header_models()
  ├── dds_topic_specs_pub_sub_header_models()
  └── parameters_header_model()
     │
     ▼
Jinja2 templates (templates/*.hpp.jinja)
     │
     ▼
Generated C++ headers
```

Type validation against IDL files happens at model construction time. If a YAML declares a type not found in any IDL, `_assert_type_exists` raises `RuntimeError` before any file is written.

---

## Bazel integration

Two Bazel rules, two public macros:

### `cc_dds_components` (macro in `defs.bzl`)

```python
cc_dds_components(
    name = "my_ports",
    idls = ["//messages:my_idl"],
    ports_yaml = "my_ports.yaml",
    namespace = "gen",  # optional, defaults to "gen"
)
```

Internally calls two rules and produces two sub-targets:

- `:{name}_types` — compiles IDL files into FastDDS C++ type support via `_cc_fastdds_types` (runs `fastddsgen`)
- `:{name}_ports` — generates Javelina-RT topic spec headers from YAML via `_cc_dds_ports`; depends on `:{name}_types`

`cc_dds_components(name="my_ports")` → deps `:my_ports_types` and `:my_ports_ports`.

### `cc_parameters` (macro in `defs.bzl`)

```python
cc_parameters(
    name = "my_params",
    yaml_parameters = "my_params.yaml",
    namespace = "gen",
)
```

### Using generated targets

Both macros produce `cc_library` targets. Add them to `deps` in your task's `cc_library`:

```python
cc_library(
    name = "my_task",
    srcs = ["my_task.cpp"],
    hdrs = ["my_task.hpp"],
    deps = [
        ":my_ports_ports",   # spec headers (sub-target of cc_dds_components)
        ":my_params",        # parameters header (cc_parameters)
        "//core/lifecycle:dds",
    ],
)
```

---

## Namespace

The generated C++ is placed in the namespace specified by the `namespace` attribute (default: `gen`). This allows multiple components in the same binary to have their own generated namespaces without collision — relevant when Javelina-RT is extracted as a standalone library.

---

## Data model normalization

The Pydantic models apply transformations before template rendering:

- **Type names**: `ImuRaw` → `ImuRawPubSubType` (FastDDS convention)
- **Topic IDs**: `imu_data` → `ImuDataTopic`, `kImuDataTopicName`
- **Parameter names**: `my_gain` → `MyGainTag` (PascalCase + Tag suffix)
- **Array types**: `float[3]` → `float, float, float` (for variadic template args)
- **Bool values**: Python `True`/`False` → `true`/`false` (C++ form)
- **Include paths**: Bazel-generated paths are stripped to filename only (`#include "name.hpp"`)

---

```python
# Hamlet 🐗 — wrote docs for the thing that writes the code. Peak recursion.
```
