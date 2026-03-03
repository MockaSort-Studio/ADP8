# ADP8 — State of the Repo

> Proudly AI-generated, human-reviewed. 🐽
> This is a working assessment, not a report for stakeholders. Language is direct.

---

## TL;DR

The core of what you're building works. Javelina-RT is architecturally sound — the template system is elegant,
the execution model is clean, and the DDS integration is genuinely good. The rest of the repo (applications,
simulation, CI, docs) ranges from functional-but-rough to skeleton. The two biggest systemic issues are:
dead code that creates confusion about canonical choices, and an almost total absence of documentation
for anything beyond "how to run the thing."

Nothing here is broken beyond repair. Some of it will require decisions before code.

---

## What's Actually Good

- **ExecutionEngine** — the self-rescheduling lambda pattern is clever and it works. Priority queue semantics
  are correct. Graceful shutdown is handled.

- **Template metaprogramming throughout core** — FNV-1a compile-time hashing, tag dispatch, tuple-based
  heterogeneous lookup. Zero-overhead abstractions that do what they promise. The LookupTable
  in particular is well-implemented and well-tested (99% coverage, meaningful tests).

- **DDSTask architecture** — type-safe pub/sub binding via templates, zero runtime indirection in hot paths,
  correct RAII everywhere. The `FillInputs → Execute → FlushOutputs` pattern is clean and teachable.

- **CI pipeline design** — using `bazel-diff` for incremental testing on PRs is the right call. The pig-nose
  trigger (`🐽` emoji on issue comments) is *chef's kiss* for a MockaSort project.

- **Bazel BUILD visibility** — conservative and correct. No circular deps, no over-exposed internals.

- **Application structure** — mirrors professional autonomous drive software. Planning, control, egomotion,
  actuation, teleop are clean separations that make sense and will stay sensible as the system grows.

- **C++ simulation** — complete closed-loop: receives ActuatorCommands, outputs IMU/steering/ground-truth.
  Physics is simplified but correct. This is usable for control validation.

---

## Issues by Priority

### 🔴 Critical — Fix Before Anything Else

**1. Dead duplicate: `core/communication/size_constrained_queue.hpp`**
The canonical queue lives in `core/support/utils/`. The `communication/` version is not included by
anything in core, has no `operator[]`, no `TransferTo()`, uses plain mutex instead of atomics,
and diverged long ago. It's a trap for contributors. Delete it.
> `core/communication/size_constrained_queue.hpp`

**2. Hardcoded `gen` namespace in generators**
All generated code lands in `namespace gen::*`. Two Javelina-RT projects using the codegen pipeline
collide instantly. The namespace is hardcoded in `gen_data_models.py:136` and the Bazel rules
(`dds_ports_gen.bzl`, `parameters_gen.bzl`) expose no `namespace` attribute. This is a **blocker
for Javelina extraction** and a problem for anyone using this in a real multi-component system.
> `core/generators/gen_data_models.py:136`, `core/generators/dds_ports_gen.bzl`

**3. Silent type validation failure in generator**
`gen_utils.py:21` prints "not found" and filters the port. If a YAML references a type that doesn't
exist in the IDL, the generator silently drops the port and proceeds. The user gets a partial
generated header with no error. Needs a `--strict` mode or explicit error.
> `core/generators/gen_utils.py:21`

**4. YAML parse error returns None instead of raising**
`gen_utils.py:43` catches `yaml.YAMLError`, prints it, and falls through to `return yaml_dict`
which at that point is undefined. The caller gets a `NameError` with no indication of what went wrong.
> `core/generators/gen_utils.py:43`

**5. Teleop keyboard device hardcoded**
`teleop_node.cpp:27` hardcodes `/dev/input/event6` with a comment saying "CHANGE THIS" that nobody
reading compiled output will ever see. Node fails on any other machine.
> `applications/teleop/teleop_node.cpp:27`

---

### 🟠 High — Address Before Standalone Release

**6. DDSContext topic cache has a concurrency race**
`dds_context.hpp:61-67` — `GetDDSTopic()` is called during task initialization. If two tasks
initialize concurrently with the same topic name, both threads pass the `find_if` check and both
create topic objects. This is safe only if task initialization is single-threaded (verify this
assumption or fix with a mutex on the cache).
> `core/communication/dds_context.hpp:61`

**7. DDSPublisher silently drops messages when no subscribers matched**
`dds_publisher.hpp:104-111` — `Publish()` returns `false` with no log, no warning. A task
publishing on startup before subscribers discover it loses all those messages silently.
Worth documenting at minimum; worth a debug log in practice.
> `core/communication/dds_publisher.hpp:104`

**8. Output endpoints can repeat stale data**
`data_endpoint.hpp:41-52` — `Sync()` on an output endpoint publishes `queue_[0]` every cycle
regardless of whether a new `Push()` happened. A task that doesn't push every cycle re-publishes
the last sample. Correct for DDS keep-last semantics, but nowhere documented. A new Javelina user
will spend an afternoon debugging this.
> `core/lifecycle/data_endpoint.hpp:41`

**9. Global mutable signal state in DDSApplication**
`dds_application.hpp:17` — `detail::shutdown_requested` is a namespace-scoped static atomic.
It works, but it's invisible, untestable in isolation, and makes the class non-reentrant
(instantiate two DDSApplications and you break both). Consider making it a member.
> `core/lifecycle/dds_application.hpp:17`

**10. Two parallel simulations with no integration**
`simulation/cpp_sim/` and `simulation/` (Genesis Python) exist as independent implementations
with no shared interface, no documentation on which to use and when, and no bridge between them.
The Genesis version is significantly more sophisticated (noise models, GPU physics, detailed motor
dynamics). The C++ version is lighter and Javelina-native. Both are useful. Neither is clearly
the answer. Decision needed before this confuses anyone else.

**11. Visitor pattern has no consumers and no backends**
`core/support/visit/` — the framework is defined (visitor_base, visitable, property macros)
and tested, but nothing in core or applications uses it. No serialization backends exist
(no JSON, CBOR, protobuf visitor implementations). Either wire it to something real or
move it out of core with a note. Infrastructure that nobody uses becomes archaeology.
> `core/support/visit/`

---

### 🟡 Medium — Before "Golden Standard" Is Claimable

**12. All control gains are hardcoded**
Stanley, Pure Pursuit, and the P-controller all have their tuning constants baked into
`point_follower.cpp` (Kp=1.2, L_min=1.0, Kv=0.6, K=1.0, etc.). Javelina has a parameters
system for exactly this. Migrating would make tuning possible without recompile and would
serve as a real-world example of how parameters work.
> `applications/controls/point_follower.cpp:28,32,70,111,122`

**13. Dead controller code**
`control_node.cpp:68-70` — `point_follower()` and `pure_pursuit()` are implemented but
commented out. If they're not production-ready, delete them. If they are, expose them via
parameter selection. Commented-out code is just guilt made permanent.
> `applications/controls/control_node.cpp:68`

**14. State estimation is dead-reckoning only**
`state_estimation.cpp` (25 lines) is a kinematic integrator with no noise handling, no bias
estimation, and no fusion. Fine for a first pass. Not fine for a system that will drive anything
further than a few meters without drifting. No IMU bias. No wheel slip. Accelerating drift is
guaranteed. The Genesis simulation has noise models that will expose this immediately.

**15. No CI linting enforcement**
Code style is enforced locally via `clang-format` and `black` wrappers, but nothing in CI
checks for it. A PR with inconsistent formatting builds green. Either add a lint step to CI
or accept that formatting is honor-system.

**16. Python 3.14 toolchain pinned**
`MODULE.bazel` pins Python 3.14, which is a pre-release. May not be available in all
environments and will break the moment anyone tries to build outside this devcontainer.
3.12 or 3.13 would be more stable.

**17. `float[0]` in YAML parameters causes silent broken output**
`gen_data_models.py` normalizes `float[3]` → `float, float, float` but never validates
that the size is > 0. An array of zero elements generates `TableItem<Tag>` with no types,
which violates the LookupTable template. Compilation error, but from a confusing call site.

**18. `const_cast` in DDSPublisher**
`dds_publisher.hpp:108` — `writer_->write(const_cast<DDSDataType*>(&payload))`. This is a
FastDDS API friction issue (requires non-const ptr). Functionally safe, but it'll catch
someone's eye in a code review. Worth a comment explaining why.

---

### 🔵 Low — Quality of Life, Good for External Contributors

**19. No architecture documentation**
The MkDocs site is 95% stub. There is no document explaining:
- What Javelina-RT is and why it exists
- How to build a task (the whole DDSTask → DDS endpoint → scheduler chain)
- How code generation works (YAML → IDL → Jinja → C++)
- How the application layer plugs in
Someone cloning this repo to learn cannot get started without reading every file in `core/`.

**20. No Javelina-RT integration guide**
Not "here is the API" — "here is how to build a new node, end to end." One example is worth
a thousand header comments. The lifecycle tests exist but aren't presented as tutorials.

**21. `orphaned test/` directories**
`test/cpptest/` (hello world + keyboard read) and `test/ros2_bazel/chatter/` are not in CI,
not in mkdocs, and not connected to anything. Either integrate them or remove them. They're
noise.

**22. Thread ID logging in DDSSubscriber is always on**
`dds_subscriber.hpp:54` logs thread IDs on every message receive. This is a debugging
artifact. It should be behind a flag or removed. It'll show up in production logs.

**23. `actuation_interface_node.cpp` TODO**
`map_control2actuators.cpp:26` has a TODO: "different cars will be handled here differently."
This is a known extensibility point. Either track it properly or make it a GitHub issue
so it doesn't get lost.

**24. `ai/` directory has no status marker**
The AI folder is archived, but nothing in the directory says that. Someone opening the repo
fresh will read the AI README, try to set up PyTorch Lightning, and waste an afternoon.
A one-line `ARCHIVED.md` or a note in the `README.md` would fix this in 30 seconds.

---

## Javelina-RT: Extraction Readiness

For Javelina to stand alone as an importable package, blockers in order:

| # | Issue | Effort |
|---|-------|--------|
| 1 | Configurable generator namespace | Medium |
| 2 | Remove dead `communication/size_constrained_queue.hpp` | Trivial |
| 3 | Fix generator YAML error handling | Small |
| 4 | Fix silent type validation failure | Small |
| 5 | DDSContext concurrent topic cache | Small |
| 6 | Architecture documentation | Large |
| 7 | Integration guide / tutorial | Medium |
| 8 | Output sink "stale publish" semantics documented | Trivial |
| 9 | Signal handler global state | Medium |

---

## Simulation: Decision Point

The two simulations serve different purposes but there's no guidance on which to use:

| | C++ (`cpp_sim/`) | Genesis Python (`simulation/`) |
|--|---|---|
| **Integration** | Javelina-RT native | ROS2 node |
| **Physics** | Simplified kinematic | GPU-accelerated, slip, noise |
| **Motor model** | First-order lag | Full electrical model (back-EMF, torque constant) |
| **Sensor noise** | None (TODO in code) | Configurable Gaussian + cross-axis coupling |
| **Portability** | Runs anywhere | Needs GPU, Genesis install |
| **Use case** | Control loop validation, CI | Realistic training, pre-deployment testing |

**Recommendation**: Keep both. Make the choice explicit in documentation. The C++ sim belongs in CI.
The Genesis sim belongs in the RL training loop and hardware-adjacent testing.

---

## AI Directory

Archived. Active RL work is in a separate repo. The only thing missing is a visible status indicator
so nobody wastes time trying to run it. A one-liner in the folder's README would be enough.

---

## Near-Term Candidates (Suggested Priority)

These are suggestions. You decide the order.

**Immediate (low effort, high clarity):**
- [x] Delete `core/communication/size_constrained_queue.hpp`
- [x] Fix YAML parse error handling in generator (`gen_utils.py:43`)
- [x] Add error (not silent filter) on missing IDL type in generator
- [x] Mark `ai/` as archived in its README
- [ ] Remove commented-out controllers from `control_node.cpp`

**Short-term (medium effort, high value):**
- [x] Make generator namespace configurable (bzl attribute → Python arg → template variable)
- [x] Write one end-to-end "how to build a Javelina task" guide
- [ ] Extract control gains into YAML parameters (good real-world generator usage example)
- [ ] Make teleop device path configurable (not hardcoded)
- [ ] Consolidate simulation documentation (what each sim is for, when to use which)

**Medium-term (larger effort, required for golden standard):**
- [ ] Architecture documentation in MkDocs (Javelina-RT concepts, app layer, codegen pipeline)
- [ ] CI linting enforcement (clang-format check on PR)
- [ ] State estimation: gyro bias estimation at minimum
- [ ] Visitor pattern: provide at least one serialization backend or move out of core
- [ ] DDSContext topic cache: thread safety audit and fix or explicit documentation of single-threaded assumption

---

*Generated from a full codebase pass. Verified against actual file contents, not guesses.
Prioritization is advisory — architectural calls remain yours.*
