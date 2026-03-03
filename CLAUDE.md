# ADP8 — Claude Partnership Guide

## The Project

**ADP8** (Autodrive Pezzotto) is a sandbox for building autonomous systems, starting with an autonomous vehicle.
Built by **MockaSort** — no owners, no VC runway, no roadmap approved by a committee.
We build things because they need to exist.

Long-term: ADP8 becomes a **template repository**. Drone? Smart irrigation? Autonomous robot?
Pick what you need and go. For now, it hosts our first autonomous system and the tools we had to build
because the existing ones weren't worth the dependency.

Everything here is a tool-in-progress, used while being built. When something matures, it gets carved out
and released standalone. DIY until it doesn't need to be.

---

## MockaSort Brand

Brutalist. Concrete. Honest. Sharp irony. No applause for mediocrity.

We don't optimize for looking serious. We optimize for being useful.

**On AI**: we use it, we credit it, we own what it produces. The engineer who reads and understands
what the AI generated is doing engineering. The one who ships a prompt output without blinking
is doing something else — and calling it software development doesn't make it so.

Comments and docs in this style:
- `// proudly AI-generated, human-reviewed` — not a disclaimer, a flex
- Not: `// This elegant abstraction gracefully handles...` — just say what it does
- Dry humor earns its place. Enthusiasm doesn't.

---

## Repo Structure

```
ADP8/
├── core/           → Javelina-RT: real-time comm + lifecycle (C++)
├── applications/   → AV tasks: planning, control, egomotion, teleop
├── simulation/     → Physics simulation
├── ai/             → ARCHIVED. Active AI work lives elsewhere.
├── messages/       → DDS message definitions
├── third_party/    → FastDDS generator binary
├── tools/          → Build utilities
└── docs/           → MkDocs site
```

`applications/` mirrors the layout of professional autonomous drive software. It's the right place for
ADP8-specific tasks. Algorithms (e.g. perception) may pull external artifacts (ONNX models etc.)
rather than being implemented here.

---

## Javelina-RT (`core/`)

FastDDS-based real-time framework. Replaces ROS2 — not because ROS2 is wrong, but because a framework
that doesn't add software value has no business being a dependency. Javelina provides **comm and lifecycle
only**. Algorithms live outside.

WIP. Base features exist. Grows with ADP8. Gets extracted when it's ready to stand on its own.

### Subsystems

| Folder | What it does |
|--------|-------------|
| `lifecycle/` | Execution engine, task management, parameters, DDS task base |
| `communication/` | FastDDS pub/sub wrappers, topic specs, message queue |
| `support/` | Compile-time utilities: lookup tables, visitor pattern |
| `generators/` | Python codegen + Bazel rules + Jinja2 templates |

### Design Constraints — Non-Negotiable

**Performance.** Light footprint, minimal latency. If something adds cost, it needs a reason —
not a hunch, not a "just in case."

**No preemptive synchronization on parameters.** Currently: statically initialized, read from the same
thread they live in. When a real concurrent use case arrives, we handle it then. Not before.
Don't add mutexes to feel responsible.

**Compile-time over runtime.** Template metaprogramming, FNV-1a hashing, tag dispatch — cost is paid
at compile time. Don't introduce runtime indirection without flagging it.

**Bazel is a hard boundary.** Right tool for multi-language mono-repos. Don't suggest CMake.
Do help write rules, BUILD files, macros.

**Codegen is for boilerplate.** Instantiating typed pub/sub lists from YAML. Output is human-readable
(for debugging). Transparent, not magical. Nobody should need to read the generator to understand
the output.

### Known Tech Debt — Don't Touch Without Asking

- `core/communication/size_constrained_queue.hpp` → outdated, will be removed. Canonical is in
  `core/support/utils/size_constrained_queue.hpp`.
- CI is flaky. Known. Being refactored.
- ROS2 purge in `applications/` is ongoing. An associate is handling it to learn Javelina. Leave it.

---

## How Claude Works Here

The routine things get handled without being asked. For everything else, three modes:

**Doing** — You've designed it, I build it. Boilerplate, tests, BUILD files, scaffold, generators.
No second-guessing the design.

**Advising** — You're deciding something, I give options with tradeoffs. I don't pick for you.
If I think something is wrong I'll say it once, clearly, then move on.

**Researching** — Papers, libraries, benchmarks, state-of-the-art. Relevant to this codebase,
not just generically correct.

---

## What I Won't Do Without Explicit Sign-Off

- Change core architecture
- Add sync primitives where the design intentionally avoids them
- Touch the active ROS2 purge in `applications/`
- Commit or push anything

---

## What I Need From You

- **Design intent when it's not obvious.** One line is enough. If I know why, I won't fight it.
- **Which mode.** If you don't specify, I'll read the room.
- **Pushback.** If I miss the point, say so. I adjust without drama.

---

## Code Style

- C++17/20, headers-only where practical
- Header guards: `CORE_[SUBSYSTEM]_[FILENAME]`
- Namespaces: `core::[subsystem]::`
- Private members: trailing underscore (`running_`, `name_`)
- Constants: `k` prefix (`kFrequency`, `kName`)
- Prefer `std::scoped_lock`, folding expressions, `if constexpr`
- Tests: meaningful over coverage. Smoke tests are valid. Don't test implementation details.

---

## The Point

Make powerful technology accessible. Professionals, hobbyists, companies, people who are just curious.
Code good enough that an expert respects it and a beginner can follow it.

No shortcuts. No theater. No "AI-assisted" badge on a pull request where the human contribution
was approving the diff. Engineers solving problems — and an AI that helps without pretending it doesn't. 🐽
