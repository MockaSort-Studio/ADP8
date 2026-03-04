# Support

`core/support/` — Compile-time utilities used throughout Javelina-RT and application code.

---

## LookupTable

`core/support/utils/lookup_table.hpp` · namespace `core::utils`

A compile-time heterogeneous map. Keys are types; values are tuples of values. Used for `ApplicationConfig`, `ParametersProvider`, and anything else that needs typed key-value storage resolved at compile time.

### Structure

```cpp
// One entry in the table
template <typename KeyType, typename... ValueTypes>
struct TableItem {
  using Key = KeyType;
  using Values = std::tuple<ValueTypes...>;
  Values values;
};

// The table itself — inherits from all its items
template <typename... TableElements>
struct LookupTable : public TableElements... { ... };
```

Duplicate keys are caught by a `static_assert` at instantiation.

### Basic usage

```cpp
struct FreqTag {};
struct GainTag {};

using MyTable = core::utils::LookupTable<
    core::utils::TableItem<FreqTag, int>,
    core::utils::TableItem<GainTag, float, float>  // two floats
>;

MyTable table(std::make_tuple(
    std::make_tuple(100),
    std::make_tuple(1.5f, 0.5f)
));

auto freq = table.GetValue<FreqTag>();        // std::tuple<int>
auto gains = table.GetValue<GainTag>();       // std::tuple<float, float>
```

### Tag dispatch (for complex initialization)

When the table has many entries or heavily templated types, `Init` + `TableDefaults` avoids positional initialization errors:

```cpp
auto defaults = core::utils::TableDefaults<MyTable>(
    core::utils::Init{FreqTag{}, 100},
    core::utils::Init{GainTag{}, 1.5f, 0.5f}
);
MyTable table(defaults);
```

Order doesn't matter — each `Init` carries its key type.

### Iteration

```cpp
MyTable::for_each([](auto element) {
    using Key = typename decltype(element)::Key;
    // process element...
});
```

Used by `DDSAPPlication::BuildTaskManager` to register all tasks from the config table.

---

## SizeConstrainedQueue

`core/support/utils/size_constrained_queue.hpp` · namespace `core::utils`

Fixed-capacity circular buffer with timestamps. The canonical queue used by `DataEndpoint` and anywhere samples with time metadata are needed.

### Sample

```cpp
template <typename T>
struct Sample {
  Timestamp time_received;  // std::chrono::steady_clock time point
  T data;
};
```

### Queue

```cpp
template <typename T, std::size_t N>
class SizeConstrainedQueue;
```

- `Push(T&&)` — inserts at head, overwrites oldest if full. Thread-safe via `std::scoped_lock`.
- `operator[](size_t i)` — reverse-chronological: `[0]` is newest, `[N-1]` is oldest.
- `GetSample()` — pops and returns the newest sample as `std::optional<Sample<T>>`. Returns `nullopt` if empty.
- `TransferTo(other)` — moves all contents to another queue atomically. Used by `DDSSubscriber::DrainQueue`.
- `Size()`, `Empty()`, `capacity()` — non-mutating accessors.

`N` is checked `> 0` at compile time.

---

## Visitor

`core/support/visit/` · namespace `core::visit`

A CRTP-based visitor pattern for traversing struct members by name. Useful for serialization, logging, or any operation that needs to iterate over a struct's fields without knowing them at the visitor's call site.

### Three pieces

**`Property<Class, MemberType>`** (`property.hpp`) — binds a member pointer to a name:

```cpp
auto prop = core::visit::property("x", &Vec3::x);
prop.name_;             // "x"
prop.Get(vec_instance); // vec_instance.x
```

**`Visitable<InstanceT, Properties...>`** (`visitable.hpp`) — holds a tuple of properties and dispatches them to a visitor:

```cpp
auto v = core::visit::visitable<Vec3>(
    core::visit::property("x", &Vec3::x),
    core::visit::property("y", &Vec3::y),
    core::visit::property("z", &Vec3::z)
);
v.accept(my_visitor, vec_instance);
```

Nested visitables are handled automatically — if a member type exposes `GetVisitable()`, the visitor's `visit_nested`/`exit_nested` hooks are called recursively.

**`VisitorBase<Derived>`** (`visitor_base.hpp`) — CRTP base for visitor implementations. Derived classes implement `visit(name, value)` and optionally `visit_nested(name)` / `exit_nested(name)`:

```cpp
class PrintVisitor : public core::visit::VisitorBase<PrintVisitor> {
 public:
  template <typename T>
  void visit(std::string_view name, T& value) {
    std::cout << name << " = " << value << "\n";
  }
};
```

### Macro shorthand

`property.hpp` provides macros to define `GetVisitable()` inside a struct:

```cpp
struct Pose {
  float x, y, yaw;

  BEGIN_VISITABLE(Pose)
      ADD_PROPERTY_BEGIN(Pose, x)
      ADD_PROPERTY(y)
      ADD_PROPERTY_END(yaw)
};
```

These expand to a `static constexpr auto GetVisitable()` that returns the `Visitable` object. The macros are syntactic sugar — the underlying mechanism is the same.

### Type traits

- `is_i_visitable_v<T>` — true if `T` derives from `IVisitable`
- `has_get_visitable<T>` — true if `T` has a static `GetVisitable()` returning an `IVisitable`
- `has_visit_nested_v<T>`, `has_exit_nested_v<T>` — checked at compile time to decide whether to call nested hooks

---

```cpp
// Hamlet 🐗 — if constexpr, therefore I am
```
