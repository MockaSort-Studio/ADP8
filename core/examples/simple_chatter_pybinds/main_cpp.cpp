// proudly AI-generated, human-reviewed
//
// C++ side of the simple_chatter_pybinds example.
// Publishes on channel_a, subscribes on channel_b.
// Run alongside bazel run //core/examples/simple_chatter_pybinds:py_node
//
// Run:  bazel run //core/examples/simple_chatter_pybinds:cpp_node
// Stop: Ctrl-C

#include "core/examples/simple_chatter_pybinds/node_cpp.hpp"
#include "core/lifecycle/dds_application.hpp"
#include "core/support/utils/lookup_table.hpp"

using namespace core::lifecycle;
using namespace core::utils;

using CppNodeConfig = LookupTable<TableItem<NodeCpp, TaskSpec<100>>>;

int main() {
  DDSAPPlication<CppNodeConfig> app{"simple_chatter_cpp"};
  app.Run();
  return 0;
}
