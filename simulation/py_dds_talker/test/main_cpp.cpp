// proudly AI-generated, human-reviewed
//
// C++ side of the py_dds_talker demo.
// Publishes on channel_a, subscribes on channel_b.
// Run alongside bazel run //simulation/py_dds_talker:py_node
//
// Run:  bazel run //simulation/py_dds_talker:cpp_node
// Stop: Ctrl-C

#include "core/lifecycle/dds_application.hpp"
#include "core/support/utils/lookup_table.hpp"
#include "simulation/py_dds_talker/test/node_cpp.hpp"

using namespace core::lifecycle;
using namespace core::utils;

using CppNodeConfig =
    LookupTable<TableItem<py_dds_talker::NodeCpp, TaskSpec<100>>>;

int main() {
  DDSAPPlication<CppNodeConfig> app{"py_dds_talker_cpp"};
  app.Run();
  return 0;
}
