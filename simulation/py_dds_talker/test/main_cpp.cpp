// proudly AI-generated, human-reviewed
//
// C++ side of the py_dds_talker demo.
// Publishes on channel_a, subscribes on channel_b.
// Run alongside bazel run //simulation/py_dds_talker/test:py_node
//
// Run:  bazel run //simulation/py_dds_talker/test:cpp_node
// Stop: Ctrl-C

#include <cstdlib>
#include <string>

#include "core/lifecycle/dds_application.hpp"
#include "core/support/utils/lookup_table.hpp"
#include "simulation/py_dds_talker/test/node_cpp.hpp"

using namespace core::lifecycle;
using namespace core::utils;

using CppNodeConfig =
    LookupTable<TableItem<py_dds_talker::NodeCpp, TaskSpec<100>>>;

int main() {
  // Load FastDDS loopback profile before any DDS initialization.
  // BUILD_WORKSPACE_DIRECTORY is set by bazel run.
  if (const char* ws = std::getenv("BUILD_WORKSPACE_DIRECTORY")) {
    const std::string xml_path =
        std::string(ws) + "/simulation/py_dds_talker/test/fastdds_sim.xml";
    ::setenv("FASTRTPS_DEFAULT_PROFILES_FILE", xml_path.c_str(), /*overwrite=*/0);
  }

  DDSAPPlication<CppNodeConfig> app{"py_dds_talker_cpp"};
  app.Run();
  return 0;
}
