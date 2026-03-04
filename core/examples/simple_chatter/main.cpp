// Two nodes chatting over two channels:
//   NodeAlpha  pub -> channel_a -> NodeBeta
//   NodeAlpha  sub <- channel_b <- NodeBeta
//
// Run with:  bazel run //core/examples/simple_chatter:simple_chatter
// Stop with: Ctrl-C

#include "core/examples/simple_chatter/node_alpha.hpp"
#include "core/examples/simple_chatter/node_beta.hpp"
#include "core/lifecycle/dds_application.hpp"
#include "core/support/utils/lookup_table.hpp"

using namespace core::lifecycle;
using namespace core::utils;

using ChatterConfig =
    LookupTable<TableItem<simple_chatter::NodeAlpha, TaskSpec<100>>,
                TableItem<simple_chatter::NodeBeta, TaskSpec<100>>>;

int main() {
  DDSAPPlication<ChatterConfig> app{"simple_chatter"};
  app.Run();
  return 0;
}
