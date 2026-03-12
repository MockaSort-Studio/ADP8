// proudly AI-generated, human-reviewed
//
// C++ smoke tests for PyDDSBridge<SubscriptionSpecs, PublicationSpecs>.
// No peer node — covers construction, FillInputs/FlushOutputs, and
// empty reads. Integration with a live peer lives in the examples.

#include <gtest/gtest.h>

#include "core/pybinds/py_dds_bridge.hpp"
#include "ping_ports_publications.hpp"
#include "ping_ports_subscriptions.hpp"

using Bridge = PyDDSBridge<ping::Subscriptions, ping::Publications>;

namespace {

TEST(PyDDSBridgeTest, ParticipantNameMatchesConstructorArg) {
  Bridge bridge{"py_dds_bridge_cpp_test"};
  EXPECT_EQ(bridge.ParticipantName(), "py_dds_bridge_cpp_test");
}

TEST(PyDDSBridgeTest, FillInputsNoPeer) {
  Bridge bridge{"py_dds_bridge_cpp_test"};
  EXPECT_NO_THROW(bridge.FillInputs());
}

TEST(PyDDSBridgeTest, GetInputsReturnsEmptyWithoutPeer) {
  Bridge bridge{"py_dds_bridge_cpp_test"};
  bridge.FillInputs();

  auto samples = bridge.GetInputs<ping::kPingInTopicName>();

  EXPECT_TRUE(samples.empty());
}

TEST(PyDDSBridgeTest, PushAndFlushNoPeer) {
  Bridge bridge{"py_dds_bridge_cpp_test"};

  PingMessage msg;
  msg.content("orphan");
  msg.id(0);

  EXPECT_NO_THROW({
    bridge.PushOutput<ping::kPingOutTopicName>(std::move(msg));
    bridge.FlushOutputs();
  });
}

}  // namespace
