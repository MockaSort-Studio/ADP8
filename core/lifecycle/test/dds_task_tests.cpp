#include <csignal>
#include <tuple>

#include "core/lifecycle/test/lifecycle_fixture.hpp"

namespace core::lifecycle {

using namespace std::chrono_literals;

TEST_F(LifecycleFixture, SameTopicSpecializedWithDifferentQueue)
{
    using TestSubTrait = std::tuple_element_t<0, TestTopicSubsList>;
    using TestPubTrait = std::tuple_element_t<0, TestTopicPubsList>;

    EXPECT_EQ(TestSubTrait::Name(), TestPubTrait::Name());
    EXPECT_EQ(TestSubTrait::QueueSize(), 2);
}
}  // namespace core::lifecycle