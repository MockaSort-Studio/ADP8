#include <gtest/gtest.h>

#include "core/communication/topic_spec.hpp"
#include "TestPayloadPubSubTypes.hpp"

namespace core::communication {

inline constexpr char kTestName[] = "Name";

TEST(TopicSpecTrait, ExpectedTrueForValidTopicSpec)
{
    using ValidSpec = TopicSpec<TestPayloadPubSubType, kTestName, 10>;
    EXPECT_TRUE(is_topic_spec_v<ValidSpec>);
}

TEST(TopicSpecTrait, ExpectedFalseForInvalidTopicSpec)
{
    using InvalidSpec = int;
    EXPECT_FALSE(is_topic_spec_v<InvalidSpec>);
}
}  // namespace core::communication
