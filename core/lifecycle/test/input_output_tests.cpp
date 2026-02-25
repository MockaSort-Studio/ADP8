#include <tuple>

#include "core/lifecycle/input.hpp"
#include "core/lifecycle/output.hpp"
#include "core/lifecycle/test/lifecycle_fixture.hpp"

namespace core::lifecycle {

using TestInputs = Inputs_t<TestTopicSubsList>;
using TestOutputs = Outputs_t<TestTopicPubsList>;

TEST(TestHash, ExpectSuccess)
{
    constexpr uint64_t hash_1 = Hash(kTestTopicName);
    constexpr uint64_t hash_2 = Hash(kTestTopicName);
    constexpr uint64_t hash_3 = Hash(kDifferentTopicName);

    EXPECT_EQ(hash_1, hash_2);
    EXPECT_NE(hash_1, hash_3);
}

TEST(TestInputOutput, SameTypeSpecializedWithDifferentTopic)
{
    TestInputs inputs;
    TestOutputs outputs;

    auto& input_1 = get<kTestTopicName>(inputs);
    auto& input_2 = get<kDifferentTopicName>(inputs);
    auto& output_1 = get<kTestTopicName>(outputs);
    auto& output_2 = get<kDifferentTopicName>(outputs);

    EXPECT_EQ(typeid(input_1), typeid(std::tuple_element_t<0, TestInputs>));
    EXPECT_EQ(typeid(input_2), typeid(std::tuple_element_t<1, TestInputs>));
    EXPECT_NE(typeid(input_1), typeid(input_2));
    EXPECT_EQ(input_1.kHash, Hash(kTestTopicName));
    EXPECT_EQ(input_2.kHash, Hash(kDifferentTopicName));
    EXPECT_EQ(input_1.kQueueSize, 2);
    EXPECT_EQ(input_2.kQueueSize, 2);

    EXPECT_EQ(typeid(output_1), typeid(std::tuple_element_t<0, TestOutputs>));
    EXPECT_EQ(typeid(output_2), typeid(std::tuple_element_t<1, TestOutputs>));
    EXPECT_NE(typeid(output_1), typeid(output_2));
    EXPECT_EQ(output_1.kHash, Hash(kTestTopicName));
    EXPECT_EQ(output_2.kHash, Hash(kDifferentTopicName));
}

}  // namespace core::lifecycle