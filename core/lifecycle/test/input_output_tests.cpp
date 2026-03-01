

#include "core/lifecycle/test/lifecycle_fixture.hpp"
namespace core::lifecycle {
using namespace std::chrono_literals;

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

TEST(TestInputOutput, TestSetGet)
{
    TestInputs inputs;
    TestOutputs outputs;

    auto& producer = get<kTestTopicName>(outputs);
    auto& consumer = get<kTestTopicName>(inputs);

    TestPayload payload;
    payload.ok(true);
    producer.Push(std::move(payload));
    producer.Sync();

    std::this_thread::sleep_for(20ms);

    consumer.Sync();
    auto response = consumer.Get();

    ASSERT_TRUE(response.has_value());
    EXPECT_TRUE(response.value().data.ok());
}

TEST(TestInputOutput, TestGetByQueueIndex)
{
    TestInputs inputs;
    TestOutputs outputs;

    auto& producer = get<kTestTopicName>(outputs);
    auto& consumer = get<kTestTopicName>(inputs);

    TestPayload payload;
    payload.ok(true);
    producer.Push(std::move(payload));
    producer.Sync();

    std::this_thread::sleep_for(20ms);

    consumer.Sync();
    auto response = consumer[0];

    EXPECT_TRUE(response.data.ok());
}

TEST(TestInputOutput, TestSinkAndSourceView)
{
    TestInputs inputs;
    TestOutputs outputs;

    auto& producer = get<kTestTopicName>(outputs);
    auto& consumer = get<kTestTopicName>(inputs);

    auto read_only_view = InputSource {consumer};
    auto write_only_view = OutputSink {producer};

    TestPayload payload;
    payload.ok(true);

    write_only_view.Push(std::move(payload));
    producer.Sync();

    std::this_thread::sleep_for(20ms);

    consumer.Sync();
    auto response = read_only_view[0];

    EXPECT_TRUE(response.data.ok());
}

}  // namespace core::lifecycle