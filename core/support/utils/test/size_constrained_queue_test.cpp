#include "core/support/utils/size_constrained_queue.hpp"

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

class MessageQueueFixture : public ::testing::Test
{
  protected:
    core::utils::SizeConstrainedQueue<std::string, 2> queue;
};

// 1. Test basic Push and Pop (LIFO behavior)
TEST_F(MessageQueueFixture, LIFOOrdering)
{
    // Testing Rvalue Push (Temporaries)
    queue.Push("First");

    // Testing Lvalue Push (Explicit Move)
    std::string second = "Second";
    queue.Push(std::move(second));

    auto sample = queue.GetSample();
    ASSERT_TRUE(sample.has_value());
    EXPECT_EQ(sample->data, "Second");

    // The original 'second' string should now be empty/unspecified
    // (This proves move semantics were invoked)
    EXPECT_TRUE(second.empty());
}

// 2. Test the "Drop Oldest" logic
TEST_F(MessageQueueFixture, DropOldestWhenFull)
{
    queue.Push("Msg_1");
    queue.Push("Msg_2");
    queue.Push("Msg_3");  // Triggers eviction of Msg_1

    auto s1 = queue.GetSample();
    auto s2 = queue.GetSample();
    auto s3 = queue.GetSample();

    EXPECT_EQ(s1->data, "Msg_3");
    EXPECT_EQ(s2->data, "Msg_2");
    EXPECT_FALSE(s3.has_value());
}

// 3. Test Timestamp Monotonicity
TEST_F(MessageQueueFixture, TimestampOrdering)
{
    queue.Push("Earlier");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    queue.Push("Later");

    auto later = queue.GetSample();
    auto earlier = queue.GetSample();

    EXPECT_GT(later->time_received, earlier->time_received);
}

// 4. Test Queue empty
TEST_F(MessageQueueFixture, EmptyTest)
{
    ASSERT_TRUE(queue.Empty());
    queue.Push("Something");
    ASSERT_FALSE(queue.Empty());

    std::ignore = queue.GetSample();
    ASSERT_TRUE(queue.Empty());
}

// 5.Test Perfect Forwarding with Move-Only Types
TEST(MessageQueueMoveOnly, HandlesUniquePtr)
{
    core::utils::SizeConstrainedQueue<std::unique_ptr<int>, 2> move_queue;

    auto ptr = std::make_unique<int>(100);

    // std::forward allows this to work without attempting a copy
    move_queue.Push(std::move(ptr));

    EXPECT_EQ(ptr, nullptr);  // Resource was stolen

    auto sample = move_queue.GetSample();
    ASSERT_TRUE(sample.has_value());
    EXPECT_EQ(*(sample->data), 100);
}

TEST(TransferToQueueTest, TransferToMovesDataAndResetsSource)
{
    // Arrange: Create two queues of size 5
    // Using a struct to ensure it handles non-primitive types (though POD is better)
    struct TestMsg
    {
        int id;
        double value;
        bool operator==(const TestMsg& other) const { return id == other.id; }
    };

    core::utils::SizeConstrainedQueue<TestMsg, 4> shared_queue;
    core::utils::SizeConstrainedQueue<TestMsg, 4> local_queue;

    shared_queue.Push<TestMsg>({1, 1.1});
    shared_queue.Push<TestMsg>({2, 2.2});
    shared_queue.Push<TestMsg>({3, 3.3});

    ASSERT_EQ(shared_queue.Empty(), false);
    ASSERT_EQ(local_queue.Empty(), true);

    shared_queue.TransferTo(local_queue);

    EXPECT_EQ(shared_queue.Empty(), true);

    EXPECT_EQ(local_queue.Empty(), false);

    // Consistent ordering
    auto s1 = local_queue.GetSample();
    EXPECT_EQ(s1->data.id, 3);
    auto s2 = local_queue.GetSample();
    EXPECT_EQ(s2->data.id, 2);

    auto s3 = local_queue.GetSample();
    EXPECT_EQ(s3->data.id, 1);

    EXPECT_EQ(local_queue.Empty(), true);
}

TEST(QueueViewTest, LogicalIndexingCorrectness)
{
    core::utils::SizeConstrainedQueue<int, 4> queue;

    queue.Push(10);
    queue.Push(20);

    EXPECT_EQ(queue.Size(), 2);
    EXPECT_EQ(queue[0].data, 20);
    EXPECT_EQ(queue[1].data, 10);

    queue.Push(30);
    queue.Push(40);

    EXPECT_EQ(queue.Size(), 4);
    EXPECT_EQ(queue[0].data, 40);
    EXPECT_EQ(queue[1].data, 30);

    queue.Push(50);

    EXPECT_EQ(queue.Size(), 4);

    EXPECT_EQ(queue[0].data, 50);
    EXPECT_EQ(queue[1].data, 40);
    EXPECT_EQ(queue[2].data, 30);
    EXPECT_EQ(queue[3].data, 20);
}

TEST(QueueViewTest, EmptyQueueAssertion)
{
    core::utils::SizeConstrainedQueue<int, 4> queue;

    EXPECT_TRUE(queue.Empty());
    EXPECT_EQ(queue.Size(), 0);
}