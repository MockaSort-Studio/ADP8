#include "core/communication/size_constrained_queue.hpp"

#include <memory>  // for std::unique_ptr
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

class MessageQueueFixture : public ::testing::Test
{
  protected:
    core::communication::SizeConstrainedQueue<std::string, 3> queue;
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
    queue.Push("Msg_3");
    queue.Push("Msg_4");  // Triggers eviction of Msg_1

    auto s1 = queue.GetSample();
    auto s2 = queue.GetSample();
    auto s3 = queue.GetSample();
    auto s4 = queue.GetSample();

    EXPECT_EQ(s1->data, "Msg_4");
    EXPECT_EQ(s2->data, "Msg_3");
    EXPECT_EQ(s3->data, "Msg_2");
    EXPECT_FALSE(s4.has_value());
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
    core::communication::SizeConstrainedQueue<std::unique_ptr<int>, 2> move_queue;

    auto ptr = std::make_unique<int>(100);

    // std::forward allows this to work without attempting a copy
    move_queue.Push(std::move(ptr));

    EXPECT_EQ(ptr, nullptr);  // Resource was stolen

    auto sample = move_queue.GetSample();
    ASSERT_TRUE(sample.has_value());
    EXPECT_EQ(*(sample->data), 100);
}