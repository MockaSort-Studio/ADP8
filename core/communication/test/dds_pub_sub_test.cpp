#include <future>
#include <memory>

#include <gtest/gtest.h>

#include "core/communication/dds_context.hpp"
#include "core/communication/dds_publisher.hpp"
#include "core/communication/dds_subscriber.hpp"
#include "TestPubSubTypes.hpp"

using namespace std::chrono_literals;

namespace {
namespace dds = eprosima::fastdds::dds;

class FastDDSPubSubFixture : public ::testing::Test
{
    using DomainParticipantPtr = std::shared_ptr<dds::DomainParticipant>;
    using Pub = core::communication::DDSPublisher<TestPayloadPubSubType>;
    using Sub = core::communication::DDSSubscriber<TestPayloadPubSubType>;

  public:
    FastDDSPubSubFixture()
    {
        pub_ = std::make_unique<Pub>();
        sub_ = std::make_unique<Sub>();

        pub_->Start("TestTopic");
        sub_->Start("TestTopic");
    }
    bool WaitForMatch(std::chrono::milliseconds timeout)
    {
        std::mutex mtx;
        std::condition_variable cv;

        // We wait for the condition: both matched
        std::unique_lock<std::mutex> lock(mtx);
        return cv.wait_for(
            lock, timeout, [this]() { return pub_->IsMatched() && sub_->IsMatched(); });
    }

  protected:
    std::unique_ptr<Pub> pub_;
    std::unique_ptr<Sub> sub_;
};
}  // namespace
TEST_F(FastDDSPubSubFixture, DiscoverySuccessAsync)
{
    std::mutex mtx;
    std::condition_variable cv;

    std::unique_lock<std::mutex> lock(mtx);

    ASSERT_TRUE(WaitForMatch(2s)) << "Discovery timed out after 2 seconds";
}

TEST_F(FastDDSPubSubFixture, AsyncCommunicationTest)
{
    ASSERT_TRUE(WaitForMatch(2s));

    std::promise<TestPayload> promise;
    auto future = promise.get_future();

    std::thread monitor(
        [&]()
        {
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start < 2s)
            {
                auto sample_opt = sub_->GetSample();
                if (sample_opt)
                {
                    // Move the data into the promise
                    promise.set_value(std::move(*sample_opt));
                    return;
                }
                std::this_thread::yield();
            }
        });

    TestPayload msg;
    msg.ok(true);
    pub_->Publish(msg);

    auto status = future.wait_for(2s);
    ASSERT_EQ(status, std::future_status::ready) << "Timed out waiting for data!";

    auto result = future.get();
    EXPECT_TRUE(msg.ok());

    if (monitor.joinable())
        monitor.join();
}