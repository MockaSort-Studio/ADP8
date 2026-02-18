#include <chrono>
#include <cstdint>
#include <thread>

#include "core/communication/dds_subscriber.hpp"
#include "sorcyPubSubTypes.hpp"

template <typename Porcoddio>
class HelloWorldSubscriber
{
  private:
    using DioStracane = core::communication::DDSSubscriber<Porcoddio>;
    // app context
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;
    // this must be unique
    std::shared_ptr<DioStracane> sub_;

  public:
    HelloWorldSubscriber() = default;

    virtual ~HelloWorldSubscriber()
    {
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
            ->delete_participant(participant_.get());
    }

    //! Initialize the subscriber
    bool init()
    {
        eprosima::fastdds::dds::DomainParticipantQos participantQos;
        participantQos.name("Participant_subscriber");
        std::cout << "DioSerpente" << std::endl;

        participant_ = std::shared_ptr<eprosima::fastdds::dds::DomainParticipant>(
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
                ->create_participant(0, participantQos));
        std::cout << "DioSerpente" << std::endl;

        if (participant_.get() == nullptr)
        {
            return false;
        }
        std::cout << "DioSerpente" << std::endl;

        sub_ = std::shared_ptr<DioStracane>(
            new DioStracane("SorcyPortyTopic", participant_));

        if (sub_.get() == nullptr)
        {
            return false;
        }
        return true;
    }

    //! Run the Subscriber
    void run(uint32_t samples)
    {
        std::thread::id this_id = std::this_thread::get_id();

        std::cout << "Main Thread ID: " << this_id << std::endl;
        while (sub_->NumberOfMessagesReceived() < samples)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

int main(int argc, char** argv)
{
    std::cout << "Starting subscriber." << std::endl;
    uint32_t samples = 10;

    auto* mysub = new HelloWorldSubscriber<SorcyPortyPubSubType>();
    std::cout << "Diolupo" << std::endl;

    if (mysub->init())
    {
        std::cout << "DioSerpente" << std::endl;
        mysub->run(samples);
    }
    return 0;
}