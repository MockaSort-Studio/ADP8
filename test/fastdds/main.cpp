#include <memory>
#include <thread>

#include "core/communication/dds_publisher.hpp"
#include "sorcyPubSubTypes.hpp"

template <typename Porcoddio>
class HelloWorldPublisher
{
  private:
    using DioStraporco = core::communication::DDSPublisher<Porcoddio>;
    // just for the sake of the experiment
    typename Porcoddio::type hello_;

    // app context
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;
    // this must be unique
    std::shared_ptr<DioStraporco> pub_;

  public:
    HelloWorldPublisher() = default;

    virtual ~HelloWorldPublisher()
    {
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
            ->delete_participant(participant_.get());
    }

    //! Initialize the publisher
    bool init()
    {
        hello_.index(0);
        hello_.message("Porcoddio");

        eprosima::fastdds::dds::DomainParticipantQos participantQos;
        participantQos.name("Participant_publisher");
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

        pub_ = std::shared_ptr<DioStraporco>(
            new DioStraporco("SorcyPortyTopic", participant_));

        if (pub_.get() == nullptr)
        {
            return false;
        }
        return true;
    }

    //! Send a publication
    bool publish()
    {
        hello_.index(hello_.index() + 1);
        return pub_->Publish(hello_);
    }

    void run(uint32_t samples)
    {
        uint32_t samples_sent = 0;
        while (samples_sent < samples)
        {
            if (publish())
            {
                samples_sent++;
                std::cout << "Message: " << hello_.message()
                          << " with index: " << hello_.index() << " SENT" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
};

int main(int argc, char** argv)
{
    std::cout << "Starting publisher." << std::endl;
    uint32_t samples = 10;

    auto* mypub = new HelloWorldPublisher<SorcyPortyPubSubType>();
    std::cout << "Diolupo" << std::endl;

    if (mypub->init())
    {
        std::cout << "DioSerpente" << std::endl;
        mypub->run(samples);
    }

    return 0;
}