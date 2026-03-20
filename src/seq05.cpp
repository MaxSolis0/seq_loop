#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <time.h>
#include <fstream>
#include <tuple>
#include <chrono>
#include <vector>
#include <iostream>
#include <mutex> //for clean program exits

struct MsgRecord {
    uint64_t id;
    struct timespec time;
};

class Seq05 : public rclcpp::Node
{
public:
    Seq05(int maxMessages)
    : Node("seq05"), counter_(0), maxMessages_(maxMessages)
    {
        // Declare parameters
        reliability_ = this->declare_parameter("reliability", "reliable");
        durability_ = this->declare_parameter("durability", "volatile");
        history_ = this->declare_parameter("history", "keep_last");
        depth_ = this->declare_parameter("depth", 1);  
        
        // Get parameter initial values
        reliability_ = this->get_parameter("reliability").as_string();
        durability_ = this->get_parameter("durability").as_string();
        history_ = this->get_parameter("history").as_string();
        depth_ = this->get_parameter("depth").as_int();
        
        // Create a object of type rclcpp:qos which contains RMW profile that can be used for publishers and subscribers
        auto qos = rclcpp::QoS(depth_); // queue depth of=based on depth parameter
        reliability_ == "best_effort" ? qos.best_effort() : qos.reliable();
        durability_ == "transient_local" ? qos.transient_local() : qos.durability_volatile();
        history_ == "keep_all" ? qos.keep_all() : qos.keep_last(depth_);
        
        pub_ = this->create_publisher<std_msgs::msg::UInt64>("seq05_out", qos);

        sub_ = this->create_subscription<std_msgs::msg::UInt64>(
            "loop05_out", qos,
            std::bind(&Seq05::loop_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000), // 1 kHz
            std::bind(&Seq05::timer_callback, this));
    }

private:

    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (counter_ >= maxMessages_) {
            // Stop the experiment after maxMessages_
            rclcpp::shutdown();
            return;
        }

        MsgRecord record;
        record.id = counter_;
        clock_gettime(CLOCK_MONOTONIC, &record.time);
        sends_.push_back(record);

        std_msgs::msg::UInt64 msg;
        msg.data = counter_++;
        pub_->publish(msg);
    }

    void loop_callback(const std_msgs::msg::UInt64::SharedPtr msg)
    {
        MsgRecord record;
        record.id = msg->data;
        clock_gettime(CLOCK_MONOTONIC, &record.time);
        receives_.push_back(record);
    }

public:
    // Post-processing function after shutdown
    void post_process()
    {
        const int64_t periodNs = 1000000; // 1 ms (1 kHz)

        std::vector<int64_t> sendTimes;
        std::vector<int64_t> recvTimes;
        std::vector<int64_t> rtts;
        std::vector<int64_t> sendJitters;
        std::vector<int64_t> rttJitters;
        std::vector<size_t> missingIDs;  // IDs of messages not received

        size_t sentCount = sends_.size();
        size_t receivedCount = 0;

        // Process send and receive data
        for (size_t i = 0; i < sends_.size(); ++i) {
            auto &sendRecord = sends_[i];
            bool received = false;
            int64_t sentNs = sendRecord.time.tv_sec * 1000000000LL + sendRecord.time.tv_nsec;
            sendTimes.push_back(sentNs);
            
            // Calculate send jitter
            if (i > 0) {
                int64_t prevSentNs = sendTimes[i - 1];
                sendJitters.push_back(sentNs - prevSentNs - periodNs);
            } else {
                sendJitters.push_back(0); // No jitter for the first message
            }

            // Try to find the corresponding received record by id
            for (size_t j = 0; j < receives_.size(); ++j) {
                auto &recvRecord = receives_[j];
                if (recvRecord.id == sendRecord.id) {
                    received = true;
                    receivedCount++;
                    int64_t recv_ns = recvRecord.time.tv_sec * 1000000000LL + recvRecord.time.tv_nsec;
                    recvTimes.push_back(recv_ns);
                  
                    rtts.push_back(recv_ns - sentNs);

                    // Calculate round trip time jitter
                    if (i > 0) {
                        rttJitters.push_back(rtts[i] - rtts[i - 1]);
                    } else {
                        rttJitters.push_back(0); // No RTT jitter for the first message
                    }

                    break; // Exit the loop once a match is found
                }
            }

            if (!received) {
                // If no receive record is found, log the missing message ID
                missingIDs.push_back(sendRecord.id);
                recvTimes.push_back(-1); // Log -1 as receive time for missing messages
                rtts.push_back(-1);
                rttJitters.push_back(-1);
            }
        }

        // Output missing message info
        if (!missingIDs.empty()) {
            std::cout << "Warning: " << missingIDs.size() 
                      << " messages were not received. IDs: ";
            for (auto id : missingIDs) std::cout << id << " ";
            std::cout << "\n";
        }

        // Simple stats function for min, max, avg
        auto stats = [](const std::vector<int64_t> &v) {
            if (v.empty()) return std::tuple<int64_t, int64_t, double>(0, 0, 0.0);
            int64_t min = v[0], max = v[0], sum = 0;
            for (auto x : v) {
                if (x < min) min = x;
                if (x > max) max = x;
                sum += x;
            }
            double avg = (double)sum / v.size();
            return std::tuple<int64_t, int64_t, double>(min, max, avg);
        };

        // Compute stats for jitter and RTT
        auto [sendJitterMin, sendJitterMax, sendJitterAvg] = stats(sendJitters);
        auto [rttMin, rttMax, rttAvg] = stats(rtts);
        auto [rttJitterMin, rttJitterMax, rttJitterAvg] = stats(rttJitters);

        std::cout << "\nExperiment results\n";
        std::cout << "Messages analyzed: " << receivedCount 
                  << " (out of " << sentCount << " sent)\n\n";

        std::cout << "Send jitter (ns): min=" << sendJitterMin << " max=" << sendJitterMax << " avg=" << sendJitterAvg << "\n";
        std::cout << "RTT (ns): min=" << rttMin << " max=" << rttMax << " avg=" << rttAvg << "\n";
        std::cout << "RTT jitter (ns): min=" << rttJitterMin << " max=" << rttJitterMax << " avg=" << rttJitterAvg << "\n";

        // --- Save CSV ---
        std::ofstream file("src/seq_loop/scripts/timing_results.csv");
        file << "id,send_time_ns,recv_time_ns,rtt_ns,send_jitter_ns,rtt_jitter_ns,received\n";

        for (size_t i = 0; i < sends_.size(); ++i) {
            auto &sendRecord = sends_[i];
            int64_t sentNs = sendTimes[i];
            int64_t recv_ns = recvTimes[i];
            int64_t rtt = rtts[i];
            int64_t sendJ = sendJitters[i];
            int64_t rttJ = rttJitters[i];
            bool received = (recv_ns != -1);

            file << sendRecord.id << ","
                 << sentNs << ","
                 << recv_ns << ","
                 << rtt << ","
                 << sendJ << ","
                 << rttJ << ","
                 << received << "\n";
        }

        file.close();
        std::cout << "Results written to timing_results.csv\n";
    }                 

private:
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int counter_;
    int maxMessages_;
    std::vector<MsgRecord> sends_;
    std::vector<MsgRecord> receives_;
    
    std::mutex mutex_;
    
    int depth_;
    std::string history_;
    std::string reliability_;
    std::string durability_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Run experiment for N messages
    int N = 5000; // e.g., 5000 messages = 5 seconds at 1 kHz
    
    auto node = std::make_shared<Seq05>(N);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin(); // blocks until rclcpp::shutdown() is called

    // Post-processing after experiment
    node->post_process();

    rclcpp::shutdown();
    return 0;
}
