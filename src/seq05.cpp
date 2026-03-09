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
    Seq05(int max_messages)
    : Node("seq05"), counter_(0), max_messages_(max_messages)
    {
        // Declare parameters
        reliability_ = this->declare_parameter("reliability", "reliable");
        durability_ = this->declare_parameter("durability", "volatile");
        history_ = this->declare_parameter("history", "keep_last");
        depth_ = this->declare_parameter("depth", 10);  
        
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

        if (counter_ >= max_messages_) {
            // Stop the experiment after max_messages_
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
        const int64_t PERIOD_NS = 1000000; // 1 ms (1 kHz)

        std::vector<int64_t> send_times;
        std::vector<int64_t> recv_times;
        std::vector<int64_t> rtts;
        std::vector<int64_t> send_jitters;
        std::vector<int64_t> rtt_jitters;
        std::vector<size_t> missing_ids;  // IDs of messages not received

        size_t sent_count = sends_.size();
        size_t received_count = 0;

        // Process send and receive data
        for (size_t i = 0; i < sends_.size(); ++i) {
            auto &send_record = sends_[i];
            bool received = false;
            int64_t sent_ns = send_record.time.tv_sec * 1000000000LL + send_record.time.tv_nsec;
            send_times.push_back(sent_ns);
            
            // Calculate send jitter
            if (i > 0) {
                int64_t prev_sent_ns = send_times[i - 1];
                send_jitters.push_back(sent_ns - prev_sent_ns - PERIOD_NS);
            } else {
                send_jitters.push_back(0); // No jitter for the first message
            }

            // Try to find the corresponding received record by id
            for (size_t j = 0; j < receives_.size(); ++j) {
                auto &recv_record = receives_[j];
                if (recv_record.id == send_record.id) {
                    received = true;
                    received_count++;
                    int64_t recv_ns = recv_record.time.tv_sec * 1000000000LL + recv_record.time.tv_nsec;
                    recv_times.push_back(recv_ns);
                  
                    rtts.push_back(recv_ns - sent_ns);

                    // Calculate round trip time jitter
                    if (i > 0) {
                        rtt_jitters.push_back(rtts[i] - rtts[i - 1]);
                    } else {
                        rtt_jitters.push_back(0); // No RTT jitter for the first message
                    }

                    break; // Exit the loop once a match is found
                }
            }

            if (!received) {
                // If no receive record is found, log the missing message ID
                missing_ids.push_back(send_record.id);
                recv_times.push_back(-1); // Log -1 as receive time for missing messages
                rtts.push_back(-1);
                rtt_jitters.push_back(-1);
            }
        }

        // Output missing message info
        if (!missing_ids.empty()) {
            std::cout << "Warning: " << missing_ids.size() 
                      << " messages were not received. IDs: ";
            for (auto id : missing_ids) std::cout << id << " ";
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
        auto [send_jitter_min, send_jitter_max, send_jitter_avg] = stats(send_jitters);
        auto [rtt_min, rtt_max, rtt_avg] = stats(rtts);
        auto [rtt_jitter_min, rtt_jitter_max, rtt_jitter_avg] = stats(rtt_jitters);

        std::cout << "\nExperiment results\n";
        std::cout << "Messages analyzed: " << received_count 
                  << " (out of " << sent_count << " sent)\n\n";

        std::cout << "Send jitter (ns): min=" << send_jitter_min << " max=" << send_jitter_max << " avg=" << send_jitter_avg << "\n";
        std::cout << "RTT (ns): min=" << rtt_min << " max=" << rtt_max << " avg=" << rtt_avg << "\n";
        std::cout << "RTT jitter (ns): min=" << rtt_jitter_min << " max=" << rtt_jitter_max << " avg=" << rtt_jitter_avg << "\n";

        // --- Save CSV ---
        std::ofstream file("src/seq_loop/scripts/timing_results.csv");
        file << "id,send_time_ns,recv_time_ns,rtt_ns,send_jitter_ns,rtt_jitter_ns,received\n";

        for (size_t i = 0; i < sends_.size(); ++i) {
            auto &send_record = sends_[i];
            int64_t sent_ns = send_times[i];
            int64_t recv_ns = recv_times[i];
            int64_t rtt = rtts[i];
            int64_t send_j = send_jitters[i];
            int64_t rtt_j = rtt_jitters[i];
            bool received = (recv_ns != -1);

            file << send_record.id << ","
                 << sent_ns << ","
                 << recv_ns << ","
                 << rtt << ","
                 << send_j << ","
                 << rtt_j << ","
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
    int max_messages_;
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
