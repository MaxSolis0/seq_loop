#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <time.h>
#include <fstream>
#include <tuple>
#include <vector>
#include <iostream>
#include <mutex> //for clean program exits

struct MsgRecord {
    uint64_t id;
    struct timespec sent_time;
    struct timespec received_time;
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
        clock_gettime(CLOCK_MONOTONIC, &record.sent_time);
        records_.push_back(record);

        std_msgs::msg::UInt64 msg;
        msg.data = counter_++;
        pub_->publish(msg);
    }

    void loop_callback(const std_msgs::msg::UInt64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        //bool found = false;

        // Find the record with the matching ID
        for (auto &record : records_) {
            if (record.id == msg->data) {
                clock_gettime(CLOCK_MONOTONIC, &record.received_time);
                //found = true;
                break;
            }
        }

        /*
        // If no matching record was found, log it with {0,0}
        if (!found) {
            // Create a new record with received_time = 0,0
            records_.push_back({msg->data, {}, {0, 0}});

            std::cerr << "Warning: Message ID " << msg->data 
                      << " not found in existing records. Logging received_time as 0,0.\n";
        }
        */
    }

public:
    // Post-processing function after shutdown
void post_process()
{
    const int64_t PERIOD_NS = 1000000; // 1 ms (1 kHz)

    std::vector<int64_t> send_times;
    std::vector<int64_t> recv_times;
    std::vector<int64_t> rtts;
    std::vector<size_t> missing_ids;  // IDs of messages not received

    for (size_t idx = 0; idx < records_.size(); ++idx) {
        auto &record = records_[idx];

        if (record.received_time.tv_sec == 0 && record.received_time.tv_nsec == 0) {
            // Log missing message
            missing_ids.push_back(record.id);
            continue; // Skip this record for RTT/jitter
        }

        int64_t sent_ns =
            record.sent_time.tv_sec * 1000000000LL + record.sent_time.tv_nsec;

        int64_t recv_ns =
            record.received_time.tv_sec * 1000000000LL + record.received_time.tv_nsec;

        send_times.push_back(sent_ns);
        recv_times.push_back(recv_ns);
        rtts.push_back(recv_ns - sent_ns);
    }

    if (!missing_ids.empty()) {
        std::cout << "Warning: " << missing_ids.size() 
                  << " messages were not received. IDs: ";
        for (auto id : missing_ids) std::cout << id << " ";
        std::cout << "\n";
    }

    // Compute jitter
    std::vector<int64_t> jitter;
    std::vector<int64_t> rtt_jitter;

    for (size_t i = 1; i < send_times.size(); i++) {
        int64_t period = send_times[i] - send_times[i - 1];
        jitter.push_back(period - PERIOD_NS);
    }

    for (size_t i = 1; i < rtts.size(); i++) {
        rtt_jitter.push_back(rtts[i] - rtts[i - 1]);
    }

    // Simple stats function
    auto stats = [](const std::vector<int64_t> &v) {
        if (v.empty()) return std::tuple<int64_t,int64_t,double>(0,0,0.0);
        int64_t min = v[0], max = v[0], sum = 0;
        for (auto x : v) {
            if (x < min) min = x;
            if (x > max) max = x;
            sum += x;
        }
        double avg = (double)sum / v.size();
        return std::tuple<int64_t,int64_t,double>(min,max,avg);
    };

    auto [jmin, jmax, javg] = stats(jitter);
    auto [rmin, rmax, ravg] = stats(rtts);
    auto [rjmin, rjmax, rjavg] = stats(rtt_jitter);

    std::cout << "\nExperiment results\n";
    std::cout << "Messages analyzed: " << rtts.size() 
              << " (out of " << records_.size() << " total)\n\n";

    std::cout << "Send jitter (ns): min=" << jmin << " max=" << jmax << " avg=" << javg << "\n";
    std::cout << "RTT (ns): min=" << rmin << " max=" << rmax << " avg=" << ravg << "\n";
    std::cout << "RTT jitter (ns): min=" << rjmin << " max=" << rjmax << " avg=" << rjavg << "\n";

    // --- Save CSV ---
    std::ofstream file("src/seq_loop/scripts/timing_results.csv");
    file << "id,send_time_ns,recv_time_ns,rtt_ns,send_jitter_ns,rtt_jitter_ns,received\n";

    for (size_t i = 0; i < records_.size(); ++i) {
        auto &record = records_[i];
        int64_t sent_ns = record.sent_time.tv_sec * 1000000000LL + record.sent_time.tv_nsec;
        int64_t recv_ns = record.received_time.tv_sec * 1000000000LL + record.received_time.tv_nsec;
        int64_t rtt = (record.received_time.tv_sec || record.received_time.tv_nsec) ? recv_ns - sent_ns : 0;

        int64_t send_j = 0;
        int64_t rtt_j = 0;

        if (i > 0 && i <= jitter.size()) {
            send_j = jitter[i - 1];
            rtt_j = rtt_jitter[i - 1];
        }

        bool received = !(record.received_time.tv_sec == 0 && record.received_time.tv_nsec == 0);

        file << record.id << ","
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
    std::vector<MsgRecord> records_;
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
