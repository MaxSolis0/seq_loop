#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <time.h>
#include <vector>
#include <mutex> //for clean program exits

struct LoopRecord {
    uint64_t id;
    struct timespec recvTime;
};

class Loop05 : public rclcpp::Node
{
public:
    Loop05() : Node("loop05") 
    {}

private:
    void seq_callback(const std_msgs::msg::UInt64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Record receive time
        LoopRecord rec;
        rec.id = msg->data;
        clock_gettime(CLOCK_MONOTONIC, &rec.recvTime);
        records_.push_back(rec);

        // Reply with the same ID
        std_msgs::msg::UInt64 reply;
        reply.data = msg->data;
        pub_->publish(reply);
    }

public:

    void run()
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
        
        pub_ = this->create_publisher<std_msgs::msg::UInt64>("loop05_out", 10);

        sub_ = this->create_subscription<std_msgs::msg::UInt64>(
            "seq05_out", 10,
            std::bind(&Loop05::seq_callback, this, std::placeholders::_1));

        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(shared_from_this());
        exec.spin(); // blocks until rclcpp::shutdown() is called
    }

private:
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_;

    std::vector<LoopRecord> records_;
    std::mutex mutex_;
    
    int depth_;
    std::string history_;
    std::string reliability_;
    std::string durability_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Loop05>();

    node->run();

    rclcpp::shutdown();
    return 0;
}
