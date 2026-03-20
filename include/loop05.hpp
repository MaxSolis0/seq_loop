//==============================================================================
// Authors : Max Solis, Aleksei Obshatko
// Group : ASDfR 5
// License : LGPL open source license
//
// Brief : Node that controls the relbot based on, coordinate setpoints or 
// tracking an object.
//==============================================================================

#ifndef LOOP05_HPP_
#define LOOP05_HPP_

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

// for clock
#include <time.h>

// for records
#include <vector>

#include <mutex> //for clean program exits

/**
 * @brief Message regurgitator.
 *
 * ROS2 parameters are described below:
 * @param reliability [string]: Reliability policy for QoS, reliable, keeps sending messages, or best_effort, sends them once. Default: best_effort.
 * @param durability [string]: Durability policy for QoS, volatile, does not store sent messages, transient_local, stores sent messages. Default: transient_local.
 * @param history [string]: History policy for QoS, how many messages to keep, or all. Default: keep_last.
 * @param depth [int]: Depth for history policy for QoS, number of messages kept. Default: 1.
 */
 
struct LoopRecord {
    uint64_t id;
    struct timespec recvTime;
};
 
class Loop05 : public rclcpp::Node
{
public:
    
    Loop05();
    
    /**
   * @brief Create and run the node
   */
    void run();

private:
    
    /**
   * @brief For every message received, repeat the same message
   *
   * @param msg: The message to be regurgitated.
   */
    void seq_callback(const std_msgs::msg::UInt64::SharedPtr msg);

    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_;

    // For record
    std::vector<LoopRecord> records_;
    
    // For clean memory exits
    std::mutex mutex_;
    
    // For QoS
    int depth_;
    std::string history_;
    std::string reliability_;
    std::string durability_;
};

#endif /* LOOP05_HPP_ */
