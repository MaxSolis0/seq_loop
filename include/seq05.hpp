//==============================================================================
// Authors : Max Solis, Aleksei Obshatko
// Group : ASDfR 5
// License : LGPL open source license
//
// Brief : Node that controls the relbot based on, coordinate setpoints or 
// tracking an object.
//==============================================================================

#ifndef SEQ05_HPP_
#define SEQ05_HPP_

// for ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <chrono>

// for log file
#include <fstream>
#include <iostream>
#include <time.h>
#include <tuple>
#include <vector>

//for clean program exits
#include <mutex> 

/**
 * @brief Timer and Message generator.
 *
 * ROS2 parameters are described below:
 * @param reliability [string]: Reliability policy for QoS, reliable, keeps sending messages, or best_effort, sends them once. Default: best_effort.
 * @param durability [string]: Durability policy for QoS, volatile, does not store sent messages, transient_local, stores sent messages. Default: transient_local.
 * @param history [string]: History policy for QoS, how many messages to keep, or all. Default: keep_last.
 * @param depth [int]: Depth for history policy for QoS, number of messages kept. Default: 1.
 */

struct MsgRecord {
    uint64_t id;
    struct timespec time;
};

class Seq05 : public rclcpp::Node
{
public:

  /**
   * @brief Construct a new Timer and Message generator object
   *
   * @param maxMessages [int]: The amount of iterations to run the timer.
   */
    Seq05(int maxMessages);
    
    /**
   * @brief Calculate the statistics for timer and communication and save it into a .csv file
   */
    void post_process(); 

private:

  /**
   * @brief Send a unique message every iteration
   */
    void timer_callback();

  /**
   * @brief Log the time and ID of a message when received
   *
   * @param msg: The message received.
   */
    void loop_callback(const std_msgs::msg::UInt64::SharedPtr msg);

    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // program variables
    int counter_;
    int maxMessages_;
    std::vector<MsgRecord> sends_;
    std::vector<MsgRecord> receives_;
    
    // clean code exit
    std::mutex mutex_;
    
    // variables for QoS
    int depth_;
    std::string history_;
    std::string reliability_;
    std::string durability_;
};

#endif /* SEQ05_HPP_ */
