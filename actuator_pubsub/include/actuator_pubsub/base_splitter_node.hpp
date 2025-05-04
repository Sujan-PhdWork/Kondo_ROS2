#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "kondo_msgs/msg/state.hpp"
#include "kondo_msgs/msg/feedback.hpp"

class BaseSplitterNode : public rclcpp::Node {
public:
  BaseSplitterNode(
    const std::string & node_name,
    const std::string & output_topic,
    size_t start_index,
    size_t end_index
  )
  : Node(node_name),
    start_index_(start_index),
    end_index_(end_index)
  {
    subscriber_ = this->create_subscription<kondo_msgs::msg::State>(
      "state_topic", 10,
      std::bind(&BaseSplitterNode::callback, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<kondo_msgs::msg::Feedback>(output_topic, 10);
  }

protected:
  void callback(const kondo_msgs::msg::State::SharedPtr msg) {
    if (msg->state.size() < end_index_) {
      RCLCPP_WARN(this->get_logger(), "Position array too short: %ld", msg->state.size());
      return;
    }

    kondo_msgs::msg::Feedback output;
    output.header.stamp = this->now();
    output.position.assign(msg->state.begin() + start_index_, msg->state.begin() + end_index_);
    publisher_->publish(output);  
  }

  rclcpp::Subscription<kondo_msgs::msg::State>::SharedPtr subscriber_;
  rclcpp::Publisher<kondo_msgs::msg::Feedback>::SharedPtr publisher_;
  size_t start_index_, end_index_;
};
