#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mujoco/mujoco.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    // Activate MuJoCo license (only needed for older versions or offline activation)
    // mj_activate("mjkey.txt");

    // Load model from file (adjust the path to your XML model)
    char error[1000] = "Could not load XML model";
    mjModel* m = mj_loadXML("/media/sujan/Work_DIsk/one_week/mujoco_ws/src/mujoco_pubsub/Thirdparty/mujoco/model/car/car.xml", NULL, error, 1000);
    if (!m) {
        printf("Load model error: %s\n", error);
        return 1;
    }

    // Make data structure
    mjData* d = mj_makeData(m);

    // Run a single forward step
    mj_step(m, d);

    // Print base x position as a sanity check
    printf("Base x position: %f\n", d->qpos[0]);

    // Free resources
    mj_deleteData(d);
    mj_deleteModel(m);

    // Deactivate license if needed
    // mj_deactivate();
    
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }