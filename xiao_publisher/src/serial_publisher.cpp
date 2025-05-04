#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "kondo_msgs/msg/imu_fsr.hpp"


#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include "SerialTransfer.h"
#include <memory>



// #define DEVICENAME "/dev/ttyACM0"
#define BAUDRATE 1000000




class SerialTalker : public rclcpp::Node {
public:
  SerialTalker(const std::string &device_name) : Node("link_node") {
    publisher_ = this->create_publisher<kondo_msgs::msg::ImuFsr>("ImuFsr_data", 10);
    open_serial(device_name);  // Change as needed
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&SerialTalker::read_serial, this));
  }

  ~SerialTalker() {
    if (serial_fd_ != -1) {
      close(serial_fd_);
    }
  }

private:
  int serial_fd_ = -1;
  uint8_t rxBuffer[64];

  int16_t fsr[4];
  int16_t quat[4];
  int16_t gyro[3];
  int16_t acc[3];

  double op_fsr[4];
  double op_quat[4];
  double op_gyro[3];
  double op_acc[3];
  
  rclcpp::Publisher<kondo_msgs::msg::ImuFsr>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<SerialTransfer::SerialTransfer> myTransfer_;


  void open_serial(std::string device_name) {
    const char* DEVICENAME = device_name.c_str();
    myTransfer_ = std::make_unique<SerialTransfer::SerialTransfer>(DEVICENAME, BAUDRATE);
   
  }

  void read_serial() {
    if (myTransfer_->available())
        {
          myTransfer_->rxObj(rxBuffer);

          // Parse buffer
          uint8_t source_id = rxBuffer[0];

          memcpy(fsr,  rxBuffer + 1,   4 * sizeof(int16_t));  // 1–8
          memcpy(quat, rxBuffer + 9,   4 * sizeof(int16_t));  // 9–16
          memcpy(gyro, rxBuffer + 17,  3 * sizeof(int16_t));  // 17–22
          memcpy(acc,  rxBuffer + 23,  3 * sizeof(int16_t));  // 23–28

          // Convert all to double
          for (int i = 0; i < 4; i++)
          {
              op_fsr[i] = static_cast<double>(fsr[i]) / 4096.0;
              op_quat[i] = static_cast<double>(quat[i]) / 16384.0;
          }
          for (int i = 0; i < 3; i++)
          {
              op_gyro[i] = static_cast<double>(gyro[i]) / 16.4;
              op_acc[i] = static_cast<double>(acc[i]) / 4096.0;
          }
        
    
          // // Display
          // printf("ID: %d | FSR: %d %d %d %d\n", source_id, fsr[0], fsr[1], fsr[2], fsr[3]);
          // printf("Quat: %d %d %d %d\n", quat[0], quat[1], quat[2], quat[3]);
          // printf("Gyro: %d %d %d\n", gyro[0], gyro[1], gyro[2]);
          // printf("Acc:  %d %d %d\n\n", acc[0], acc[1], acc[2]);

          auto data = kondo_msgs::msg::ImuFsr();
          
          data.header.stamp = this->now();  // Current ROS time
          data.header.frame_id =std::to_string(source_id);
          
          data.quaternion.x=quat[0];
          data.quaternion.y=quat[1];
          data.quaternion.z=quat[3];
          data.quaternion.w=quat[4];

          data.angular_position.x=gyro[0];
          data.angular_position.y=gyro[1];
          data.angular_position.z=gyro[2];

          data.linear_acceleration.x=acc[0];
          data.linear_acceleration.y=acc[1];
          data.linear_acceleration.z=acc[2];
          
          data.force={fsr[0], fsr[1], fsr[2], fsr[3]};
          
          publisher_->publish(data);


        }

    
    }
};
  
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  if (argc < 2) {
    std::cerr << "Usage: ros2 run your_package your_node_executable <device_name>\n";
    return 1;
  }

  std::string device_name = argv[1];
  rclcpp::spin(std::make_shared<SerialTalker>(device_name));
  rclcpp::shutdown();
  return 0;
}
