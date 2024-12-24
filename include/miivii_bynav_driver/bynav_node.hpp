#ifndef BYNAV_DRIVER__BYNAV_NODE_HPP_
#define BYNAV_DRIVER__BYNAV_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "miivii_bynav_driver/bynav_connection.hpp"
#include "miivii_bynav_driver/bynav_control.hpp"
#include <memory>

namespace miivii_bynav_driver
{

  class BynavNode : public rclcpp::Node
  {
  public:
    explicit BynavNode(const rclcpp::NodeOptions &options);
    ~BynavNode();

  private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

    std::shared_ptr<BynavControl> bynavControl_;
    std::string connection_type_;
    std::string interface_;
    std::string log_port_;
    int serial_baud_rate_;
    int port_;
    int imu_rate_;
    std::string frame_id_;
    BynavConnection::BynavMessageOpts opts_;

    void ConfigureAndCollection();
    void PublishIMUData(const sensor_msgs::msg::Imu &imu_data);
  };

}

#endif // BYNAV_DRIVER__BYNAV_NODE_HPP_
