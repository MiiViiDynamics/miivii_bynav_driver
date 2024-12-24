#include "miivii_bynav_driver/bynav_node.hpp"

namespace miivii_bynav_driver
{

  BynavNode::BynavNode(const rclcpp::NodeOptions &options)
      : Node("miivii_bynav_node", options)
  {
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

    bynavControl_ = std::make_shared<miivii_bynav_driver::BynavControl>(this->get_logger());

    this->declare_parameter("connection_type", "tcp");
    this->declare_parameter("interface", "192.168.8.151");
    this->declare_parameter("log_port", "ICOM1");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("port", 1111);
    this->declare_parameter("imu_rate", 100);
    this->declare_parameter("frame_id", "imu");

    // this->declare_parameter("connection_type", "serial");
    // this->declare_parameter("interface", "/dev/ttyUART_232_A");
    // this->declare_parameter("log_port", "COM1");
    // this->declare_parameter("baud_rate", 115200);
    // this->declare_parameter("port", 1111);
    // this->declare_parameter("imu_rate", 100);
    // this->declare_parameter("frame_id", "imu");

    // 获取参数值
    connection_type_ = this->get_parameter("connection_type").as_string();
    interface_ = this->get_parameter("interface").as_string();
    log_port_ = this->get_parameter("log_port").as_string();
    serial_baud_rate_ = this->get_parameter("baud_rate").as_int();
    port_ = this->get_parameter("port").as_int();
    imu_rate_ = this->get_parameter("imu_rate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();

    RCLCPP_INFO(this->get_logger(), "Connection Type: %s", connection_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "Interface: %s", interface_.c_str());
    RCLCPP_INFO(this->get_logger(), "LOG Port: %s", log_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud Rate: %d", serial_baud_rate_);
    RCLCPP_INFO(this->get_logger(), "Port: %d", port_);
    RCLCPP_INFO(this->get_logger(), "IMU Rate: %d", imu_rate_);
    RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());

    // opts_["INSPVAXA"] = 0.05;
    opts_["RAWIMUA"] = -1;
    opts_["INSPVAA"] = 0.1;
    opts_["INSSTDEVA"] = 0.1;

    ConfigureAndCollection();
  }

  void BynavNode::ConfigureAndCollection()
  {
    bynavControl_->SetConnectionParams(connection_type_, interface_, log_port_, connection_type_ == "serial" ? serial_baud_rate_ : port_);

    bynavControl_->SetImuRate(imu_rate_);
    bynavControl_->SetFrameId(frame_id_);
    bynavControl_->SetImuDataCallback(std::bind(&BynavNode::PublishIMUData, this, std::placeholders::_1));

    // 启动连接
    try
    {
      bynavControl_->InitializeConnection();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize connection: %s", e.what());
      return;
    }

    // 启动数据处理
    bynavControl_->StartDataProcessingThread(opts_);
  }

  void BynavNode::PublishIMUData(const sensor_msgs::msg::Imu &imu_data)
  {
    imu_publisher_->publish(imu_data);
  }

  BynavNode::~BynavNode()
  {
    bynavControl_->StopDataProcessingThread();
    bynavControl_->DestroyConnection();
  }

} // namespace miivii_bynav_driver

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<miivii_bynav_driver::BynavNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
