#ifndef BYNAV_CONTROL_HPP_
#define BYNAV_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "miivii_bynav_driver/bynav_connection.hpp"
#include "miivii_bynav_driver/parsers/message_parser.hpp"
#include "miivii_bynav_driver/parsers/parser_utils.hpp"
#include <thread>
#include <queue>
#include <mutex>
#include <memory>

#include <boost/circular_buffer.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
namespace miivii_bynav_driver
{

    class BynavControl
    {
    public:
        using ImuDataCallback = std::function<void(const sensor_msgs::msg::Imu &)>;

        BynavControl(rclcpp::Logger logger);
        ~BynavControl();

        void SetImuRate(int rate);
        void SetConnectionParams(std::string type, std::string param1, std::string param2, int param3);
        void InitializeConnection();
        void DestroyConnection();
        void StartDataProcessingThread(BynavConnection::BynavMessageOpts const &opts);
        void StopDataProcessingThread();

        void SetImuDataCallback(const ImuDataCallback &callback);
        void SetFrameId(std::string const &frame_id);

    private:
        rclcpp::Logger logger_;
        std::mutex queue_mutex_;
        std::condition_variable queue_cond_;
        bool processing_thread_active_ = false;
        std::thread processing_thread_;

        BynavConnection bynavConnection_;
        BynavConnection::BynavMessageOpts opts;
        ParserUtils parserUtils_;

        int imu_rate_;
        std::string connection_type_;
        std::string frame_id_;

        ImuDataCallback imu_data_callback_;

        void ProcessData();
        void AnalysisData(const std::string &data);

        static constexpr uint32_t GPS_UTC_DIFF = 315964800 - 18; // 其中18秒是截止到2024年的润秒
        static constexpr uint32_t SECONDS_PER_WEEK = 604800;     // 一周秒数
        static constexpr double IMU_TOLERANCE_S = 0.01;          // 时间差容忍度
        static constexpr double DEGREES_TO_RADIANS = M_PI / 180.0;

        const double GYRO_LSB_TO_DEG_S = 3.35276126861572e-07; // Gyro 转换系数
        const double ACCEL_LSB_TO_MS2 = 4.65661287307739e-08;  // Accel 转换系数

        geometry_msgs::msg::Quaternion GetQuaternionFromEuler(double roll, double pitch, double yaw);
        miivii_bynav_driver::msg::RawIMU FindRawIMU(double timestamp);
        void GenerateIMUData();

        boost::circular_buffer<miivii_bynav_driver::msg::RawIMU> rawimu_msgs_;
        std::queue<miivii_bynav_driver::msg::Inspvax> inspvax_queue_;
        miivii_bynav_driver::msg::Inspvax last_inspvax_;
        miivii_bynav_driver::msg::Inspva last_inspva_;
        miivii_bynav_driver::msg::Insstdev last_insstdev_;
        double last_inspva_time_ = 0.0;
        double last_publish_time_ = 0.0;
    };

}

#endif // BYNAV_CONTROL_HPP_
