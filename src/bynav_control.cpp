#include <miivii_bynav_driver/bynav_control.hpp>
#include <sstream>

#include <boost/make_shared.hpp>
#include <rclcpp/rclcpp.hpp>

namespace miivii_bynav_driver
{
    BynavControl::BynavControl(rclcpp::Logger logger)
        : logger_(logger),
          bynavConnection_(),
          parserUtils_(),
          rawimu_msgs_(100)
    {
    }

    BynavControl::~BynavControl() {}

    void BynavControl::SetImuRate(int rate)
    {
        imu_rate_ = rate;
    }

    void BynavControl::SetConnectionParams(std::string type, std::string param1, std::string param2, int param3)
    {
        connection_type_ = type;
        bynavConnection_.SetInterface(param1);
        bynavConnection_.SetLogPort(param2);

        auto type_int = bynavConnection_.ParseConnection(connection_type_);

        if (type_int == BynavConnection::ConnectionType::INVALID)
        {
            throw std::runtime_error("Invalid connection type");
        }
        else if (type_int == BynavConnection::ConnectionType::SERIAL)
        {

            if (param3 <= 0)
            {
                throw std::runtime_error("Invalid baud rate");
            }
            else
            {
                bynavConnection_.SetSerialBaud(param3);
            }
        }
        else if (type_int == BynavConnection::ConnectionType::TCP)
        {
            if (param3 <= 0)
            {
                throw std::runtime_error("Invalid port number");
            }
            else
            {
                bynavConnection_.SetPort(param3);
            }
        }

        bynavConnection_.SetConnection(type_int);
    }

    void BynavControl::InitializeConnection()
    {
        auto type = bynavConnection_.ParseConnection(connection_type_);

        if (bynavConnection_.Connect())
        {
            RCLCPP_INFO(logger_, "Successfully connected to the device.");
        }
        else
        {
            throw std::runtime_error("Connect failed");
        }
    }

    void BynavControl::StartDataProcessingThread(BynavConnection::BynavMessageOpts const &opts)
    {
        processing_thread_active_ = true;
        bynavConnection_.CollectData(opts);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        processing_thread_ = std::thread(&BynavControl::ProcessData, this);
    }

    void BynavControl::StopDataProcessingThread()
    {
        processing_thread_active_ = false;
        queue_cond_.notify_one();
        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }
    }

    void BynavControl::SetImuDataCallback(const ImuDataCallback &callback)
    {
        imu_data_callback_ = callback;
    }

    void BynavControl::SetFrameId(std::string const &frame_id)
    {
        frame_id_ = frame_id;
    }

    void BynavControl::ProcessData()
    {
        std::string accumulated_data;
        size_t pos, last_pos = 0;

        while (rclcpp::ok())
        {
            auto result = bynavConnection_.ReadData();

            if (result == BynavConnection::READ_SUCCESS)
            {
                // 将接收到的数据追加到accumulated_data中
                auto data_buffer = bynavConnection_.GetDataBuffer();
                accumulated_data.append(data_buffer.begin(), data_buffer.end());
                bynavConnection_.ClearDataBuffer();

                while ((pos = accumulated_data.find("\r\n", last_pos)) != std::string::npos)
                {
                    std::string data = accumulated_data.substr(last_pos, pos - last_pos);

                    if (!data.empty())
                    {
                        AnalysisData(data);
                    }

                    last_pos = pos + 2; // 跳过 "\r\n"
                }

                if (last_pos < accumulated_data.length())
                {
                    accumulated_data.erase(0, last_pos); // 清除已处理的数据
                }
                else
                {
                    accumulated_data.clear(); // 清空消息
                }
                last_pos = 0;
            }
            else
            {
                RCLCPP_WARN(logger_, "Error reading data");
                break;
            }
        }
    }

    void BynavControl::AnalysisData(const std::string &data)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        try
        {
            // RCLCPP_INFO(logger_, "Received data: %s", data.c_str());
            parserUtils_.parseHeader(data);
            auto message_type = parserUtils_.getMessageName();
            auto messageParser = miivii_bynav_driver::MessageParser::createParser(message_type);

            if (auto rawimu_parser = dynamic_cast<miivii_bynav_driver::RawIMUParser *>(messageParser.get()))
            {
                rawimu_parser->parse(data);
                auto rawimu_data = rawimu_parser->getParsedMessage();
                rawimu_msgs_.push_back(rawimu_data);
                // RCLCPP_INFO(logger_, "rawimu z_accel: %d", rawimu_data.z_accel);
                this->GenerateIMUData();
            }
            else if (auto inspvax_parser = dynamic_cast<miivii_bynav_driver::InspvaxParser *>(messageParser.get()))
            {
                inspvax_parser->parse(data);
                last_inspvax_ = inspvax_parser->getParsedMessage();
            }
            else if (auto inspva_parser = dynamic_cast<miivii_bynav_driver::InspvaParser *>(messageParser.get()))
            {
                inspva_parser->parse(data);
                last_inspva_ = inspva_parser->getParsedMessage();
                // RCLCPP_INFO(logger_, "inspva roll: %f", last_inspva_.roll);
            }
            else if (auto insstdev_parser = dynamic_cast<miivii_bynav_driver::InsstdevParser *>(messageParser.get()))
            {

                insstdev_parser->parse(data);
                last_insstdev_ = insstdev_parser->getParsedMessage();
                // RCLCPP_INFO(logger_, "insstdev roll_dev: %f", last_insstdev_.roll_dev);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

    miivii_bynav_driver::msg::RawIMU BynavControl::FindRawIMU(double timestamp)
    {

        miivii_bynav_driver::msg::RawIMU closest_imu;
        for (auto it = rawimu_msgs_.rbegin(); it != rawimu_msgs_.rend(); ++it)
        {
            double rawimu_time = GPS_UTC_DIFF + it->gps_week_num * SECONDS_PER_WEEK + it->gps_seconds;

            // RCLCPP_INFO(logger_, "time: %f", rawimu_time);
            if (std::fabs(rawimu_time - timestamp) < IMU_TOLERANCE_S)
            {
                closest_imu = *it;
                break;
            }
        }
        return closest_imu;
    }

    geometry_msgs::msg::Quaternion BynavControl::GetQuaternionFromEuler(double roll, double pitch, double yaw)
    {
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);

        geometry_msgs::msg::Quaternion msg_quat;
        msg_quat.x = quat.x();
        msg_quat.y = quat.y();
        msg_quat.z = quat.z();
        msg_quat.w = quat.w();

        return msg_quat;
    }

    void BynavControl::GenerateIMUData()
    {
        double last_inspva_time = GPS_UTC_DIFF + last_inspva_.week * SECONDS_PER_WEEK + last_inspva_.seconds;
        // RCLCPP_INFO(logger_, "Last Inspva time: %f", GPS_UTC_DIFF + last_inspva_.week * SECONDS_PER_WEEK + last_inspva_.seconds);

        if (last_inspva_time_ == 0.0 || last_inspva_time > last_inspva_time_)
        {
            last_inspva_time_ = last_inspva_time;
        }
        else
        {
            return;
        }

        auto rawimu_data = FindRawIMU(last_inspva_time);
        if (rawimu_data.gps_week_num != 0 || rawimu_data.gps_seconds != 0)
        {

            // RCLCPP_INFO(logger_, "Closest RawIMU time: %f", GPS_UTC_DIFF + rawimu_data.gps_week_num * SECONDS_PER_WEEK + rawimu_data.gps_seconds);

            double utc_time = GPS_UTC_DIFF + rawimu_data.gps_week_num * SECONDS_PER_WEEK + rawimu_data.gps_seconds;

            if (utc_time > last_publish_time_)
            {
                last_publish_time_ = utc_time;

                long long seconds = static_cast<long long>(utc_time);

                double fractional_seconds = utc_time - static_cast<double>(seconds);
                long long nanoseconds = static_cast<long long>(fractional_seconds * 1e9);

                sensor_msgs::msg::Imu imu;

                // 输出UTC时间
                // auto timePoint = std::chrono::system_clock::time_point(std::chrono::seconds(seconds) + std::chrono::nanoseconds(nanoseconds));
                // std::time_t utcTime = std::chrono::system_clock::to_time_t(timePoint);
                // std::tm *utcTm = std::gmtime(&utcTime);
                // std::cout << "UTC Time: " << std::put_time(utcTm, "%Y-%m-%d %H:%M:%S") << std::endl;

                imu.header.stamp.sec = seconds;
                imu.header.stamp.nanosec = nanoseconds;
                imu.header.frame_id = frame_id_;
                imu.orientation = GetQuaternionFromEuler(last_inspva_.roll, last_inspva_.pitch, last_inspva_.azimuth);

                if (last_insstdev_.bynav_msg_header.gps_week_num != 0)
                {
                    imu.orientation_covariance[0] = std::pow(2, last_insstdev_.pitch_dev);
                    imu.orientation_covariance[4] = std::pow(2, last_insstdev_.roll_dev);
                    imu.orientation_covariance[8] = std::pow(2, last_insstdev_.azimuth_dev);
                }
                else
                {
                    imu.orientation_covariance[0] = 1e-3;
                    imu.orientation_covariance[4] = 1e-3;
                    imu.orientation_covariance[8] = 1e-3;
                }

                imu.angular_velocity.x = rawimu_data.x_gyro * GYRO_LSB_TO_DEG_S * imu_rate_;
                imu.angular_velocity.y = -(rawimu_data.y_gyro) * GYRO_LSB_TO_DEG_S * imu_rate_;
                imu.angular_velocity.z = -(rawimu_data.z_gyro) * GYRO_LSB_TO_DEG_S * imu_rate_;
                imu.angular_velocity_covariance[0] = 1e-3;
                imu.angular_velocity_covariance[4] = 1e-3;
                imu.angular_velocity_covariance[8] = 1e-3;

                imu.linear_acceleration.x = rawimu_data.x_accel * ACCEL_LSB_TO_MS2 * imu_rate_;
                imu.linear_acceleration.y = rawimu_data.y_accel * ACCEL_LSB_TO_MS2 * imu_rate_;
                imu.linear_acceleration.z = rawimu_data.z_accel * ACCEL_LSB_TO_MS2 * imu_rate_;
                imu.linear_acceleration_covariance[0] = 1e-3;
                imu.linear_acceleration_covariance[4] = 1e-3;
                imu.linear_acceleration_covariance[8] = 1e-3;

                if (imu_data_callback_)
                {
                    imu_data_callback_(imu);
                }
            }
        }
    }

    void BynavControl::DestroyConnection()
    {
        bynavConnection_.Disconnect();
    }

} // namespace miivii_bynav_driver
