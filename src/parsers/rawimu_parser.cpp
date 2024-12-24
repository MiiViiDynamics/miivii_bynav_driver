#include "miivii_bynav_driver/parsers/rawimu_parser.hpp"

namespace miivii_bynav_driver
{

    void RawIMUParser::parse(const std::string &raw_message)
    {
        rawimu_msg_ = parseRawIMU(raw_message);
    }

    miivii_bynav_driver::msg::RawIMU RawIMUParser::getParsedMessage()
    {
        return rawimu_msg_;
    }

    miivii_bynav_driver::msg::RawIMU RawIMUParser::parseRawIMU(const std::string &raw_message)
    {
        miivii_bynav_driver::msg::RawIMU rawimu_msg;

        rawimu_msg.bynav_msg_header = parserUtil_.extractHeader(raw_message);

        size_t delimiter_pos = raw_message.find(';');
        std::string data_str = raw_message.substr(delimiter_pos + 1);
        std::vector<std::string> fields = parserUtil_.split(data_str, ',');

        rawimu_msg.gps_week_num = static_cast<uint32_t>(std::stoi(fields[0]));
        rawimu_msg.gps_seconds = std::stod(fields[1]);
        rawimu_msg.imu_status = static_cast<uint32_t>(std::stoi(fields[2]));
        rawimu_msg.z_accel = std::stoi(fields[3]);
        rawimu_msg.y_accel = std::stoi(fields[4]);
        rawimu_msg.x_accel = std::stoi(fields[5]);
        rawimu_msg.z_gyro = std::stoi(fields[6]);
        rawimu_msg.y_gyro = std::stoi(fields[7]);
        rawimu_msg.x_gyro = std::stoi(fields[8]);

        return rawimu_msg;
    }

} // namespace miivii_bynav_driver
