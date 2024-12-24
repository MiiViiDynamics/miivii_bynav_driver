#include "miivii_bynav_driver/parsers/corrimudata_parser.hpp"

namespace miivii_bynav_driver
{
    const double DEGREES_TO_RADIANS = M_PI / 180.0;

    void CorrectIMUParser::parse(const std::string &raw_message)
    {
        corrected_imu_data_msg_ = parseCorrectedImu(raw_message);
    }

    miivii_bynav_driver::msg::CorrectedImuData CorrectIMUParser::getParsedMessage()
    {
        return corrected_imu_data_msg_;
    }

    miivii_bynav_driver::msg::CorrectedImuData CorrectIMUParser::parseCorrectedImu(const std::string &raw_message)
    {

        miivii_bynav_driver::msg::CorrectedImuData corrected_imu_data_msg;

        size_t delimiter_pos = raw_message.find(';');
        std::string data_str = raw_message.substr(delimiter_pos + 1);
        std::vector<std::string> fields = parserUtil_.split(data_str, ',');

        corrected_imu_data_msg.bynav_msg_header = parserUtil_.extractHeader(raw_message);

        corrected_imu_data_msg.gps_week_num = std::stoul(fields[0]);
        corrected_imu_data_msg.gps_seconds = std::stod(fields[1]);

        corrected_imu_data_msg.pitch_rate = std::stod(fields[2]);
        corrected_imu_data_msg.roll_rate = std::stod(fields[3]);
        corrected_imu_data_msg.yaw_rate = std::stod(fields[4]);

        corrected_imu_data_msg.lateral_acceleration = std::stod(fields[5]);
        corrected_imu_data_msg.longitudinal_acceleration = std::stod(fields[6]);
        corrected_imu_data_msg.vertical_acceleration = std::stod(fields[7]);

        return corrected_imu_data_msg;
    }

} // namespace miivii_bynav_driver