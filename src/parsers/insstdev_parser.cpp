#include "miivii_bynav_driver/parsers/insstdev_parser.hpp"

namespace miivii_bynav_driver
{

    void InsstdevParser::parse(const std::string &raw_message)
    {
        insstdev_msg_ = parseInsstdev(raw_message);
    }

    miivii_bynav_driver::msg::Insstdev InsstdevParser::getParsedMessage()
    {
        return insstdev_msg_;
    }

    miivii_bynav_driver::msg::Insstdev InsstdevParser::parseInsstdev(const std::string &raw_message)
    {
        miivii_bynav_driver::msg::Insstdev insstdev_msg_;

        insstdev_msg_.bynav_msg_header = parserUtil_.extractHeader(raw_message);

        size_t delimiter_pos = raw_message.find(';');
        std::string data_str = raw_message.substr(delimiter_pos + 1);
        std::vector<std::string> fields = parserUtil_.split(data_str, ',');

        insstdev_msg_.latitude_dev = std::stof(fields[0]);
        insstdev_msg_.longitude_dev = std::stof(fields[1]);
        insstdev_msg_.height_dev = std::stof(fields[2]);
        insstdev_msg_.north_velocity_dev = std::stof(fields[3]);
        insstdev_msg_.east_velocity_dev = std::stof(fields[4]);
        insstdev_msg_.up_velocity_dev = std::stof(fields[5]);
        insstdev_msg_.roll_dev = std::stof(fields[6]);
        insstdev_msg_.pitch_dev = std::stof(fields[7]);
        insstdev_msg_.azimuth_dev = std::stof(fields[8]);
        insstdev_msg_.extended_status = static_cast<uint32_t>(std::stoi(fields[9]));
        insstdev_msg_.time_since_update = static_cast<uint16_t>(std::stoi(fields[10]));

        return insstdev_msg_;
    }

} // namespace miivii_bynav_driver
