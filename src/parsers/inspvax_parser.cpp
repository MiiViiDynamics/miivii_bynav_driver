#include "miivii_bynav_driver/parsers/inspvax_parser.hpp"

namespace miivii_bynav_driver
{

    void InspvaxParser::parse(const std::string &raw_message)
    {
        inspvax_msg_ = parseInspvax(raw_message);
    }

    miivii_bynav_driver::msg::Inspvax InspvaxParser::getParsedMessage()
    {
        return inspvax_msg_;
    }

    miivii_bynav_driver::msg::Inspvax InspvaxParser::parseInspvax(const std::string &raw_message)
    {
        miivii_bynav_driver::msg::Inspvax inspvax_msg;

        inspvax_msg.bynav_msg_header = parserUtil_.extractHeader(raw_message);

        size_t delimiter_pos = raw_message.find(';');
        std::string data_str = raw_message.substr(delimiter_pos + 1);
        std::vector<std::string> fields = parserUtil_.split(data_str, ',');

        inspvax_msg.ins_status = fields[0];
        inspvax_msg.position_type = fields[1];

        inspvax_msg.latitude = std::stod(fields[2]);
        inspvax_msg.longitude = std::stod(fields[3]);
        inspvax_msg.altitude = std::stod(fields[4]);
        inspvax_msg.undulation = std::stof(fields[5]);
        inspvax_msg.north_velocity = std::stod(fields[6]);
        inspvax_msg.east_velocity = std::stod(fields[7]);
        inspvax_msg.up_velocity = std::stod(fields[8]);
        inspvax_msg.roll = std::stod(fields[9]);
        inspvax_msg.pitch = std::stod(fields[10]);
        inspvax_msg.azimuth = std::stod(fields[11]);

        inspvax_msg.latitude_std = std::stof(fields[12]);
        inspvax_msg.longitude_std = std::stof(fields[13]);
        inspvax_msg.altitude_std = std::stof(fields[14]);
        inspvax_msg.north_velocity_std = std::stof(fields[15]);
        inspvax_msg.east_velocity_std = std::stof(fields[16]);
        inspvax_msg.up_velocity_std = std::stof(fields[17]);
        inspvax_msg.roll_std = std::stof(fields[18]);
        inspvax_msg.pitch_std = std::stof(fields[19]);
        inspvax_msg.azimuth_std = std::stof(fields[20]);

        inspvax_msg.extended_status = static_cast<uint32_t>(std::stoi(fields[21]));
        inspvax_msg.seconds_since_update = static_cast<uint16_t>(std::stoul(fields[22]));

        return inspvax_msg;
    }

} // namespace miivii_bynav_driver
