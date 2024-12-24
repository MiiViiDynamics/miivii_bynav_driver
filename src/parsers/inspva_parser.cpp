#include "miivii_bynav_driver/parsers/inspva_parser.hpp"

namespace miivii_bynav_driver
{

    void InspvaParser::parse(const std::string &raw_message)
    {
        inspva_msg_ = parseInspva(raw_message);
    }

    miivii_bynav_driver::msg::Inspva InspvaParser::getParsedMessage()
    {
        return inspva_msg_;
    }

    miivii_bynav_driver::msg::Inspva InspvaParser::parseInspva(const std::string &raw_message)
    {
        miivii_bynav_driver::msg::Inspva inspva_msg;

        inspva_msg.bynav_msg_header = parserUtil_.extractHeader(raw_message);

        size_t delimiter_pos = raw_message.find(';');
        std::string data_str = raw_message.substr(delimiter_pos + 1);
        std::vector<std::string> fields = parserUtil_.split(data_str, ',');

        inspva_msg.week = static_cast<uint32_t>(std::stoi(fields[0]));
        inspva_msg.seconds = std::stod(fields[1]);
        inspva_msg.latitude = std::stod(fields[2]);
        inspva_msg.longitude = std::stod(fields[3]);
        inspva_msg.height = std::stod(fields[4]);
        inspva_msg.north_velocity = std::stod(fields[5]);
        inspva_msg.east_velocity = std::stod(fields[6]);
        inspva_msg.up_velocity = std::stod(fields[7]);
        inspva_msg.roll = std::stod(fields[8]);
        inspva_msg.pitch = std::stod(fields[9]);
        inspva_msg.azimuth = std::stod(fields[10]);
        inspva_msg.status = fields[11];

        return inspva_msg;
    }

} // namespace miivii_bynav_driver
