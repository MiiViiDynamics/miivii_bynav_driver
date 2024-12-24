#include "miivii_bynav_driver/parsers/parser_utils.hpp"

namespace miivii_bynav_driver
{

    ParserUtils::ParserUtils() {}

    void ParserUtils::parseHeader(const std::string &raw_message)
    {
        try
        {
            if (raw_message.empty())
            {
                throw std::invalid_argument("Input string is empty.");
            }
            message_head_ = extractHeader(raw_message);
            message_name_ = message_head_.message_name;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ParserUtils"), "Error parsing header: %s", e.what());
            throw;
        }
    }

    std::string ParserUtils::getMessageName() const
    {
        return message_name_;
    }

    miivii_bynav_driver::msg::BynavMessageHeader ParserUtils::getMessageHeader() const
    {
        return message_head_;
    }

    miivii_bynav_driver::msg::BynavMessageHeader ParserUtils::extractHeader(const std::string &raw_message)
    {
        size_t delimiter_pos = raw_message.find(';');
        if (delimiter_pos == std::string::npos)
        {
            throw std::runtime_error("Delimiter not found in the raw message, Discard this.");
        }
        std::string header_str = raw_message.substr(0, delimiter_pos);

        std::vector<std::string> fields = split(header_str, ',');
        if (fields.size() < 7)
        {
            throw std::runtime_error("Not enough fields in the header.");
        }

        miivii_bynav_driver::msg::BynavMessageHeader header;
        header.message_name = fields[0];
        header.port = fields[1];
        header.sequence_num = std::stoul(fields[2]);
        header.percent_idle_time = std::stof(fields[3]);
        header.gps_time_status = fields[4];
        header.gps_week_num = std::stoul(fields[5]);
        header.gps_seconds = std::stod(fields[6]);

        return header;
    }

    std::vector<std::string> ParserUtils::split(const std::string &s, char delimiter)
    {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter))
        {
            if (token.empty())
            {
                throw std::runtime_error("Empty token found while splitting the string.");
            }
            tokens.push_back(token);
        }
        return tokens;
    }
}