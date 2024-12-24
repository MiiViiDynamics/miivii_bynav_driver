#ifndef PARSER_UTILS_HPP
#define PARSER_UTILS_HPP

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include "miivii_bynav_driver/msg/bynav_message_header.hpp"
#include "rclcpp/rclcpp.hpp"

namespace miivii_bynav_driver
{

    class ParserUtils
    {
    public:
        ParserUtils();
        void parseHeader(const std::string &raw_message);
        std::string getMessageName() const;
        miivii_bynav_driver::msg::BynavMessageHeader getMessageHeader() const;

        miivii_bynav_driver::msg::BynavMessageHeader extractHeader(const std::string &raw_message);
        std::vector<std::string> split(const std::string &s, char delimiter);

    private:
        std::string message_name_;
        miivii_bynav_driver::msg::BynavMessageHeader message_head_;
    };
}

#endif // PARSER_UTILS_HPP
