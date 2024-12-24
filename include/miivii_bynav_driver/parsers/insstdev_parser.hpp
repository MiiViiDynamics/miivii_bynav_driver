#ifndef INSSTDEV_PARSER_HPP
#define INSSTDEV_PARSER_HPP

#include "miivii_bynav_driver/parsers/message_parser.hpp"
#include "miivii_bynav_driver/parsers/parser_utils.hpp"
#include "miivii_bynav_driver/msg/insstdev.hpp"

namespace miivii_bynav_driver
{

    class InsstdevParser : public BynavMessages
    {
    public:
        void parse(const std::string &raw_message) override;
        miivii_bynav_driver::msg::Insstdev getParsedMessage();

    private:
        ParserUtils parserUtil_;
        miivii_bynav_driver::msg::Insstdev insstdev_msg_;
        miivii_bynav_driver::msg::Insstdev parseInsstdev(const std::string &raw_message);
    };

} // namespace miivii_bynav_driver

#endif // INSSTDEV_PARSER_HPP
