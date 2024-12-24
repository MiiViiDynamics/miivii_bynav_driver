#ifndef INSPVAX_PARSER_HPP
#define INSPVAX_PARSER_HPP

#include "miivii_bynav_driver/parsers/message_parser.hpp"
#include "miivii_bynav_driver/parsers/parser_utils.hpp"
#include "miivii_bynav_driver/msg/inspvax.hpp"

namespace miivii_bynav_driver
{

    class InspvaxParser : public BynavMessages
    {
    public:
        void parse(const std::string &raw_message) override;
        miivii_bynav_driver::msg::Inspvax getParsedMessage();

    private:
        ParserUtils parserUtil_;
        miivii_bynav_driver::msg::Inspvax inspvax_msg_;
        miivii_bynav_driver::msg::Inspvax parseInspvax(const std::string &raw_message);
    };

} // namespace miivii_bynav_driver

#endif // INSPVAX_PARSER_HPP
