#ifndef INSPVA_PARSER_HPP
#define INSPVA_PARSER_HPP

#include "miivii_bynav_driver/parsers/message_parser.hpp"
#include "miivii_bynav_driver/parsers/parser_utils.hpp"
#include "miivii_bynav_driver/msg/inspva.hpp"

namespace miivii_bynav_driver
{

    class InspvaParser : public BynavMessages
    {
    public:
        void parse(const std::string &raw_message) override;
        miivii_bynav_driver::msg::Inspva getParsedMessage();

    private:
        ParserUtils parserUtil_;
        miivii_bynav_driver::msg::Inspva inspva_msg_;
        miivii_bynav_driver::msg::Inspva parseInspva(const std::string &raw_message);
    };

} // namespace miivii_bynav_driver

#endif // INSPVA_PARSER_HPP
