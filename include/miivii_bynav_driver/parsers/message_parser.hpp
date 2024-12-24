#ifndef MESSAGE_PARSER_HPP
#define MESSAGE_PARSER_HPP

#include "miivii_bynav_driver/parsers/bynav_messages.hpp"
#include <memory>
#include <string>

#include "miivii_bynav_driver/parsers/inspvax_parser.hpp"
#include "miivii_bynav_driver/parsers/corrimudata_parser.hpp"
#include "miivii_bynav_driver/parsers/rawimu_parser.hpp"
#include "miivii_bynav_driver/parsers/inspva_parser.hpp"
#include "miivii_bynav_driver/parsers/insstdev_parser.hpp"
#include "miivii_bynav_driver/parsers/inspvax_parser.hpp"

namespace miivii_bynav_driver
{

    class MessageParser
    {
    public:
        static std::unique_ptr<BynavMessages> createParser(const std::string &message_type);
    };

} // namespace miivii_bynav_driver

#endif // MESSAGE_PARSER_HPP
