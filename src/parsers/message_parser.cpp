#include "miivii_bynav_driver/parsers/message_parser.hpp"

namespace miivii_bynav_driver
{

    std::unique_ptr<BynavMessages> MessageParser::createParser(const std::string &message_type)
    {
        if (message_type == "#INSPVAXA")
        {
            return std::make_unique<InspvaxParser>();
        }
        else if (message_type == "#CORRIMUDATAA")
        {
            return std::make_unique<CorrectIMUParser>();
        }
        else if (message_type == "#RAWIMUA")
        {
            return std::make_unique<RawIMUParser>();
        }
        else if (message_type == "#INSPVAA")
        {
            return std::make_unique<InspvaParser>();
        }
        else if (message_type == "#INSSTDEVA")
        {
            return std::make_unique<InsstdevParser>();
        }
        throw std::runtime_error("Unknown message type: " + message_type);
    }

} // namespace miivii_bynav_driver