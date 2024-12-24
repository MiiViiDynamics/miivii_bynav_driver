#ifndef RAWIMU_PARSER_HPP
#define RAWIMU_PARSER_HPP

#include "miivii_bynav_driver/parsers/message_parser.hpp"
#include "miivii_bynav_driver/parsers/parser_utils.hpp"
#include "miivii_bynav_driver/msg/raw_imu.hpp"

namespace miivii_bynav_driver
{

    class RawIMUParser : public BynavMessages
    {
    public:
        void parse(const std::string &raw_message) override;
        miivii_bynav_driver::msg::RawIMU getParsedMessage();

    private:
        ParserUtils parserUtil_;
        miivii_bynav_driver::msg::RawIMU rawimu_msg_;
        miivii_bynav_driver::msg::RawIMU parseRawIMU(const std::string &raw_message);
    };

} // namespace miivii_bynav_driver

#endif // RAWIMU_PARSER_HPP
