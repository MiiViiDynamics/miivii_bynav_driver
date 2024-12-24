#ifndef CORRIMUDATA_PARSER_HPP
#define CORRIMUDATA_PARSER_HPP

#include "miivii_bynav_driver/parsers/message_parser.hpp"
#include "miivii_bynav_driver/parsers/parser_utils.hpp"
#include "miivii_bynav_driver/msg/corrected_imu_data.hpp"
#include <cmath>

namespace miivii_bynav_driver
{

    class CorrectIMUParser : public BynavMessages
    {
    public:
        void parse(const std::string &raw_message) override;
        miivii_bynav_driver::msg::CorrectedImuData getParsedMessage();

    private:
        ParserUtils parserUtil_;
        miivii_bynav_driver::msg::CorrectedImuData corrected_imu_data_msg_;
        miivii_bynav_driver::msg::CorrectedImuData parseCorrectedImu(const std::string &raw_message);
    };

} // namespace miivii_bynav_driver

#endif // CORRIMUDATA_PARSER_HPP
