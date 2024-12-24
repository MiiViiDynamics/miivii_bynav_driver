#ifndef BYNAV_MESSAGES_HPP
#define BYNAV_MESSAGES_HPP

#include <string>

namespace miivii_bynav_driver
{

    class BynavMessages
    {
    public:
        virtual ~BynavMessages() = default;
        virtual void parse(const std::string &raw_message) = 0;
    };

} // namespace miivii_bynav_driver

#endif // BYNAV_MESSAGES_HPP
