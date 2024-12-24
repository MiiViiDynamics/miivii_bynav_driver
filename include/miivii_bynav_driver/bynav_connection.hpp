#ifndef BYNAV_CONNECTION_HPP
#define BYNAV_CONNECTION_HPP

#include <string>
#include <vector>
#include <memory>
#include <boost/asio.hpp>
#include <boost/make_shared.hpp>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <swri_serial_util/serial_port.h> // Assuming this is the header for serial port utilities

namespace miivii_bynav_driver
{

    class BynavConnection
    {
    public:
        enum ConnectionType
        {
            INVALID,
            SERIAL,
            TCP
        };

        enum ReadResult
        {
            READ_ERROR,
            READ_SUCCESS,
            READ_TIMEOUT,
            READ_INTERRUPTED
        };

        using BynavMessageOpts = std::map<std::string, double>;

        BynavConnection();
        ~BynavConnection();

        ConnectionType ParseConnection(const std::string &connection);
        bool Connect();
        void Disconnect();

        void SetConnection(ConnectionType connection);
        void SetInterface(const std::string &interface);
        void SetLogPort(const std::string &log_port);
        void SetSerialBaud(int32_t serial_baud);
        void SetPort(int port);

        void CollectData(BynavMessageOpts const &opts);
        bool Write(const std::string &command);
        ReadResult ReadData();
        std::vector<uint8_t> GetDataBuffer() const;
        void ClearDataBuffer();

    private:
        ConnectionType connection_;
        bool is_connected_ = false;
        std::string interface_;
        std::string log_port_; // Port name for connection
        int32_t serial_baud_;  // Baud rate for serial connection
        int port_;             // Port number for TCP connection

        swri_serial_util::SerialPort serial_;

        boost::asio::io_service io_service_;
        boost::asio::ip::tcp::socket tcp_socket_;

        std::vector<uint8_t> data_buffer_;        // Buffer to store incoming data
        std::array<uint8_t, 1024> socket_buffer_; // Buffer for TCP socket data

        std::string GenerateUnlogCommand() const;
        bool CreateSerialConnection();
        bool ConfigureSerial();
        bool CreateTcpConnection();
    };

} // namespace miivii_bynav_driver

#endif // BYNAV_CONNECTION_HPP
