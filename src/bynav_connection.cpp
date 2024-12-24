#include <miivii_bynav_driver/bynav_connection.hpp>

namespace miivii_bynav_driver
{

    BynavConnection::BynavConnection()
        : connection_(SERIAL), is_connected_(false), serial_baud_(115200), tcp_socket_(io_service_)
    {
    }

    BynavConnection::~BynavConnection()
    {
        Disconnect();
    }

    BynavConnection::ConnectionType BynavConnection::ParseConnection(const std::string &connection)
    {
        if (connection == "serial")
        {
            return SERIAL;
        }
        else if (connection == "tcp")
        {
            return TCP;
        }

        return INVALID;
    }

    bool BynavConnection::Connect()
    {
        Disconnect();

        if (connection_ == SERIAL)
        {
            return CreateSerialConnection();
        }
        else if (connection_ == TCP)
        {
            return CreateTcpConnection();
        }

        std::cout << "Invalid connection type." << std::endl;

        return false;
    }

    void BynavConnection::Disconnect()
    {
        if (connection_ == SERIAL)
        {
            serial_.Close();
        }
        else if (connection_ == TCP)
        {
            tcp_socket_.close();
        }
        is_connected_ = false;
    }

    void BynavConnection::SetConnection(ConnectionType connection)
    {
        connection_ = connection;
    }

    void BynavConnection::SetInterface(const std::string &interface)
    {
        interface_ = interface;
    }

    void BynavConnection::SetLogPort(const std::string &log_port)
    {
        log_port_ = log_port;
    }

    void BynavConnection::SetSerialBaud(int32_t serial_baud)
    {
        serial_baud_ = serial_baud;
    }

    void BynavConnection::SetPort(int port)
    {
        port_ = port;
    }

    bool BynavConnection::CreateSerialConnection()
    {
        swri_serial_util::SerialConfig config;
        config.baud = serial_baud_;
        config.parity = swri_serial_util::SerialConfig::NO_PARITY;
        config.flow_control = false;
        config.data_bits = 8;
        config.stop_bits = 1;
        config.low_latency_mode = false;
        config.writable = true;

        if (serial_.Open(interface_, config))
        {
            is_connected_ = ConfigureSerial();
        }
        else
        {
            std::cout << serial_.ErrorMsg() << std::endl;
        }

        return is_connected_;
    }

    bool BynavConnection::CreateTcpConnection()
    {
        std::string port = std::to_string(port_);
        try
        {
            std::cout << "Connecting to " << interface_ << ":" << port << std::endl;
            boost::asio::ip::tcp::resolver resolver(io_service_);
            boost::asio::ip::tcp::resolver::query query(interface_, port);
            boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);

            boost::asio::connect(tcp_socket_, iter);
            std::cout << "Connected to " << interface_ << ":" << port << std::endl;
        }
        catch (std::exception &e)
        {
            std::cout << "Error connecting to " << interface_ << ":" << port << " " << e.what() << std::endl;
            return false;
        }

        is_connected_ = true;
        return is_connected_;
    }

    std::string BynavConnection::GenerateUnlogCommand() const
    {
        std::stringstream command;
        command << "unlogall " << log_port_ << "\r\n"
                << "unlogall " << log_port_ << "\r\n"
                << "unlogall " << log_port_ << "\r\n";

        return command.str();
    }

    bool BynavConnection::ConfigureSerial()
    {
        bool configured = true;
        const std::string unlogCommand = GenerateUnlogCommand();
        configured = configured && Write(unlogCommand);

        if (ReadData() != READ_SUCCESS)
        {
            configured = false;
        }
        return configured;
    }

    void BynavConnection::CollectData(BynavMessageOpts const &opts)
    {
        if (connection_ == TCP)
        {
            const std::string unlogCommand = GenerateUnlogCommand();
            Write(unlogCommand);
        }

        for (const auto &option : opts)
        {
            std::stringstream command;
            command << std::setprecision(3);
            if (option.second < 0.0)
            {
                command << "log " << log_port_ << " " << option.first << " onchanged\r\n";
            }
            else
            {
                command << "log " << log_port_ << " " << option.first << " ontime " << option.second
                        << "\r\n";
            }
            Write(command.str());
        }
    }

    bool BynavConnection::Write(const std::string &command)
    {
        std::vector<uint8_t> bytes(command.begin(), command.end());

        if (connection_ == SERIAL)
        {
            int32_t written = serial_.Write(bytes);
            if (written != (int32_t)command.length())
            {
                return false;
            }
            return written == (int32_t)command.length();
        }
        else if (connection_ == TCP)
        {
            boost::system::error_code error;
            try
            {
                size_t written;
                written = boost::asio::write(tcp_socket_, boost::asio::buffer(bytes), error);
                if (error)
                {
                    std::cout << "Error writing to TCP socket: " << error.message() << std::endl;
                    Disconnect();
                }
                return written == (int32_t)command.length();
            }
            catch (std::exception &e)
            {
                std::cout << "Error writing to TCP socket: " << e.what() << std::endl;
                Disconnect();
            }
        }

        return false;
    }

    BynavConnection::ReadResult BynavConnection::ReadData()
    {
        if (connection_ == SERIAL)
        {
            swri_serial_util::SerialPort::Result result =
                serial_.ReadBytes(data_buffer_, 0, 1000);

            if (result == swri_serial_util::SerialPort::ERROR)
            {
                std::cout << "Error reading from serial device: " << serial_.ErrorMsg() << std::endl;
                return READ_ERROR;
            }
            else if (result == swri_serial_util::SerialPort::TIMEOUT)
            {
                std::cout << "Timed out waiting for serial device." << std::endl;
                return READ_TIMEOUT;
            }
            else if (result == swri_serial_util::SerialPort::INTERRUPTED)
            {
                std::cout << "Interrupted during read from serial device." << std::endl;
                return READ_INTERRUPTED;
            }

            return READ_SUCCESS;
        }
        else if (connection_ == TCP)
        {
            try
            {
                boost::system::error_code error;
                size_t len;

                len = tcp_socket_.read_some(boost::asio::buffer(socket_buffer_), error);

                data_buffer_.insert(data_buffer_.end(), socket_buffer_.begin(),
                                    socket_buffer_.begin() + len);
                if (error)
                {
                    std::cout << "Read error: " << error.message() << std::endl;
                    Disconnect();
                    return READ_ERROR;
                }
                return READ_SUCCESS;
            }
            catch (std::exception &e)
            {
                std::cout << "Read error: " << e.what() << std::endl;
            }
        }

        std::cout << "Unsupported connection type." << std::endl;

        return READ_ERROR;
    }

    std::vector<uint8_t> BynavConnection::GetDataBuffer() const
    {
        return data_buffer_;
    }

    void BynavConnection::ClearDataBuffer()
    {
        data_buffer_.clear();
    }

} // namespace miivii_bynav_driver
