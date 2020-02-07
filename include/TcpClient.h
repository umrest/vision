#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <boost/array.hpp>

#include <chrono>
#include <thread>

#include <comm/CommunicationDefinitions.h>

using namespace comm;

class TcpClient
{
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket;

    bool socket_connected = false;

    std::string host = "127.0.0.1";
    int port = 8091;

    unsigned char socket_buffer[1024]; // buffer from serial

public:
    TcpClient() : socket(io_service)
    {
        socket_reconnect();
    }

    void socket_reconnect()
    {
        boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(host), port);

        boost::system::error_code ec;

        socket.connect(endpoint, ec);

        if (!ec)
        {

            std::cout << "Socket: Reconnect Succeeded" << std::endl;

            boost::asio::socket_base::send_buffer_size option(128);
            socket.set_option(option);

            comm::Identifier identifier;
            identifier.identifier = (uint8_t)CommunicationDefinitions::IDENTIFIER::VISION;
            std::vector<uint8_t> data = identifier.Serialize();
            write(data);
        }
        else
        {
            socket_connected = false;
            std::cout << "Socket: Reconnect Failed" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void write_no_key(std::vector<uint8_t> &data){
        boost::system::error_code ec;
        int bytesTransferred = socket.write_some(boost::asio::buffer(data, data.size()), ec);

        handle_socket_write(ec, bytesTransferred);
    }

    void write(std::vector<uint8_t> &data)
    {
        boost::system::error_code ec;

        boost::array<boost::asio::const_buffer, 2> d = {
            boost::asio::buffer(CommunicationDefinitions::key, 3),
            boost::asio::buffer(data, data.size())};

        int bytesTransferred = socket.write_some(d, ec);

        handle_socket_write(ec, bytesTransferred);
    }

    bool read_nonblocking(uint8_t *data, int size)
    {
        if (socket.available() >= size)
        {
            socket.read_some(boost::asio::buffer(data, size));
            return true;
        }
        return false;
    }

    void read(char *data, int size)
    {
        socket.async_read_some(boost::asio::buffer(socket_buffer, 1),
                               boost::bind(&TcpClient::handle_socket_receive,
                                           this, boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
    }

    void handle_socket_receive(const boost::system::error_code &ec, std::size_t bytes_transferred)
    {
        if (!ec)
        {
            // Data

            //socket_read();

            std::cout << "Socket Recieve" << std::endl;
        }
        else
        {
            std::cout << "Socket: Disconnected" << std::endl;
            socket.close();
            socket_reconnect();
            //socket_read();
        }
    }

    void handle_socket_write(const boost::system::error_code &ec, std::size_t bytes_transferred)
    {
        if (!ec)
        {
        }
        else
        {
            std::cout << "Socket: Disconnected" << std::endl;
            socket.close();
            socket_reconnect();
            //socket_read();
        }
    }
};