/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-05
 */


#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <iomanip>
#include "network/NetworkInterface.h"


namespace network
{
    /**
     * Return a string extracted from a streambuf.
     * @param streambuf
     * @return the string extracted from the streambuf
     */
    std::string make_string(const boost::asio::streambuf &streambuf)
    {
        return {boost::asio::buffers_begin(streambuf.data()),
                boost::asio::buffers_end(streambuf.data())};
    }

    NetworkInterface::NetworkInterface(): socket(io_service)
    {
    }

    NetworkInterface::NetworkInterface(const std::string &ip, unsigned short port) :
            endpoint(boost::asio::ip::address::from_string(ip), port),
            socket(io_service)
    {
        boost::asio::socket_base::keep_alive option(true);
        socket.set_option(option);
        socket.connect(endpoint);
    }

    void NetworkInterface::sendMessage(std::string msg)
    {
        if (!is_open())
        {
            throw std::runtime_error("Connection is not open");
        }
        std::stringstream sizestr;
        size_t out_size = htonl(msg.size());
        sizestr << std::setw(8) << std::hex << out_size;
        boost::asio::write(socket, boost::asio::buffer(sizestr.str()));
        boost::asio::write(socket, boost::asio::buffer(msg));
        std::cout << "roborobo: message sent" << std::endl;
    }

    std::string NetworkInterface::receiveMessage()
    {
        std::cout << "roborobo: Waiting for message" << std::endl;
        if (!is_open())
        {
            throw std::runtime_error("Connection is not open");
        }
        std::string message;
        char header[8] = {'\0'};
        boost::system::error_code error;
        socket.read_some(boost::asio::buffer(header, 8), error);
        if (error == boost::asio::error::eof)
        {
            std::cout << "roborobo: message eof" << std::endl;
            message = "";
        }
        else
        {
            std::cout << "socker err=" << error << std::endl;
            size_t message_length = static_cast<size_t>(std::stoul(header, nullptr, 16));
            std::cout << "roborobo: Expecting " << message_length << " bytes" << std::endl;
            boost::asio::streambuf message_buf;
            size_t bytes_read = boost::asio::read(socket, message_buf.prepare(message_length), error);
            assert(bytes_read == message_length);
            message_buf.commit(bytes_read);
            message = make_string(message_buf);
            message_buf.consume(bytes_read);
        }
        std::cout << "roborobo: message received" << std::endl;
        return message;
    }

    void NetworkInterface::close()
    {
        socket.close();
    }

    void NetworkInterface::connect(std::string ip, unsigned short port)
    {
        endpoint = tcp::endpoint(boost::asio::ip::address::from_string(ip), port);
        if (socket.is_open())
        {
            close();
        }
        socket.connect(endpoint);
    }

}