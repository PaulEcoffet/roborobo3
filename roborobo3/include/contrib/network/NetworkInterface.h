/**
 * @author Paul Ecoffet <paul.ecoffet@isir.upmc.fr>
 * @date 2017-12-05
 */

#ifndef ROBOROBO3_NETWORKINTERFACE_H
#define ROBOROBO3_NETWORKINTERFACE_H

#include <string>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;


namespace network
{

    /**
     * Simple network interface to send and receive plain text messages over socket with a server. Messages are sent
     * prefixed with their lengths.
     */
    class NetworkInterface
    {
    public:
        NetworkInterface();
        NetworkInterface(const std::string& ip, unsigned short port);

        void connect(std::string ip, unsigned short port);

        /**
         * Send the message `msg` prefixed with its length in a 8 character hexadecimal number
         * Ex:
         * "0000007Message"
         *
         * @param msg The message to send
         */
        void sendMessage(std::string msg);

        /**
         * Read the message sent by the server. The message must be preceded by a 8 character hexadecimal number.
         * If the connection is closed by the server, then receiveMessage returns an empty string
         *
         * @return The message received from the server, or an empty string if the connection is closed by the server.
         */
        std::string receiveMessage();

        /**
         * Close the connection with the server.
         */
        void close();

        bool is_open()
        {
            return socket.is_open();
        }

    protected:
        boost::asio::io_service io_service;
        tcp::endpoint endpoint;
        tcp::socket socket;

    };

}
#endif //ROBOROBO3_NETWORKINTERFACE_H
