#ifndef COMMS_HPP
#define COMMS_HPP
#define DEBUG_NOMCU false
#define COMMSDEBUG false
#include <boost/asio.hpp>

class Comms
{
public:
    Comms() = default;
    Comms(std::string portname)
    {
        serial_port_name = portname;
        boost::asio::io_service io;
        serial = std::make_unique<boost::asio::serial_port>(io, serial_port_name);
    }

    void send_command(std::string message) 
    {

        if(DEBUG_NOMCU) {
            std::cout << "(send command) message: " << message << std::endl;
        }
        else {  
            if(COMMSDEBUG) {
                std::cout << "(send_command) Serial message: " << message << std::endl;
            }

            // Set the baud rate, character size, flow control, and parity options.
            serial->set_option(boost::asio::serial_port_base::baud_rate(115200));
            serial->set_option(boost::asio::serial_port_base::character_size(8));
            serial->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
            serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            boost::asio::write(*serial, boost::asio::buffer(message.c_str(), message.size()));
        }
    }

    void serial_read_func()
    {
        // setup async serial read
        boost::asio::io_context io_context;
        boost::asio::serial_port serial_read(io_context, "/dev/ttyACM0");
        
        // CONFIGURE NON-BLOCKING READS
        boost::asio::serial_port_base::flow_control flow_control(boost::asio::serial_port_base::flow_control::none);
        boost::asio::serial_port_base::parity parity(boost::asio::serial_port_base::parity::none);
        boost::asio::serial_port_base::stop_bits stop_bits(boost::asio::serial_port_base::stop_bits::one);
        boost::asio::serial_port_base::character_size char_size(8);
        
        boost::asio::streambuf buffer;
        //char data[1024];
        //std::string data;
        //boost::asio::async_read_until(serial_read, boost::asio::dynamic_buffer(data, sizeof(data)), '\n', //buffer, '\n',
        boost::asio::async_read_until(serial_read, buffer, '\n',
                    //[data](const boost::system::error_code& error, size_t bytes_transferred) {
                    [&](const boost::system::error_code& error, size_t bytes_transferred) {
                        if(!error) {
                            std::istream input_stream(&buffer);
                            std::string line;
                            //std::string line2;

                            std::getline(input_stream, line);
                            //serial_line_read = line;
                            std::cout << line;
                            
                            //std::getline(input_stream, line);
                            //serial_line_read = line;
                            
                            //std::cout << "in async_read_until callback" << std::endl;
                            //std::cout << "Received: " << line << std::endl;
                            if(COMMSDEBUG) {
                                std::cout << "(async_read_until) bytes transferred: " << bytes_transferred << std::endl;
                            }
                        }
                        else {
                            std::cerr << "Error: " << error.message() << std::endl;
                        }
                    });
        io_context.run();
    }
private:
    std::string serial_port_name;
    boost::asio::serial_port* serial;
};

#endif