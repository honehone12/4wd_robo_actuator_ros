#ifndef ACTUATOR_DRIVER_HPP_
#define ACTUATOR_DRIVER_HPP_

#include "boost/asio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <vector>
#include <deque>
#include <cmath>

#define SERIAL_COMMUNICATION_MAX_FALSE_COUNT 100
#define SERIAL_COMMUNICATION_HEADER_SIZE 8
#define SERIAL_COMMUNICATION_DATA_DESCRIPTION_LEN 4

#define SERIAL_COMMUNICATION_BYTE_TYPE 0x00
#define SERIAL_COMMUNICATION_FLOAT_TYPE 0x01

//////////////////////////////////////////
// if these values are changed, 
// arduino should be also changed.
#define CMD_VEL_TRANSFER_TARGET_FPS_DELAY /*67ms*/ 33ms /*16ms*/ // 0.016sec=60FPS
#define CMD_VEL_TRANSFER_DUMMY_CMD 9.999f

namespace robo_actuator
{

using byte = u_int8_t;
using ParityType = boost::asio::serial_port_base::parity::type;
using StopBitsType = boost::asio::serial_port_base::stop_bits;

union FloatTypeExchanger
{
    float float_data;
    uint8_t bin[sizeof(float)];
};

// can be better with this setting data.
// this serial port class can know many things in advance with this. 
//ex. update() can check data description.
union SerialDataDescription
{
    struct
    {
        uint8_t byte_array_length;
        uint8_t type_size;
        uint8_t type_length;
        uint8_t data_type;
    };
    uint8_t bin[4];
};

static const std::array<uint8_t, 8> header 
{
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff 
};

class SerialPortBuffer
{
private:
    boost::asio::io_service io_service;
    boost::asio::serial_port serial_port;
    std::deque<byte> frontend_read_buffer;
    std::vector<byte> backend_read_buffer;
    std::vector<byte> backend_write_buffer;

public:
    SerialPortBuffer() : serial_port(io_service)
    { }
    ~SerialPortBuffer()
    { }

    void open(
        const std::string& port_name,
        unsigned int baud_rate,
        size_t backend_buffer_size,
        ParityType parity
    );

    size_t size()
    {
        return frontend_read_buffer.size();
    }

    bool available()
    {
        return !frontend_read_buffer.empty();
    }

    bool update(size_t target_byte_arryay_length);
    short readByte();
    
    bool read(
        std::vector<byte>& read_buffer,
        unsigned int byte_array_length
    );
    bool write(
        const std::vector<byte>& write_buffer,
        const SerialDataDescription& data_description
    );

    void close()
    {
        if(serial_port.is_open())
        {
            frontend_read_buffer.clear();
            serial_port.close();       
        }
    }

private:
    bool openCheck()
    {
        if(serial_port.is_open())
        {
            return true;
        }
        else
        {
            printf("serial port is closed.");

            return false;
        }
    }

    bool updateBuffer();
};

class CmdVelTransfer : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    rclcpp::TimerBase::SharedPtr timer;
    SerialPortBuffer serial_port_buffer;
    SerialDataDescription float_data_description
    {
        8, 4, 2, SERIAL_COMMUNICATION_FLOAT_TYPE
    };
    std::vector<byte> write_data_buffer;
    std::vector<byte> dummy_cmd_buffer;
    std::vector<byte> read_data_buffer;
    bool message_flg;

public:
    CmdVelTransfer();
    ~CmdVelTransfer()
    { }
    
    void onMessageRecieved(geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel);
    void onTimer();
    void tryReadFeedback();

    void writeCMD()
    {
        if(!serial_port_buffer.write(
            write_data_buffer, float_data_description))
        {
            RCLCPP_ERROR(get_logger(), "failed to write cmd_vel.");
        }
        message_flg = false;
    }
    
    void writeDummy()
    {
        if(!serial_port_buffer.write(
            dummy_cmd_buffer, float_data_description))
        {
            RCLCPP_ERROR(get_logger(), "failed to write cmd_vel.");
        }
    }

    bool hasMessageRecieved()
    {
        return message_flg;
    }

    void dispose()
    {
        write_data_buffer.clear();
        serial_port_buffer.close();
    }
};

}

#endif

