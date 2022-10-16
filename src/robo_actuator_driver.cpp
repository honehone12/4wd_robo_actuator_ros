#include "robo_actuator/robo_actuator_driver.hpp"

///////////////////////////////////////////////
// this node sends cmd_vel to arduino.
// note : this node does not send cmd in realtime.
// cmd transfer depends on node's own loop rate.
// because sending cmd in non-organized way is not 
// safe with arduino so far.
// means a lot of nonsence packets are sent for
// checking communication is doing nice.  
// check performance and delay of actuater.


namespace robo_actuator
{    
using namespace std::chrono_literals;

void SerialPortBuffer::open
(
    const std::string& port_name,
    unsigned int baud_rate,
    size_t backend_buffer_size = 15,
    ParityType parity = ParityType::none
)
{
    serial_port.open(port_name);
    serial_port.set_option(
        boost::asio::serial_port_base::baud_rate(baud_rate)
    );
    // ensure Arduino default 8N1
    serial_port.set_option(
        boost::asio::serial_port_base::character_size(8u)
    );
    serial_port.set_option(
        boost::asio::serial_port_base::parity(parity)
    );
    serial_port.set_option(
        boost::asio::serial_port_base::stop_bits(StopBitsType::one)
    );

    backend_read_buffer.resize(backend_buffer_size);
}

bool SerialPortBuffer::update(size_t target_byte_arryay_length)
{
    if(!updateBuffer())
    {
        return false;
    }

    unsigned int header_count(0u);
    unsigned int false_count(0u);
    short ret_byte(0);
    while (header_count < SERIAL_COMMUNICATION_HEADER_SIZE)
    {
        if(frontend_read_buffer.size() > 0)
        {
            ret_byte = readByte(); 
            if(ret_byte < 0)
            {
                return false;
            }
            
            if(ret_byte == 0xff)
            {
                header_count++;
                //printf("[Info!] read header\n");
            }
            else
            {
                //printf("[Warning!] read unexpected value %d\n", byte_container);
                header_count = 0u;
                false_count++;
                if(false_count >= SERIAL_COMMUNICATION_MAX_FALSE_COUNT)
                {
                    printf("[Error!] exit with time out.");
                    return false;
                }
            }
        }
        else
        {
            if(!updateBuffer())
            {
                return false;
            }
        }
    }

    while(
        frontend_read_buffer.size() < 
            SERIAL_COMMUNICATION_DATA_DESCRIPTION_LEN + target_byte_arryay_length
    )
    {
        if(!updateBuffer())
        {
            return false;
        }
    }
    return true;
}

short SerialPortBuffer::readByte()
{
    if(openCheck())
    {
        if(frontend_read_buffer.size() > 0)
        {
            byte ret_byte = frontend_read_buffer.front();
            frontend_read_buffer.pop_front();
            return (short)ret_byte;
        }
        printf("[Error!] serial port buffer is empty. call update() before read.\n");
    }
    else
    {
        printf("[Error!] serial port is closed.\n");
    }
    return -1;
}

bool SerialPortBuffer::read(
    std::vector<uint8_t>& read_buffer,
    unsigned int byte_array_length
)
{
    if(openCheck())
    {
        if(frontend_read_buffer.size() >= byte_array_length)
        {
            if(!read_buffer.empty())
            {
                read_buffer.clear();
            }

            read_buffer.insert(
                read_buffer.begin(),
                frontend_read_buffer.begin(),
                frontend_read_buffer.begin() + byte_array_length
            );
            frontend_read_buffer.erase(
                frontend_read_buffer.begin(),
                frontend_read_buffer.begin() + byte_array_length
            );
            return true;
        }
        else
        {
            printf("[Error!] serial port buffer is not enough to read. call update() before read.\n");
        }
    }
    
    return false;
}

bool SerialPortBuffer::write(
    const std::vector<uint8_t>& write_buffer,
    const SerialDataDescription& data_description
)
{
    if(openCheck())
    {
        if(write_buffer.size() != data_description.byte_array_length)
        {
            printf("[Error!] write buffer size is different from data description.\n");
            return false;
        }
        if(
            data_description.byte_array_length != 
                data_description.type_size * data_description.type_length
        )
        {
            printf("[Error!] data description has inconsistency.\n");
            return false;
        }
        if(
            (data_description.data_type == SERIAL_COMMUNICATION_FLOAT_TYPE && 
                data_description.type_size != sizeof(float)) ||
            (data_description.data_type == SERIAL_COMMUNICATION_BYTE_TYPE &&
                data_description.type_size != sizeof(char))
        )
        {
            printf("[Error!] data type and data size have inconsistency.\n");
            return false;
        }

        backend_write_buffer.clear();
        backend_write_buffer.insert(
            backend_write_buffer.begin(),
            header.begin(),
            header.end()
        );
        backend_write_buffer.insert(
            backend_write_buffer.end(),
            {
                data_description.byte_array_length,
                data_description.type_size,
                data_description.type_length,
                data_description.data_type
            }
        );
        backend_write_buffer.insert(
            backend_write_buffer.end(),
            write_buffer.begin(),
            write_buffer.end()
        );

        boost::system::error_code err_code;
        size_t written_size(0);
        try
        {
            written_size = boost::asio::write(
                serial_port,
                boost::asio::buffer(backend_write_buffer),
                err_code
            );    
        }
        catch(const boost::system::system_error& e)
        {
            printf("%s", e.what());
            printf("%s \n", err_code.message().c_str());
            return false;
        }
        
        // printf("[Info!] == write raw : start ==\n");
        // for (size_t i = 0; i < backend_write_buffer.size(); i++)
        // {
        //     printf("%d\n", backend_write_buffer[i]);
        // }
        // printf("[Info!] == write raw : end ==\n");
        if(written_size == 0)
        {
            printf("[Error!] serial port returned zero. seems an error occured in writing.\n");
            printf("%s \n", err_code.message().c_str());
            return false;
        }
    }
    return true;
}

// is there a way to check readbuffer ??
// otherwise this can causes read read deadlock;
bool SerialPortBuffer::updateBuffer()
{
    if(openCheck())
    {
        boost::system::error_code err_code;
        size_t read_size(0);   
        try
        {
            read_size = boost::asio::read(
                serial_port,
                boost::asio::buffer(backend_read_buffer),
                err_code
            );
        }
        catch(const boost::system::system_error& e)
        {
            printf("%s", e.what());
            printf("%s \n", err_code.message().c_str());
            return false;
        }
        
        if(read_size == 0)
        {
            printf("[Error!] serial port returned zero. seems an error occured in reading.\n");
            printf("%s \n", err_code.message().c_str());
        }
        else
        {
            // printf("[Info!] == backend read raw : start ==\n");
            // for (size_t i = 0; i < backend_read_buffer.size(); i++)
            // {
            //     printf("%x\n", backend_read_buffer[i]);
            // }
            // printf("[Info!] == backend read raw : end ==\n");
            frontend_read_buffer.insert(
                frontend_read_buffer.end(),
                backend_read_buffer.begin(),
                backend_read_buffer.end()
            );
            //printf("[Info!] backend buffer size %ld \n", backend_buffer.size());
            return true;
        }
    }
    return false;
}

CmdVelTransfer::CmdVelTransfer() : 
    Node("robo_actuator_driver"),
    message_flg(false)
{
    write_data_buffer.resize(8, 0); // 2 * sizeof(float)
    dummy_cmd_buffer.resize(8, 0);
    FloatTypeExchanger linear_exchager{ CMD_VEL_TRANSFER_DUMMY_CMD };
    FloatTypeExchanger angular_exchanger{ CMD_VEL_TRANSFER_DUMMY_CMD };
    for (size_t i = 0; i < 4; i++)
    {
        dummy_cmd_buffer[i] = linear_exchager.bin[i];
        dummy_cmd_buffer[i + 4] = angular_exchanger.bin[i];
    }

    std::string port("/dev/ttyUSB0");
    declare_parameter("port_name", port);
    get_parameter("port_name", port);

    serial_port_buffer.open(port, 115200u),

    subscription = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        std::bind(
            &CmdVelTransfer::onMessageRecieved,
            this,
            std::placeholders::_1
        )
    );
    timer = create_wall_timer(
        CMD_VEL_TRANSFER_TARGET_FPS_DELAY,
        std::bind(&CmdVelTransfer::onTimer, this)
    );

    RCLCPP_INFO(get_logger(), "init serial port %s", port.c_str());
}

void CmdVelTransfer::onMessageRecieved(geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel)
{
    FloatTypeExchanger linear_exchager;
    FloatTypeExchanger angular_exchanger;
    linear_exchager.float_data = cmd_vel->linear.x;
    angular_exchanger.float_data = cmd_vel->angular.z;
    for (size_t i = 0; i < 4; i++) // sizeof(float)
    {
        write_data_buffer[i] = linear_exchager.bin[i];
        write_data_buffer[i + 4] = angular_exchanger.bin[i];
    }
    message_flg = true;
}

void CmdVelTransfer::tryReadFeedback()
{
    if(serial_port_buffer.update(float_data_description.byte_array_length))
    {
        bool is_expected_data_description(true);
        short ret_byte(0);
        for (size_t i = 0; i < SERIAL_COMMUNICATION_DATA_DESCRIPTION_LEN; i++)
        {
            ret_byte = serial_port_buffer.readByte(); 
            if(ret_byte >= 0)
            {
                byte b((uint)ret_byte);
                if(b != float_data_description.bin[i])
                {
                    is_expected_data_description = false;
                    RCLCPP_WARN(get_logger(), "unexpected byte %x", b);
                }
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "failed to read.");
                return;
            }
        }

        if(
            is_expected_data_description &&
                serial_port_buffer.read(
                    read_data_buffer,
                    float_data_description.byte_array_length
                )
        )
        {
            FloatTypeExchanger feedback_ex_linear;
            FloatTypeExchanger feedback_ex_angular;
            for (size_t i = 0; i < 4; i++)
            {
                feedback_ex_linear.bin[i] = read_data_buffer[i];
                feedback_ex_angular.bin[i] = read_data_buffer[i + 4];
            }

            ////////////////////////////////////////////////////
            // feedback check.
            if(std::isfinite(feedback_ex_linear.float_data))
            {
                if(abs(feedback_ex_linear.float_data) < CMD_VEL_TRANSFER_DUMMY_CMD)
                {
                    RCLCPP_INFO(get_logger(), "got feedback linear %f", feedback_ex_linear.float_data);
                }
                // else
                // {
                //     RCLCPP_INFO(get_logger(), "got dummy linear.");
                // }
            }
            if(std::isfinite(feedback_ex_angular.float_data))
            {
                if(abs(feedback_ex_angular.float_data) < CMD_VEL_TRANSFER_DUMMY_CMD)
                {
                    RCLCPP_INFO(get_logger(), "got feedback angular %f", feedback_ex_angular.float_data);
                }
                // else
                // {
                //     RCLCPP_INFO(get_logger(), "got dummy angular.");
                // }
            }
        }
        //RCLCPP_INFO(get_logger(), "port buffer size %ld", serial_port_buffer.size());
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "failed to read");
        return;
    }
}

void CmdVelTransfer::onTimer()
{
    tryReadFeedback();

    if(hasMessageRecieved())
    {
        writeCMD(); 
    }
    else
    {
        writeDummy();
    }
}

} //ns

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto cmd_vel_transfer =
        std::make_shared<robo_actuator::CmdVelTransfer>();
    
    rclcpp::spin(cmd_vel_transfer);

    cmd_vel_transfer->dispose();
    rclcpp::shutdown();
    return 0;
}

