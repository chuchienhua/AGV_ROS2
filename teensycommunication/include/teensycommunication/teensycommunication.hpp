#ifndef TEENSYCOMMUNICATION_HPP
#define TEENSYCOMMUNICATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>

#define FRAME_LEN 16
#define DATA_LEN 12

using std::uint16_t;
using std::uint8_t;

typedef struct __attribute__((__packed__))
{
    uint8_t sof;
    union
    {
        uint8_t data[DATA_LEN];
        struct
        {
            float vx;
            uint8_t onesitearrived;
            uint8_t button_status;
        } controlData;
    };
    uint16_t checksum;
    uint8_t eof;
} CtrlMsg;

class Teensycommunication : public rclcpp::Node
{
public:
    Teensycommunication();

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergencyStopSub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr onesitearrivedSub_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr button_receiver_Pub_;

    rclcpp::TimerBase::SharedPtr connectionCheckTimer_;
    rclcpp::TimerBase::SharedPtr communicationTimer_;

    std::string port_name_;
    serial::Serial serialPort;
    bool emergencyStopFlag;
    float vx, vy, w;
    uint8_t button_status;
    uint8_t onesitearrived;

    void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void onesitearrivedCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void connectionCheckCallback();
    void communicationCallback();
    uint16_t calculateChecksum(const uint8_t *data, int len);
};

#endif // TEENSYCOMMUNICATION_HPP
