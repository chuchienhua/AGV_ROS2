#include "teensycommunication/teensycommunication.hpp"

Teensycommunication::Teensycommunication() : Node("teensycommunication")
{
    this->declare_parameter<std::string>("device_port", "/dev/teensy4");
    this->get_parameter("device_port", port_name_);

    emergencyStopSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "tdk_base_controller/emergency_stop", 10,
        std::bind(&Teensycommunication::emergencyStopCallback, this, std::placeholders::_1));

    onesitearrivedSub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/onesite_arrived", 10,
        std::bind(&Teensycommunication::onesitearrivedCallback, this, std::placeholders::_1));

    button_receiver_Pub_ = this->create_publisher<std_msgs::msg::UInt8>("/button_receiver", 10);

    connectionCheckTimer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&Teensycommunication::connectionCheckCallback, this));

    communicationTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&Teensycommunication::communicationCallback, this));
}

void Teensycommunication::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    emergencyStopFlag = msg->data;
}

void Teensycommunication::onesitearrivedCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    onesitearrived = msg->data;
}

void Teensycommunication::connectionCheckCallback()
{
    if (!serialPort.isOpen())
    {
        try
        {
            serialPort.setPort(port_name_);
            serialPort.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
            serialPort.setTimeout(timeout);
            serialPort.open();
            serialPort.flushInput();
            serialPort.flushOutput();
            vx = 0;
            vy = 0;
            w = 0;
            button_status = 0x00;
            onesitearrived = 0x00;
            RCLCPP_INFO(this->get_logger(), "Connected to teensy successfully!");
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port " << port_name_ << ", " << e.what());
        }
    }
}

void Teensycommunication::communicationCallback()
{
    std_msgs::msg::UInt8 button_status_msg;

    if (!serialPort.isOpen())
        return;

    CtrlMsg ctrlMsg;
    ctrlMsg.sof = 0xAA;
    ctrlMsg.eof = 0xCD;

    ctrlMsg.controlData.vx = vx;
    ctrlMsg.controlData.onesitearrived = onesitearrived;
    ctrlMsg.controlData.button_status = button_status;

    ctrlMsg.checksum = calculateChecksum(ctrlMsg.data, DATA_LEN);
    try
    {
        serialPort.write((uint8_t *)&ctrlMsg, FRAME_LEN);
        uint8_t rxBuf[FRAME_LEN];
        serialPort.read(rxBuf, FRAME_LEN);
        CtrlMsg *rxMsg = (CtrlMsg *)rxBuf;
        if (((rxMsg->sof & 0xFE) == 0xAA) && (rxMsg->eof == 0xCD))
        {
            button_status = rxMsg->controlData.button_status;
            button_status_msg.data = rxMsg->controlData.button_status;
            button_receiver_Pub_->publish(button_status_msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Error response! SOF = %2X, EOF = %2X", rxMsg->sof, rxMsg->eof);
        }
    }
    catch (serial::SerialException &e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error when communicating to teensy: " << e.what());
        serialPort.close();
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error when communicating to teensy: " << e.what());
        serialPort.close();
    }
}

uint16_t Teensycommunication::calculateChecksum(const uint8_t *data, int len)
{
    uint16_t sum = 0;
    for (int i = 0; i < len / 2; i++)
    {
        sum ^= *(((uint16_t *)data) + i);
    }
    return sum;
}
