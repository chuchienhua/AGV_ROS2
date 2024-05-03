#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <vector>
#include <sstream>
#include <filesystem>
#include <sys/stat.h> // 包含头文件以使用mkdir
#include <errno.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#define VX_AXIS 1             // 左垂直摇杆
#define VY_AXIS 0             // 左水平摇杆
#define W_AXIS 3              // 右水平摇杆
#define EMERG_BTN 4           // L1按钮
#define JS_CTRL_BTN 5         // R1按钮
#define HOME_BTN 2            // 三角按钮
#define ADD_POINT_BTN 1       // 圆按钮
#define DEL_POINT_BTN 0       // 叉按钮
#define SAVE_POINT_BTN 3      // 方按钮
#define navigation_BTN 9      // 开始按钮
#define stop_navigation_BTN 8 // 后退按钮

#define BTN_DEBOUNCE_TIME 0.1 // 按钮防抖时间（秒）

using std::placeholders::_1;

class CcpJsNode : public rclcpp::Node
{
public:
    CcpJsNode() : Node("ccp_js_node"),
                  lastAddTime(get_clock()->now()),
                  lastDelTime(get_clock()->now()),
                  lastSaveTime(get_clock()->now()),
                  lastNavigationTime(get_clock()->now()),
                  lastStopNavigationTime(get_clock()->now()),
                  lastJsCmdVelTime(get_clock()->now())
    {
        // Publiser / Subscribe
        joyStickSub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&CcpJsNode::joyStickCallback, this, _1));
        cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&CcpJsNode::cmdVelCallback, this, _1));
        amclPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10, std::bind(&CcpJsNode::amclPoseCallback, this, _1));
        cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("omni_base_driver/cmd_vel", 10);
        joycontrolPub = this->create_publisher<std_msgs::msg::Bool>("/joy_enable", 10);
        emergencyStopPub = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
        goalPub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
        savedPosePub = this->create_publisher<visualization_msgs::msg::Marker>("/saved_point", 10);
        navigationPub = this->create_publisher<std_msgs::msg::Int8>("/navigationSend", 10);
        stopnavigationPub = this->create_publisher<std_msgs::msg::Int8>("/stopnavigationSend", 10);

        // parame
        this->declare_parameter<double>("vx_sacler", 0.5);
        this->declare_parameter<double>("vy_sacler", 0.5);
        this->declare_parameter<double>("w_sacler", 3.14 / 2);

        this->get_parameter("vx_sacler", vx_sacler);
        this->get_parameter("vy_sacler", vy_sacler);
        this->get_parameter("w_sacler", w_sacler);

        savedPosePubTimer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CcpJsNode::savedPosePubTimerCallback, this));
    }

private:
    // subscribe callback
    void joyStickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void savedPosePubTimerCallback();
    void goHome();
    void addPoint();
    void delPoint();
    void savePointsToFile();

    // 變數
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joyStickSub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amclPoseSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr joycontrolPub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergencyStopPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr savedPosePub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr navigationPub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr stopnavigationPub;
    rclcpp::TimerBase::SharedPtr savedPosePubTimer;

    double vx_sacler;
    double vy_sacler;
    double w_sacler;
    bool emergencyStopFlag = false;
    bool jsCmdVel = false;
    bool lastJsCmdVel = false;
    bool lastAddBtnState = false;
    bool lastDelBtnState = false;
    bool lastSaveBtnState = false;
    bool lastNavigationState = false;
    bool NavigationState = false;
    bool lastStopNavigationState = false;
    bool StopNavigationState = false;
    geometry_msgs::msg::Pose currentPose;
    std::vector<geometry_msgs::msg::Pose> savedPose;
    rclcpp::Time lastAddTime, lastDelTime, lastSaveTime, lastNavigationTime, lastStopNavigationTime, lastJsCmdVelTime;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CcpJsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// Implementation of callback functions goes here
void CcpJsNode::joyStickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    std_msgs::msg::Bool boolMsg;
    std_msgs::msg::Bool boolJoyMsg;
    std_msgs::msg::Int8 NavigationMsg;
    std_msgs::msg::Int8 StopNavigationMsg;
    rclcpp::Time now = this->now();

    if (!emergencyStopFlag && msg->buttons[EMERG_BTN])
    {
        emergencyStopFlag = true;
        boolMsg.data = true;
        RCLCPP_INFO(this->get_logger(), "Emergency Stop!");
        emergencyStopPub->publish(boolMsg);
    }

    if (msg->buttons[JS_CTRL_BTN] && !lastJsCmdVel && (now - lastJsCmdVelTime).seconds() > BTN_DEBOUNCE_TIME)
    {
        jsCmdVel = !jsCmdVel;
        boolJoyMsg.data = jsCmdVel;
        RCLCPP_INFO(this->get_logger(), "Joystick control = %s", jsCmdVel ? "enabled" : "disabled");
        joycontrolPub->publish(boolJoyMsg);
        lastJsCmdVelTime = now;
    }
    lastJsCmdVel = msg->buttons[JS_CTRL_BTN];

    if (msg->buttons[HOME_BTN])
    {
        goHome();
    }

    if (msg->buttons[ADD_POINT_BTN] && !lastAddBtnState && (now - lastAddTime).seconds() > BTN_DEBOUNCE_TIME)
    {
        addPoint();
        lastAddTime = now;
    }
    lastAddBtnState = msg->buttons[ADD_POINT_BTN];

    if (msg->buttons[DEL_POINT_BTN] && !lastDelBtnState && (now - lastDelTime).seconds() > BTN_DEBOUNCE_TIME)
    {
        delPoint();
        lastDelTime = now;
    }
    lastDelBtnState = msg->buttons[DEL_POINT_BTN];

    if (msg->buttons[SAVE_POINT_BTN] && !lastSaveBtnState && (now - lastSaveTime).seconds() > BTN_DEBOUNCE_TIME)
    {
        savePointsToFile();
        lastSaveTime = now;
    }
    lastSaveBtnState = msg->buttons[SAVE_POINT_BTN];

    if (!jsCmdVel)
        return;

    if (jsCmdVel)
    {
        geometry_msgs::msg::Twist twistMsg;
        twistMsg.linear.x = msg->axes[VX_AXIS] * vx_sacler;
        twistMsg.linear.y = msg->axes[VY_AXIS] * vy_sacler;
        twistMsg.angular.z = msg->axes[W_AXIS] * w_sacler;

        if (twistMsg.linear.x != 0 || twistMsg.linear.y != 0 || twistMsg.angular.z != 0)
        {
            cmdVelPub->publish(twistMsg);
        }
    }
}

void CcpJsNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (jsCmdVel)
        cmdVelPub->publish(*msg);
}

void CcpJsNode::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    currentPose.position = msg->pose.pose.position;
    currentPose.orientation = msg->pose.pose.orientation;
}

void CcpJsNode::savedPosePubTimerCallback()
{
    visualization_msgs::msg::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = this->now();
    points.id = 0;
    points.type = visualization_msgs::msg::Marker::POINTS;
    points.color.r = 1.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0f;
    points.scale.x = 0.25f;
    points.scale.y = 0.25f;

    for (auto &pose : savedPose)
    {
        geometry_msgs::msg::Point p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = 0;
        points.points.push_back(p);
    }

    savedPosePub->publish(points);
}

void CcpJsNode::goHome()
{
    RCLCPP_INFO(this->get_logger(), "Going Home!");
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.w = 1;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;

    jsCmdVel = false;
    goalPub->publish(msg);
}

void CcpJsNode::addPoint()
{
    RCLCPP_INFO(this->get_logger(), "Adding Node %ld!", savedPose.size());
    savedPose.push_back(currentPose);
}

void CcpJsNode::delPoint()
{
    if (!savedPose.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Deleting Point!");
        savedPose.pop_back();
    }
}

void CcpJsNode::savePointsToFile()
{
    std::string workspace_path = "/home/eddie/ros2_ws/src";
    std::filesystem::path dir_path = workspace_path; // Convert string to filesystem path
    dir_path /= "ccp_js_node/saved_points";          // Correctly append relative path

    if (!std::filesystem::exists(dir_path))
    {
        std::filesystem::create_directories(dir_path); // Create directory if it does not exist
    }

    auto now = std::chrono::system_clock::now();
    auto t_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t_c), "%Y-%m-%d-%H-%M-%S");
    std::string filename = ss.str() + ".csv";

    std::filesystem::path file_path = dir_path / filename; // Correct usage of '/' operator with path

    std::ofstream file(file_path); // std::ofstream can take std::filesystem::path directly since C++17
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
        return;
    }

    file << "ID,X,Y,Z,Qx,Qy,Qz,Qw\n";
    int id = 0;
    for (const auto &pose : savedPose)
    {
        file << id++ << ","
             << pose.position.x << ","
             << pose.position.y << ","
             << pose.position.z << ","
             << pose.orientation.x << ","
             << pose.orientation.y << ","
             << pose.orientation.z << ","
             << pose.orientation.w << "\n";
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "Saved points to %s", file_path.c_str());
}
