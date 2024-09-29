#include "rclcpp/rclcpp.hpp"
#include "msg_interfaces/msg/local_navigation_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/node.hpp>
#include <fstream>
#include <thread>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Motion_control : public rclcpp::Node
{
public:
    Motion_control(/* args */) : Node("pid")
    {
        RCLCPP_INFO(get_logger(), "运动控制节点启动成功!");

        this->declare_parameter("PID_P", Kp);
        this->declare_parameter("PID_I", Ki);
        this->declare_parameter("PID_D", Kd);
        this->declare_parameter("Car_type", Chassis_type);
        this->declare_parameter("Speculative_time", speculate_time);
        this->declare_parameter("Wheel_base", wheelBase);
        this->declare_parameter("Wheel_pitch", wheelpitch);

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(5ms, std::bind(&Motion_control::motion_control_callback, this));

        status_sub_ = this->create_subscription<msg_interfaces::msg::LocalNavigationInterface>("local_nav_data", 1, std::bind(&Motion_control::status_callback, this, _1));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<msg_interfaces::msg::LocalNavigationInterface>::SharedPtr status_sub_;

    void motion_control_callback();
    void status_callback(msg_interfaces::msg::LocalNavigationInterface::SharedPtr statu_data);

    void PID_control(float target_value, float current_value);
    void Ackerman_chassis(double linearVelocity, double angularVelocity, double wheelBase);
    void Differential_chassis(double linearVel, double angularVel, double wheelpitch);

    float wheelBase = 0.3;  // 轮距（单位：m）
    float wheelpitch = 0.25; // 轮间距（单位：m）

    int8_t Central_offset; // 中心偏转值
    float speed;           // 订阅线速度
    uint8_t Running_state; // 运行状态
    float timestamp_sub;   // 时间戳

    float linear_velocity_x; // 左轮线速度
    float linear_velocity_y; // 右轮线速度
    float angular_velocity;  // 角速度
    float timestamp;         // 时间戳

    float Kp = 0.049;          // 比例系数
    float Ki = 0;          // 积分系数
    float Kd = 0.8;          // 微分系数
    float current_value = 0; // 当前值
    float lastError = 0.0;   // 上一次的偏差
    float sumError = 0.0;    // 偏差的累积量

    int Chassis_type = 0; // 底盘类型 0阿克曼 1差速

    int speculate_time; // 无磁条时推测时长

    int dadelayed = 0;
};

/****************************************
函数功能：主回调
入口参数：无
返回  值：无
*****************************************/
void Motion_control::motion_control_callback()
{
    this->get_parameter("PID_P", Kp);
    this->get_parameter("PID_I", Ki);
    this->get_parameter("PID_D", Kd);
    this->get_parameter("Car_type", Chassis_type);
    this->get_parameter("Speculative_time", speculate_time);
    this->get_parameter("Wheel_base", wheelBase);
    this->get_parameter("Wheel_pitch", wheelpitch);

    PID_control(0, Central_offset);

    if (Chassis_type == 0)
    {
        Ackerman_chassis(speed, angular_velocity, wheelBase);
    }
    else if (Chassis_type == 1)
    {
        Differential_chassis(speed, angular_velocity, wheelpitch);
    }

    if (Running_state != 4)
    {
        dadelayed = 0;
    }

    switch (Running_state)
    {
    case 0:
        linear_velocity_x = 0.0;
        linear_velocity_y = 0.0;
        angular_velocity = 0.0;

        break;
    case 1:
        linear_velocity_x = 0.0;
        linear_velocity_y = 0.0;

        break;
    case 2:

        break;
    case 3:
        linear_velocity_x = -linear_velocity_x;
        linear_velocity_y = -linear_velocity_y;

        break;
    case 4:
        if (dadelayed > speculate_time * 1000 / 5)
        {
            linear_velocity_x = 0.0;
            linear_velocity_y = 0.0;
        }
        dadelayed++;

        break;
    }
}

/****************************************
函数功能：计算阿克曼底盘左右轮速度
入口参数：线速度 角速度 轮距
返回  值：无
*****************************************/
void Motion_control::Ackerman_chassis(double linearVelocity, double angularVelocity, double wheelBase)
{
    double radius = wheelBase / std::tan(angularVelocity);
    linear_velocity_x = linearVelocity;
    // linear_velocity_y = linearVelocity * (1.0 + wheelBase / (2.0 * radius));

    // std::cout << "Left wheel velocity: " << linear_velocity_x << " m/s" << std::endl;
    // std::cout << "Right wheel velocity: " << linear_velocity_y << " m/s" << std::endl;
}

/****************************************
函数功能：计算差速底盘左右轮速度
入口参数：线速度 角速度 轮间距
返回  值：无
*****************************************/
void Motion_control::Differential_chassis(double linearVel, double angularVel, double wheelpitch)
{
    // 计算左右轮速度
    linear_velocity_x = linearVel - (angularVel * wheelpitch / 2);
    linear_velocity_y = linearVel + (angularVel * wheelpitch / 2);

    // std::cout << "Left wheel velocity: " << linear_velocity_x << " m/s" << std::endl;
    // std::cout << "Right wheel velocity: " << linear_velocity_y << " m/s" << std::endl;
}

/****************************************
函数功能：磁条居中PID函数
入口参数：目标值 当前值
返回  值：无
*****************************************/
void Motion_control::PID_control(float target_value, float current_value)
{
    double deviation = target_value - current_value;
    sumError += deviation;
    sumError *= 0.9;
    double derivative = deviation - lastError;
    double output = Kp * deviation + Ki * sumError + Kd * derivative;
    lastError = deviation;

    this->angular_velocity = 0.0 + output;
}

void Motion_control::status_callback(msg_interfaces::msg::LocalNavigationInterface::SharedPtr statu_data)
{
    Central_offset = statu_data->centeroffset;
    speed = statu_data->velocity;
    Running_state = statu_data->running_state;
    timestamp_sub = statu_data->timestamp;

    auto motion_ontrol = geometry_msgs::msg::Twist();
    rclcpp::Time now = get_clock()->now();
    timestamp = now.seconds();

    motion_ontrol.linear.x = speed;
   
    motion_ontrol.angular.z = angular_velocity;
    // RCLCPP_INFO(get_logger(), "Current timestamp: %f seconds", timestamp);
    publisher_->publish(motion_ontrol);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Motion_control>());
    rclcpp::shutdown();
    return 0;
}