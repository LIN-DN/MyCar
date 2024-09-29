#include <bitset>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "msg_interfaces/msg/local_navigation_interface.hpp"

class MagnetSubscriberNode : public rclcpp::Node
{
public:
    MagnetSubscriberNode() : Node("magnet_subscriber_node")
    {

        // 创建订阅者，指定回调函数
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "magnet_data", rclcpp::SensorDataQoS(), std::bind(&MagnetSubscriberNode::magnetCallback, this, std::placeholders::_1));

        subscription_rfid = this->create_subscription<std_msgs::msg::String>(
            "rfid_ori_data", 10, std::bind(&MagnetSubscriberNode::rfidCallback, this, std::placeholders::_1));
        //
        publisher_ = this->create_publisher<msg_interfaces::msg::LocalNavigationInterface>("local_nav_data", 10);
        publisher_rfid = this->create_publisher<std_msgs::msg::String>("rfid_data", 10);
        RCLCPP_INFO(this->get_logger(), "Magnetic 节点启动");
    }

private:
    int hexToD(std::string hexString)
    {
        try
        {
            // 使用std::stoi进行转换
            int decimalNumber = std::stoi(hexString, 0, 16);

            // 打印结果
            // std::cout << "16进制字符串 " << hexString << " 转换为10进制: " << decimalNumber << std::endl;
            return decimalNumber;
        }
        catch (const std::invalid_argument &e)
        {

            return -1;
        }
        catch (const std::out_of_range &e)
        {

            return -1;
        }

        return -1;
    }
    std::string hexToBinary(std::string hexString)
    {
        try
        {
            unsigned long long decimalValue = std::stoull(hexString, nullptr, 16);
            std::bitset<32> binary(decimalValue);

            // 将二进制位集合转换为字符串
            std::string binaryString = binary.to_string();

            // 反转字符串
            // std::reverse(binaryString.begin(), binaryString.end());

            // 返回反转后的字符串
            return binaryString;
        }
        catch (const std::invalid_argument &)
        {
            throw std::runtime_error("Invalid hexadecimal string");
        }
    }
    // 订阅回调函数
    void magnetCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string orgin_sensor = msg->data.substr(8, 8);
        if (orgin_sensor != "00000000")
        {
            ct = hexToBinary(orgin_sensor);
            if (magnet_state != 0)
            {
                Motion_control(magnet_state);
                auto pubMsg = msg_interfaces::msg::LocalNavigationInterface();
                printf("cv %d \n",current_value);
                pubMsg.centeroffset = 16 - current_value;
                pubMsg.velocity = vel;
                pubMsg.running_state = run_state;
                publisher_->publish(pubMsg);
            }
        }
        else
        {
            auto pubMsg = msg_interfaces::msg::LocalNavigationInterface();
            pubMsg.centeroffset = 0;
            pubMsg.velocity = 0.0;
            pubMsg.running_state = 1;
            publisher_->publish(pubMsg);
        }
    }

    void rfidCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string orgin_header = msg->data.substr(0, 8);
        if (orgin_header == "0d3a5210")
        {
            std::string rfid_hex = msg->data.substr(8, 2);
            rfid = hexToD(rfid_hex);
            if (rfid != -1 && rfid != 0)
            {
                switch (rfid)
                {
                case 201:
                    magnet_state = 1;
                    /* code */
                    break;
                case 202:
                    magnet_state = 2;
                    break;
                case 203:
                    magnet_state = 3;
                    break;
                case 204:
                    if (magnet_state == 2)
                    {
                        magnet_state = 3;
                    }
                    break;

                default:
                    break;
                }

                if (rfid < 40)
                {
                    auto rfid_msg = std_msgs::msg::String();
                    rfid_msg.data = std::to_string(rfid);
                    publisher_rfid->publish(rfid_msg);
                }
            }
        }
    }

    void Motion_control(int i) // 底盘寻磁选择
    {
        // Select_Mode==1 巡左磁 Select_Mode==2 巡中磁Select_Mode==3 巡右磁

        int sensor_number = 0;
        int sensor_buffer[31] = {0};
        int count = 0;

        for (int i = 0; i < 31; i++)
        {
            if (ct[i] == 1)
            {
                ++sensor_number;
            }
        }

        if (sensor_number < 25)
        {
            if (ct[0] == '1' && ct[1] == '0' && ct[2] == '0')
            {
                sensor_buffer[count] = 0;
                count++;
            }
            if (ct[0] == '1' && ct[1] == '1' && ct[2] == '0')
            {
                sensor_buffer[count] = 1;
                count++;
            }
            for (int i = 0; i < 29; i++)
            {
                if (ct[i] == '1' && ct[i + 2] == '1' && ct[i + 3] == '0')
                {
                    sensor_buffer[count] = i + 1;
                    count++;
                }
            }
            if (ct[29] == '1' && ct[30] == '1' && ct[31] == '1')
            {
                sensor_buffer[count] = 29;
                count++;
            }
            else if (ct[29] == '0' && ct[30] == '1' && ct[31] == '1')
            {
                sensor_buffer[count] = 30;
                count++;
            }
            else if (ct[29] == '0' && ct[30] == '0' && ct[31] == '1')
            {
                sensor_buffer[count] = 31;
                count++;
            }
        }

        if (i == 2) // 巡中磁
        {
            if (count == 1)
            {
                // Steering_angle(190+(sensor_buffer[0]-16)*Steering_ratio);
                current_value = sensor_buffer[0];
            }
            else if (count == 2)
            {
                int jja = 0;
                jja = sensor_buffer[0] - 18;

                int jjb = 0;
                jjb = sensor_buffer[1] - 18;

                if (jja < 0 && jjb < 0)
                {
                    if (jja < jjb)
                    {
                        // Steering_angle(190+(sensor_buffer[1]-16)*Steering_ratio);
                        current_value = sensor_buffer[1];
                    }
                }
                else if (jja < 0 || jjb < 0)
                {
                    if (jja * -1 < jjb)
                    {
                        // Steering_angle(190+(sensor_buffer[0]-16)*Steering_ratio);
                        current_value = sensor_buffer[0];
                    }
                    else if (jja * -1 > jjb)
                    {
                        // Steering_angle(190+(sensor_buffer[1]-16)*Steering_ratio);
                        current_value = sensor_buffer[1];
                    }
                }
                else if (jja > 0 && jjb > 0)
                {
                    if (jja < jjb)
                    {
                        // Steering_angle(190+(sensor_buffer[0]-16)*Steering_ratio);
                        current_value = sensor_buffer[0];
                    }
                    else if (jja > jjb)
                    {
                        // Steering_angle(190+(sensor_buffer[1]-16)*Steering_ratio);
                        current_value = sensor_buffer[1];
                    }
                }
            }
            else if (count == 3)
            {
                // Steering_angle(190+(sensor_buffer[1]-16)*Steering_ratio);
                current_value = sensor_buffer[1];
            }
            // Steering_angle(190+(sensor_buffer[0]-16)*Steering_ratio);
        }
        else if (i == 3) // 巡左磁
        {
            // Steering_angle(190+(sensor_buffer[0]-16)*Steering_ratio);
            current_value = sensor_buffer[0];
        }
        else if (i == 1) // 巡右磁
        {
            int max = sensor_buffer[0];
            int idx = 0;
            for (int i = 0; i <= 9; i++)
            {
                if (max < sensor_buffer[i])
                {
                    max = sensor_buffer[i];
                    idx = i;
                }
            }
            // Steering_angle(190+(sensor_buffer[idx]-16)*Steering_ratio);
            current_value = sensor_buffer[idx];
        }

        // std::cout << "磁条当前位置：" << current_value << std::endl;
    }

    // 订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_rfid;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // 发布者
    rclcpp::Publisher<msg_interfaces::msg::LocalNavigationInterface>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_rfid;
    int current_value;
    std::string ct;
    int magnet_state = 3;
    float vel = 0.15;
    int run_state = 2;
    int rfid;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MagnetSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}