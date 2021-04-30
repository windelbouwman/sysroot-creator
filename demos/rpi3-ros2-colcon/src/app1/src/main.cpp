
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>

class MyNode : public rclcpp::Node {
    public:
        MyNode(std::string node_name);
        virtual ~MyNode();
    
    private:
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_publisher_sine_wave;
        void on_timer();
};

MyNode::MyNode(std::string node_name)
  : rclcpp::Node(node_name)
{
    rclcpp::QoS qos_profile(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    m_publisher_sine_wave =
        this->create_publisher<std_msgs::msg::Float64>("~/sine_wave",
                                                        qos_profile);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(150),
                                      std::bind(&MyNode::on_timer, this));
}

MyNode::~MyNode()
{
}

void MyNode::on_timer()
{
    const double t_now = (double)this->now().nanoseconds() / 1.0e9;
    std_msgs::msg::Float64 msg;
    msg.data = 3.14 * sin(t_now);
    m_publisher_sine_wave->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>("demo_app1");
    std::cout << "Entering App 1" << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
