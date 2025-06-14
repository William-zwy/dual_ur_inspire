#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <iostream>

#include <sensor_msgs/msg/joint_state.hpp>

class Hands: public rclcpp::Node
{
    public:
    explicit Hands(const rclcpp::NodeOptions & node_options);
    ~Hands() = default;

    private:
    void parse_and_print(const std::string &message);
    void start_tcp_server();
    void hand_cmd_timer_callback();
    std::vector<double> parse_pose(const std::string &s);
    void init_params();

    int port_;
    bool show_tcp_data_;
    bool running_ = true;
    std::thread server_thread_;
    std::vector<double> left_qpos_;
    std::vector<double> right_qpos_;  

    rclcpp::TimerBase::SharedPtr send_hand_cmd_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_hand_cmd_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_hand_cmd_publisher_;
    sensor_msgs::msg::JointState left_hand_cmd_;
    sensor_msgs::msg::JointState right_hand_cmd_;

};