
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>

class Dual_ur_inspire: public rclcpp::Node
{
    public:
    explicit Dual_ur_inspire(const rclcpp::NodeOptions & node_options);
    ~Dual_ur_inspire() = default;

    enum HandJoint
        {
            thumb_proximal_yaw_joint = 0,
            thumb_proximal_pitch_joint,
            thumb_intermediate_joint,
            thumb_distal_joint,
            index_proximal_joint,
            index_intermediate_joint,
            middle_proximal_joint,
            middle_intermediate_joint,
            ring_proximal_joint,
            ring_intermediate_joint,
            pinky_proximal_joint,
            pinky_intermediate_joint,
        };

    private:
    void start_tcp_server();
    void parse_and_print(const std::string &message);
    std::vector<double> parse_pose(const std::string &s);
    std::vector<double> add_poistion_cmd(std::vector<double> qpos);
    void hand_cmd_timer_callback();

    //tcp
    int port_;
    bool running_ = true;
    std::thread server_thread_;
    std::vector<double> left_pose_;
    std::vector<double> right_pose_;
    std::vector<double> left_qpos_;
    std::vector<double> right_qpos_;  

    //ros2bridege
    rclcpp::TimerBase::SharedPtr send_hand_cmd_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_hand_cmd_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_hand_cmd_publisher_;
    sensor_msgs::msg::JointState left_hand_cmd_;
    sensor_msgs::msg::JointState right_hand_cmd_;
};