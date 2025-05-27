
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

    private:
    void start_tcp_server();
    void parse_and_print(const std::string &message);
    std::vector<double> parse_pose(const std::string &s);

    //tcp
    int port_;
    bool running_ = true;
    std::thread server_thread_;
};