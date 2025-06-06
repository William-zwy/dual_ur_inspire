#include "left_arm_node.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

LeftArm::LeftArm(const rclcpp::NodeOptions &node_options) : Node("left_arm_node", node_options)
{
    port_ = 45678;
    server_thread_ = std::thread(&LeftArm::start_tcp_server, this);

    left_pose_.resize(7, 0.0);
    left_arm_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0), 
                                std::bind(&LeftArm::left_arm_timer_callback, this));
    joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/ur_arm_left_ros2_controller/commands", 10);
}

void LeftArm::moveit_init()
{
    left_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                                shared_from_this(), "left_arm");

    left_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            shared_from_this(), "/ur_arm_left_ros2_controller/follow_joint_trajectory");
}

void LeftArm::start_tcp_server()
{
    while (running_)
    {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port_);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            close(server_fd);
            return;
        }

        if (listen(server_fd, 1) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Listen failed");
            close(server_fd);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for TCP connection on port %d...", port_);

        socklen_t addrlen = sizeof(address);
        int client_fd = accept(server_fd, (struct sockaddr *)&address, &addrlen);
        if (client_fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Accept failed");
            close(server_fd);
            continue;  
        }

        RCLCPP_INFO(this->get_logger(), "Client connected.");

        char buffer[1024];
        bool client_connected = true;
        while (running_ && client_connected)
        {
            ssize_t bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);
            if (bytes_read <= 0)
            {
                RCLCPP_WARN(this->get_logger(), "Client disconnected or read error");
                client_connected = false;
                break;
            }

            buffer[bytes_read] = '\0';
            std::string message(buffer);
            parse_and_print(message);
        }

        close(client_fd);
        close(server_fd);
        RCLCPP_INFO(this->get_logger(), "Connection closed, waiting for new connection...");
    }
}

void LeftArm::parse_and_print(const std::string &message)
{
    std::string trimmed = message;
    trimmed.erase(trimmed.find_last_not_of("\n\r") + 1); // 去除结尾换行符

    std::vector<std::string> parts;
    std::stringstream ss(trimmed);
    std::string segment;
    while (std::getline(ss, segment, ';'))
    {
        parts.push_back(segment);
    }

    if (parts.size() < 1)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid message: %s", trimmed.c_str());
        return;
    }

    std::vector<double> new_left_pose = parse_pose(parts[0]);

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        if (is_pose_changed(new_left_pose, last_left_pose_)) {
            left_pose_ = new_left_pose;
            last_left_pose_ = new_left_pose;
            left_new_goal_received_ = true;
        }

    }

    if (left_pose_.size() != 7)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid pose size");
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "Left Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    //             left_pose_[0], left_pose_[1], left_pose_[2],
    //             left_pose_[3], left_pose_[4], left_pose_[5], left_pose_[6]);

}

std::vector<double> LeftArm::parse_pose(const std::string &s)
{
    std::vector<double> values;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        try
        {
            values.push_back(std::stod(item));
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to parse float: %s", item.c_str());
        }
    }
    return values;
}

std::vector<double> LeftArm::add_poistion_cmd(const std::vector<double>& qpos)
{
    std::vector<double> hand_position_cmd(6);

    hand_position_cmd[0] = qpos[thumb_proximal_yaw_joint];
    hand_position_cmd[1] = qpos[thumb_proximal_pitch_joint];
    hand_position_cmd[2] = qpos[index_proximal_joint];
    hand_position_cmd[3] = qpos[middle_proximal_joint];
    hand_position_cmd[4] = qpos[ring_proximal_joint];
    hand_position_cmd[5] = qpos[pinky_proximal_joint];

    return hand_position_cmd;
}

bool LeftArm::is_pose_changed(const std::vector<double>& a, const std::vector<double>& b, double tol) 
{
    if (a.size() != b.size()) return true;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > tol) return true;
    }
    return false;
}

// 输入 [x, y, z, qx, qy, qz, qw]
geometry_msgs::msg::Pose LeftArm::translate_pose_to_msg(const std::vector<double>& Quaternion_pose)
{
    geometry_msgs::msg::Pose goal_pose;

    goal_pose.position.x = 0.817*(0.6+Quaternion_pose[0])/0.6;  // x
    goal_pose.position.y = Quaternion_pose[1];  // y 
    goal_pose.position.z = 2.034+Quaternion_pose[2]-1.6;  // z

    goal_pose.orientation.x = Quaternion_pose[3];  // qx
    goal_pose.orientation.y = Quaternion_pose[4];  // qy
    goal_pose.orientation.z = Quaternion_pose[5];  // qz
    goal_pose.orientation.w = Quaternion_pose[6];  // qw

    return goal_pose;
}

void LeftArm::left_arm_timer_callback() {
    geometry_msgs::msg::Pose left_goal_pose;
    bool triggered = false;

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (left_new_goal_received_) {
            left_new_goal_received_ = false;
            RCLCPP_INFO(this->get_logger(), "receive goal");
            left_goal_pose = translate_pose_to_msg(left_pose_);
            triggered = true;
        }
    }

    if (triggered) {
        left_move_group_interface_->setPoseTarget(left_goal_pose);

        left_move_group_interface_->setMaxVelocityScalingFactor(0.5);  // 50% 最大速度
        left_move_group_interface_->setMaxAccelerationScalingFactor(0.5);  // 50% 最大加速度

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(left_move_group_interface_->plan(plan));

        RCLCPP_INFO(this->get_logger(), "Left Planning frame: %s", left_move_group_interface_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "Left End effector link: %s", left_move_group_interface_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "Left Pose reference frame: %s", left_move_group_interface_->getPoseReferenceFrame().c_str());

        if (success)
        {

            const auto& traj = plan.trajectory_.joint_trajectory;
            if (!traj.points.empty())
            {
                const auto& t = traj.points.back().time_from_start;
                double total_time = t.sec + 1e-9 * t.nanosec;
                RCLCPP_INFO(this->get_logger(), "Left trajectory total duration: %.3f seconds", total_time);

                const std::vector<double>& joint_positions = traj.points.back().positions;

                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = joint_positions;
                joint_cmd_pub_->publish(cmd_msg);

            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Left trajectory is empty, cannot compute duration.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Left Planing failed!");
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LeftArm>(rclcpp::NodeOptions());
    node->moveit_init();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}