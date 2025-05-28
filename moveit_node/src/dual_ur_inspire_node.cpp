#include "dual_ur_inspire_node.hpp"

Dual_ur_inspire::Dual_ur_inspire(const rclcpp::NodeOptions &node_options) : Node("dual_ur_inspire_node", node_options)
{
    //tcp
    port_ = 12345;
    server_thread_ = std::thread(&Dual_ur_inspire::start_tcp_server, this);

    //hand command
    // left_hand_cmd_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_hand_cmd", 10);
    // right_hand_cmd_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("right_hand_cmd", 10);
    // send_hand_cmd_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0), 
    //                                 std::bind(&Dual_ur_inspire::hand_cmd_timer_callback, this));
        
    left_pose_.resize(7, 0.0);
    right_pose_.resize(7, 0.0);
    left_qpos_.resize(12, 0.0);
    right_qpos_.resize(12, 0.0);

    left_hand_cmd_.name = {            
            "left_hand_L_thumb_proximal_yaw_joint",
            "left_hand_L_thumb_proximal_pitch_joint",
            "left_hand_L_thumb_intermediate_joint",
            "left_hand_L_thumb_distal_joint",
            "left_hand_L_index_proximal_joint",
            "left_hand_L_index_intermediate_joint",
            "left_hand_L_middle_proximal_joint",
            "left_hand_L_middle_intermediate_joint",
            "left_hand_L_ring_proximal_joint",
            "left_hand_L_ring_intermediate_joint",
            "left_hand_L_pinky_proximal_joint",
            "left_hand_L_pinky_intermediate_joint",
        };
    size_t num_left_hand_joints = left_hand_cmd_.name.size();
    left_hand_cmd_.position.resize(num_left_hand_joints, 0.0);

    right_hand_cmd_.name = {      
            "right_hand_R_thumb_proximal_yaw_joint",
            "right_hand_R_thumb_proximal_pitch_joint",
            "right_hand_R_thumb_intermediate_joint",
            "right_hand_R_thumb_distal_joint",      
            "right_hand_R_index_proximal_joint",
            "right_hand_R_index_intermediate_joint",
            "right_hand_R_middle_proximal_joint",
            "right_hand_R_middle_intermediate_joint",
            "right_hand_R_ring_proximal_joint",
            "right_hand_R_ring_intermediate_joint",
            "right_hand_R_pinky_proximal_joint",
            "right_hand_R_pinky_intermediate_joint",
        };
    size_t num_right_hand_joints = right_hand_cmd_.name.size();
    right_hand_cmd_.position.resize(num_right_hand_joints, 0.0);

    //moveit
    left_arm_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0), 
                                    std::bind(&Dual_ur_inspire::left_arm_timer_callback, this));
    right_arm_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0), 
                                    std::bind(&Dual_ur_inspire::right_arm_timer_callback, this));
}

void Dual_ur_inspire::moveit_init()
{
    left_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                                shared_from_this(), "left_arm");
    right_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                                shared_from_this(), "right_arm");
}

void Dual_ur_inspire::start_tcp_server()
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
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Client connected.");

    char buffer[1024];
    while (running_)
    {
        ssize_t bytes_read = read(client_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "Client disconnected or read error");
            break;
        }

        buffer[bytes_read] = '\0';
        std::string message(buffer);
        parse_and_print(message);
    }

    close(client_fd);
    close(server_fd);
}

void Dual_ur_inspire::parse_and_print(const std::string &message)
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

    if (parts.size() < 2)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid message: %s", trimmed.c_str());
        return;
    }

    // left_pose_ = parse_pose(parts[0]);
    // right_pose_ = parse_pose(parts[1]);
    std::vector<double> new_left_pose = parse_pose(parts[0]);
    std::vector<double> new_right_pose = parse_pose(parts[1]);

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        if (is_pose_changed(new_left_pose, last_left_pose_)) {
            left_pose_ = new_left_pose;
            last_left_pose_ = new_left_pose;
            left_new_goal_received_ = true;
        }

        if (is_pose_changed(new_right_pose, last_right_pose_)) {
            right_pose_ = new_right_pose;
            last_right_pose_ = new_right_pose;
            right_new_goal_received_ = true;
        }
    }

    if (left_pose_.size() != 7 || right_pose_.size() != 7)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid pose size");
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "Left Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    //             left_pose_[0], left_pose_[1], left_pose_[2],
    //             left_pose_[3], left_pose_[4], left_pose_[5], left_pose_[6]);

    // RCLCPP_INFO(this->get_logger(), "Right Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    //             right_pose_[0], right_pose_[1], right_pose_[2],
    //             right_pose_[3], right_pose_[4], right_pose_[5], right_pose_[6]);

    if (parts.size() >= 4)
    {
        left_qpos_ = parse_pose(parts[2]);
        right_qpos_ = parse_pose(parts[3]);

        std::ostringstream left_qpos_stream, right_qpos_stream;

        for (size_t i = 0; i < left_qpos_.size(); ++i)
        {
            left_qpos_stream << std::fixed << std::setprecision(3) << left_qpos_[i];
            if (i != left_qpos_.size() - 1)
                left_qpos_stream << ", ";
        }

        for (size_t i = 0; i < right_qpos_.size(); ++i)
        {
            right_qpos_stream << std::fixed << std::setprecision(3) << right_qpos_[i];
            if (i != right_qpos_.size() - 1)
                right_qpos_stream << ", ";
        }

        // RCLCPP_INFO(this->get_logger(), "Left Qpos: [%s]", left_qpos_stream.str().c_str());
        // RCLCPP_INFO(this->get_logger(), "Right Qpos: [%s]", right_qpos_stream.str().c_str());
    }
}

std::vector<double> Dual_ur_inspire::parse_pose(const std::string &s)
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

std::vector<double> Dual_ur_inspire::add_poistion_cmd(const std::vector<double>& qpos)
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

// void Dual_ur_inspire::hand_cmd_timer_callback()
// {
//     left_hand_cmd_.header.stamp = this->now();
//     // std::vector<double> left_hand_position_cmd(6);
//     // left_hand_position_cmd = add_poistion_cmd(left_qpos_);
//     // left_hand_cmd_.position = left_hand_position_cmd;
//     left_hand_cmd_.position = left_qpos_;

//     left_hand_cmd_publisher_->publish(left_hand_cmd_);

//     right_hand_cmd_.header.stamp = this->now();
//     // std::vector<double> right_hand_position_cmd(6);
//     // right_hand_position_cmd = add_poistion_cmd(right_qpos_);
//     // right_hand_cmd_.position = right_hand_position_cmd;
//     right_hand_cmd_.position = right_qpos_;

//     right_hand_cmd_publisher_->publish(right_hand_cmd_);
// }

bool Dual_ur_inspire::is_pose_changed(const std::vector<double>& a, const std::vector<double>& b, double tol) 
{
    if (a.size() != b.size()) return true;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > tol) return true;
    }
    return false;
}

// 输入 [x, y, z, qx, qy, qz, qw]
geometry_msgs::msg::Pose Dual_ur_inspire::translate_pose_to_msg(const std::vector<double>& Quaternion_pose)
{
    geometry_msgs::msg::Pose goal_pose;

    goal_pose.position.x = Quaternion_pose[0];  // x
    goal_pose.position.y = Quaternion_pose[1];  // y 
    goal_pose.position.z = Quaternion_pose[2];  // z

    goal_pose.orientation.x = Quaternion_pose[3];  // qx
    goal_pose.orientation.y = Quaternion_pose[4];  // qy
    goal_pose.orientation.z = Quaternion_pose[5];  // qz
    goal_pose.orientation.w = Quaternion_pose[6];  // qw

    return goal_pose;
}

void Dual_ur_inspire::left_arm_timer_callback()
{
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

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(left_move_group_interface_->plan(plan));

        RCLCPP_INFO(this->get_logger(), "Left Planning frame: %s", left_move_group_interface_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "Left End effector link: %s", left_move_group_interface_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "Left Pose reference frame: %s", left_move_group_interface_->getPoseReferenceFrame().c_str());

        if (success)
        {
            left_move_group_interface_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Left Planing failed!");
        }
    }
}

void Dual_ur_inspire::right_arm_timer_callback()
{
    geometry_msgs::msg::Pose right_goal_pose;
    bool triggered = false;

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (right_new_goal_received_) {
            right_new_goal_received_ = false;
            right_goal_pose = translate_pose_to_msg(right_pose_);
            triggered = true;
        }
    }

    if (triggered) {
        right_move_group_interface_->setPoseTarget(right_goal_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(right_move_group_interface_->plan(plan));

        RCLCPP_INFO(this->get_logger(), "Right Planning frame: %s", right_move_group_interface_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "Right End effector link: %s", right_move_group_interface_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "Right Pose reference frame: %s", right_move_group_interface_->getPoseReferenceFrame().c_str());

        if (success)
        {
            right_move_group_interface_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Right Planing failed!");
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dual_ur_inspire>(rclcpp::NodeOptions());
    node->moveit_init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}