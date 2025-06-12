#include "fingers_node.hpp"

Hands::Hands(const rclcpp::NodeOptions &node_options) : Node("hands_node", node_options)
{
    //tcp
    port_ = 23456;
    server_thread_ = std::thread(&Hands::start_tcp_server, this);

    //hand command
    left_hand_cmd_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_hand_cmd", 10);
    right_hand_cmd_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("right_hand_cmd", 10);
    send_hand_cmd_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0), 
                                    std::bind(&Hands::hand_cmd_timer_callback, this));

    left_qpos_.resize(12, 0.0);
    right_qpos_.resize(12, 0.0);

    left_hand_cmd_.name = {            
            "left_hand_L_index_proximal_joint",
            "left_hand_L_index_intermediate_joint",
            "left_hand_L_middle_proximal_joint",
            "left_hand_L_middle_intermediate_joint",
            "left_hand_L_pinky_proximal_joint",
            "left_hand_L_pinky_intermediate_joint",
            "left_hand_L_ring_proximal_joint",
            "left_hand_L_ring_intermediate_joint",
            "left_hand_L_thumb_proximal_yaw_joint",
            "left_hand_L_thumb_proximal_pitch_joint",
            "left_hand_L_thumb_intermediate_joint",
            "left_hand_L_thumb_distal_joint",
        };
    size_t num_left_hand_joints = left_hand_cmd_.name.size();
    left_hand_cmd_.position.resize(num_left_hand_joints, 0.0);

    right_hand_cmd_.name = {        
            "right_hand_R_index_proximal_joint",
            "right_hand_R_index_intermediate_joint",
            "right_hand_R_middle_proximal_joint",
            "right_hand_R_middle_intermediate_joint",
            "right_hand_R_pinky_proximal_joint",
            "right_hand_R_pinky_intermediate_joint",
            "right_hand_R_ring_proximal_joint",
            "right_hand_R_ring_intermediate_joint",
            "right_hand_R_thumb_proximal_yaw_joint",
            "right_hand_R_thumb_proximal_pitch_joint",
            "right_hand_R_thumb_intermediate_joint",
            "right_hand_R_thumb_distal_joint",    
        };
    size_t num_right_hand_joints = right_hand_cmd_.name.size();
    right_hand_cmd_.position.resize(num_right_hand_joints, 0.0);
}

void Hands::start_tcp_server()
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

void Hands::parse_and_print(const std::string &message)
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

    left_qpos_ = parse_pose(parts[0]);
    right_qpos_ = parse_pose(parts[1]);

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

std::vector<double> Hands::parse_pose(const std::string &s)
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

void Hands::hand_cmd_timer_callback()
{
    left_hand_cmd_.header.stamp = this->now();
    left_hand_cmd_.position = left_qpos_;

    left_hand_cmd_publisher_->publish(left_hand_cmd_);

    right_hand_cmd_.header.stamp = this->now();
    right_hand_cmd_.position = right_qpos_;

    right_hand_cmd_publisher_->publish(right_hand_cmd_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Hands>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
