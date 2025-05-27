#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <iostream>

class TcpServerNode : public rclcpp::Node
{
public:
    TcpServerNode()
        : Node("tcp_server_node")
    {
        port_ = 12345;
        server_thread_ = std::thread(&TcpServerNode::start_server, this);
    }

    ~TcpServerNode()
    {
        running_ = false;
        if (server_thread_.joinable())
        {
            server_thread_.join();
        }
    }

private:
    int port_;
    bool running_ = true;
    std::thread server_thread_;

    void start_server()
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

    void parse_and_print(const std::string &message)
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

        std::vector<double> left_pose = parse_pose(parts[0]);
        std::vector<double> right_pose = parse_pose(parts[1]);

        if (left_pose.size() != 7 || right_pose.size() != 7)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid pose size");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Left Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    left_pose[0], left_pose[1], left_pose[2],
                    left_pose[3], left_pose[4], left_pose[5], left_pose[6]);

        RCLCPP_INFO(this->get_logger(), "Right Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    right_pose[0], right_pose[1], right_pose[2],
                    right_pose[3], right_pose[4], right_pose[5], right_pose[6]);

        if (parts.size() >= 4)
        {
            std::vector<double> left_qpos = parse_pose(parts[2]);
            std::vector<double> right_qpos = parse_pose(parts[3]);

            std::ostringstream left_qpos_stream, right_qpos_stream;

            for (size_t i = 0; i < left_qpos.size(); ++i)
            {
                left_qpos_stream << std::fixed << std::setprecision(3) << left_qpos[i];
                if (i != left_qpos.size() - 1)
                    left_qpos_stream << ", ";
            }

            for (size_t i = 0; i < right_qpos.size(); ++i)
            {
                right_qpos_stream << std::fixed << std::setprecision(3) << right_qpos[i];
                if (i != right_qpos.size() - 1)
                    right_qpos_stream << ", ";
            }

            RCLCPP_INFO(this->get_logger(), "Left Qpos: [%s]", left_qpos_stream.str().c_str());
            RCLCPP_INFO(this->get_logger(), "Right Qpos: [%s]", right_qpos_stream.str().c_str());
        }
    }

    std::vector<double> parse_pose(const std::string &s)
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
