#include "right_arm_node.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

RightArm::RightArm(const rclcpp::NodeOptions &node_options) : Node("right_arm_node", node_options)
{
    port_ = 34567;
    server_thread_ = std::thread(&RightArm::start_tcp_server, this);

    right_pose_.resize(7, 0.0);
    right_arm_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0), 
                                    std::bind(&RightArm::right_arm_timer_callback, this));
}

void RightArm::moveit_init()
{
    right_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                                shared_from_this(), "right_arm");

    right_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            shared_from_this(), "/ur_arm_right_ros2_controller/follow_joint_trajectory");
}

void RightArm::start_tcp_server()
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

void RightArm::parse_and_print(const std::string &message)
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

    std::vector<double> new_right_pose = parse_pose(parts[0]);

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        if (is_pose_changed(new_right_pose, last_right_pose_)) {
            right_pose_ = new_right_pose;
            last_right_pose_ = new_right_pose;
            right_new_goal_received_ = true;
        }
    }

    if (right_pose_.size() != 7)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid pose size");
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "Right Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    //             right_pose_[0], right_pose_[1], right_pose_[2],
    //             right_pose_[3], right_pose_[4], right_pose_[5], right_pose_[6]);
}

std::vector<double> RightArm::parse_pose(const std::string &s)
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

std::vector<double> RightArm::add_poistion_cmd(const std::vector<double>& qpos)
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

bool RightArm::is_pose_changed(const std::vector<double>& a, const std::vector<double>& b, double tol) 
{
    if (a.size() != b.size()) return true;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > tol) return true;
    }
    return false;
}

// 输入 [x, y, z, qx, qy, qz, qw]
geometry_msgs::msg::Pose RightArm::translate_pose_to_msg(const std::vector<double>& Quaternion_pose)
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

void RightArm::right_arm_timer_callback()
{
    // RCLCPP_INFO(this->get_logger(), "Right callback thread id: %ld", std::hash<std::thread::id>{}(std::this_thread::get_id()));
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

        right_move_group_interface_->setMaxVelocityScalingFactor(0.5);  // 50% 最大速度
        right_move_group_interface_->setMaxAccelerationScalingFactor(0.5);  // 50% 最大加速度

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(right_move_group_interface_->plan(plan));

        RCLCPP_INFO(this->get_logger(), "Right Planning frame: %s", right_move_group_interface_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "Right End effector link: %s", right_move_group_interface_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "Right Pose reference frame: %s", right_move_group_interface_->getPoseReferenceFrame().c_str());

        if (success)
        {
            if (!right_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_ERROR(this->get_logger(), "Right arm action server not available");
                return;
            }

            const auto& traj = plan.trajectory_.joint_trajectory;
            if (!traj.points.empty())
            {
                const auto& t = traj.points.back().time_from_start;
                double total_time = t.sec + 1e-9 * t.nanosec;
                RCLCPP_INFO(this->get_logger(), "Right trajectory total duration: %.3f seconds", total_time);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Right trajectory is empty, cannot compute duration.");
            }

            auto goal = control_msgs::action::FollowJointTrajectory::Goal();
            goal.trajectory = plan.trajectory_.joint_trajectory;
            // rclcpp::Time exec_time = this->now() + rclcpp::Duration::from_seconds(1.0);
            rclcpp::Time exec_time = this->now();
            goal.trajectory.header.stamp = exec_time;

            auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
            send_goal_options.goal_response_callback = [this](auto goal_handle) {
                RCLCPP_INFO(this->get_logger(), "Right arm goal accepted");
            };
            send_goal_options.result_callback = [this](const auto& result) {
                RCLCPP_INFO(this->get_logger(), "Right arm execution completed");
            };

            right_action_client_->async_send_goal(goal, send_goal_options);

            // right_move_group_interface_->asyncExecute(plan);
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

    auto node = std::make_shared<RightArm>(rclcpp::NodeOptions());
    node->moveit_init();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}