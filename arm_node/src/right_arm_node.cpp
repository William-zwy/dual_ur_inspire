#include "right_arm_node.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <fstream>

RightArm::RightArm(const rclcpp::NodeOptions &node_options) : Node("right_arm_node", node_options)
{
    port_ = 34567;
    server_thread_ = std::thread(&RightArm::start_tcp_server, this);

    right_pose_.resize(7, 0.0);
    right_arm_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0), 
                                    std::bind(&RightArm::right_arm_timer_callback, this));
    right_arm_cmd_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("right_arm_cmd_TracIK", 10);
    controller_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/ur_arm_right_ros2_controller/commands", 10);

    ee_init_point_ = {0.4,0.5,1.8,0.707,0,0,0.707};
    hand_init_point_ = {0.3,0.5,1.1,0.707,0,0,0.707};
    trans_ratio_ = 1.2;
    desired_angle_.data = {0,0,0,0,0,0};

    right_arm_cmd_.name = {        
            "right_shoulder_pan_joint",
            "right_shoulder_lift_joint",
            "right_elbow_joint",
            "right_wrist_1_joint",
            "right_wrist_2_joint",
            "right_wrist_3_joint"  
        };
    size_t num_right_arm_joints = right_arm_cmd_.name.size();
    right_arm_cmd_.position.resize(num_right_arm_joints, 0.0);

    joint_seed_ = KDL::JntArray(6);

    KDL::Tree tree;

    this->declare_parameter<std::string>("urdf_path", "");
    std::string urdf_path = this->get_parameter("urdf_path").as_string();

    std::ifstream file(urdf_path);
    if (!file)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to open URDF file at: %s", urdf_path.c_str());
        return;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string urdf_string = buffer.str();

    urdf::Model model;
    if (!model.initString(urdf_string))
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to parse URDF string to URDF model.");
        return;
    }

    if (!kdl_parser::treeFromUrdfModel(model, tree))
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to parse KDL tree from URDF model.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully loaded KDL tree from URDF.");

    if (!tree.getChain("stand", "right_ee_link", kdl_chain_)) {
        RCLCPP_FATAL(this->get_logger(), "Failed to extract KDL chain from URDF.");
        return;
    }

    unsigned int num_joints = kdl_chain_.getNrOfJoints();
    joint_min_.resize(num_joints);
    joint_max_.resize(num_joints);
    joint_seed_.resize(num_joints);

    unsigned int idx = 0;
    for (const auto& segment : kdl_chain_.segments) {
        const KDL::Joint& joint = segment.getJoint();
        if (joint.getType() == KDL::Joint::None) continue;

        auto urdf_joint = model.getJoint(joint.getName());
        if (urdf_joint && urdf_joint->limits) {
            joint_min_(idx) = urdf_joint->limits->lower;
            joint_max_(idx) = urdf_joint->limits->upper;
        } else {
            joint_min_(idx) = -M_PI;
            joint_max_(idx) = M_PI;
        }
        ++idx;
    }

    idx = 0;
    for (const auto& segment : kdl_chain_.segments) {
        const KDL::Joint& joint = segment.getJoint();
        if (joint.getType() == KDL::Joint::None) continue;

        RCLCPP_INFO(this->get_logger(), "Joint %u (%s): min = %.6f, max = %.6f",
                    idx, joint.getName().c_str(), joint_min_(idx), joint_max_(idx));
        ++idx;
    }

    ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(kdl_chain_, joint_min_, joint_max_, 0.05, 1e-5, TRAC_IK::Distance);

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
    trimmed.erase(trimmed.find_last_not_of("\n\r") + 1);

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
    goal_pose.position.z = 0.916+Quaternion_pose[2]-1.2;  // z
    // goal_pose.position.x = Quaternion_pose[0];  // x
    // goal_pose.position.y = Quaternion_pose[1];  // y 
    // goal_pose.position.z = Quaternion_pose[2];  // z

    goal_pose.orientation.x = Quaternion_pose[3];  // qx
    goal_pose.orientation.y = Quaternion_pose[4];  // qy
    goal_pose.orientation.z = Quaternion_pose[5];  // qz
    goal_pose.orientation.w = Quaternion_pose[6];  // qw

    return goal_pose;
}

void RightArm::right_arm_timer_callback()
{
    geometry_msgs::msg::Pose right_goal_pose;
    bool triggered = false;

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (right_new_goal_received_) {
            right_new_goal_received_ = false;
            right_goal_pose = translate_pose_to_msg(right_pose_);
            // right_goal_pose = get_target_pose(right_pose_);
            triggered = true;
        }
    }

    if (triggered) {
        std::vector<double> ikJointValues = manipulatorIK(right_goal_pose);
        send_joint_cmd(ikJointValues);
    }

}

void RightArm::send_joint_cmd(const std::vector<double>& pose)
{
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = pose;
    right_arm_cmd_.position = pose;

    RCLCPP_INFO(this->get_logger(), "Command: [%f %f %f %f %f %f]",
    right_arm_cmd_.position[0], right_arm_cmd_.position[1], right_arm_cmd_.position[2],
    right_arm_cmd_.position[3], right_arm_cmd_.position[4], right_arm_cmd_.position[5]);

    controller_cmd_pub_->publish(cmd_msg);
    // right_arm_cmd_publisher_->publish(right_arm_cmd_);
}

geometry_msgs::msg::Pose RightArm::get_target_pose(const std::vector<double>& pose)
{
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = ee_init_point_[0] + (pose[0] -  hand_init_point_[0])*trans_ratio_;
    goal_pose.position.y = ee_init_point_[1] + (pose[1] -  hand_init_point_[1])*trans_ratio_;
    goal_pose.position.z = ee_init_point_[2] + (pose[2] -  hand_init_point_[2])*trans_ratio_;
    goal_pose.orientation.x = pose[3];
    goal_pose.orientation.y = pose[4];
    goal_pose.orientation.z = pose[5];
    goal_pose.orientation.w = pose[6];

    return goal_pose;
}

std::vector<double> RightArm::manipulatorIK(geometry_msgs::msg::Pose target_pose)
{
    std::vector<double> ikJointValues;
    KDL::JntArray return_joints;
    double joint_angle_change;

    RCLCPP_INFO(this->get_logger(), "target_pose - Position: [%f, %f, %f], Orientation: [%f, %f, %f, %f]",
            target_pose.position.x,  // x position
            target_pose.position.y,  // y position
            target_pose.position.z,  // z position
            target_pose.orientation.x,  // x quaternion
            target_pose.orientation.y,  // y quaternion
            target_pose.orientation.z,  // z quaternion
            target_pose.orientation.w   // w quaternion
            );

    double it[] = {target_pose.position.x, target_pose.position.y, target_pose.position.z};
    memcpy(desired_end_effector_pose_.p.data, it, sizeof(it));
    desired_end_effector_pose_.M = KDL::Rotation::Quaternion(
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z,
    target_pose.orientation.w
    );
    joint_seed_.data << desired_angle_.data[0],desired_angle_.data[1],desired_angle_.data[2],desired_angle_.data[3],desired_angle_.data[4],desired_angle_.data[5];
    RCLCPP_INFO(this->get_logger(), "Seed: [%f %f %f %f %f %f]",
    joint_seed_(0), joint_seed_(1), joint_seed_(2),
    joint_seed_(3), joint_seed_(4), joint_seed_(5));

    for (int i = 0; i < 3; ++i) {
        RCLCPP_INFO(this->get_logger(), 
            "Rotation matrix row %d: [%.6f, %.6f, %.6f]",
            i,
            desired_end_effector_pose_.M(i, 0),
            desired_end_effector_pose_.M(i, 1),
            desired_end_effector_pose_.M(i, 2));
    }

    int rc = ik_solver_->CartToJnt(joint_seed_, desired_end_effector_pose_, return_joints);
    
    if(rc>=0)
    {
        joint_angle_change = 0;
        for(int i=0;i<6;i++)
        {
            joint_angle_change += (return_joints(i) - desired_angle_.data[i])*(return_joints(i) - desired_angle_.data[i]);
        }
        
        if(joint_angle_change<change_threshold_) 
        {
            for(int i=0;i<6;i++) 
            {
                desired_angle_.data[i] = return_joints.data(i);
            }
                
        }
        else
        {
            for(int i=0;i<6;i++) 
            {
                RCLCPP_WARN(this->get_logger(),"Dramatic change occurred. Change: %f",joint_angle_change);
            }
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),"Did not find IK solution");
    }

    for(int i=0;i<6;i++)
        ikJointValues.push_back(desired_angle_.data[i]);

    return ikJointValues;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RightArm>(rclcpp::NodeOptions());

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}