#include "left_arm_node.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <fstream>

LeftArm::LeftArm(const rclcpp::NodeOptions &node_options) : Node("left_arm_node", node_options)
{
    init_params();

    server_thread_ = std::thread(&LeftArm::start_tcp_server, this);

    left_pose_.resize(7, 0.0);
    left_arm_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0), 
                                    std::bind(&LeftArm::left_arm_timer_callback, this));
    left_arm_cmd_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_arm_cmd_TracIK", 10);
    controller_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/ur_arm_left_ros2_controller/commands", 10);

    ee_init_point_ = {0.4,0.5,1.8,0.707,0,0,0.707};
    hand_init_point_ = {0.3,0.5,1.1,0.707,0,0,0.707};
    trans_ratio_ = 1.2;
    desired_angle_.data = {0,0,0,0,0,0};

    left_arm_cmd_.name = {        
            "left_shoulder_pan_joint",
            "left_shoulder_lift_joint",
            "left_elbow_joint",
            "left_wrist_1_joint",
            "left_wrist_2_joint",
            "left_wrist_3_joint"  
        };
    size_t num_left_arm_joints = left_arm_cmd_.name.size();
    left_arm_cmd_.position.resize(num_left_arm_joints, 0.0);

    joint_seed_ = KDL::JntArray(6);

    KDL::Tree tree;

    std::ifstream file(urdf_path_);
    if (!file)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to open URDF file at: %s", urdf_path_.c_str());
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

    if (!tree.getChain("stand", "left_ee_link", kdl_chain_)) {
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

    if(debugging_)
    {
        idx = 0;
        for (const auto& segment : kdl_chain_.segments) {
            const KDL::Joint& joint = segment.getJoint();
            if (joint.getType() == KDL::Joint::None) continue;

            RCLCPP_INFO(this->get_logger(), "Joint %u (%s): min = %.6f, max = %.6f",
                        idx, joint.getName().c_str(), joint_min_(idx), joint_max_(idx));
            ++idx;
        }
    }

    ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(kdl_chain_, joint_min_, joint_max_, 0.05, 1e-5, TRAC_IK::Distance);

}

void LeftArm::init_params()
{
    this->declare_parameter<int>("tcp_port", 45678);
    this->declare_parameter<std::string>("urdf_path", "");
    this->declare_parameter<std::string>("chain_root", "");
    this->declare_parameter<std::string>("chain_tip", "");
    this->declare_parameter<float>("robot_arm_length", 0.8);
    this->declare_parameter<float>("human_arm_length", 0.6);
    this->declare_parameter<float>("z_init", 1.0);
    this->declare_parameter<float>("Z_bias", 1.6);
    this->declare_parameter<bool>("debugging", false);
    this->declare_parameter<bool>("show_tcp_data", false);

    port_ = this->get_parameter("tcp_port").as_int();
    urdf_path_ = this->get_parameter("urdf_path").as_string();
    chain_root_ = this->get_parameter("chain_root").as_string();
    chain_tip_ = this->get_parameter("chain_tip").as_string();
    robot_arm_length_ = this->get_parameter("robot_arm_length").as_double();
    human_arm_length_ = this->get_parameter("human_arm_length").as_double();
    z_init_ = this->get_parameter("z_init").as_double();
    Z_bias_ = this->get_parameter("Z_bias").as_double();
    debugging_ = this->get_parameter("debugging").as_bool();
    show_tcp_data_ = this->get_parameter("show_tcp_data").as_bool();
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

    if(show_tcp_data_)
    {
        RCLCPP_INFO(this->get_logger(), "left Pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    left_pose_[0], left_pose_[1], left_pose_[2],
                    left_pose_[3], left_pose_[4], left_pose_[5], left_pose_[6]);
    }
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

bool LeftArm::is_pose_changed(const std::vector<double>& a, const std::vector<double>& b, double tol) 
{
    if (a.size() != b.size()) return true;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > tol) return true;
    }
    return false;
}

// 输入 [x, y, z, qx, qy, qz, qw]
geometry_msgs::msg::Pose LeftArm::translate_pose_to_msg(const std::vector<double>& pose)
{
    geometry_msgs::msg::Pose goal_pose;

    goal_pose.position.x = robot_arm_length_*(human_arm_length_+pose[0])/human_arm_length_;  // x
    goal_pose.position.y = pose[1];  // y 
    goal_pose.position.z = z_init_+pose[2]-Z_bias_;  // z

    goal_pose.orientation.x = pose[3];  // qx
    goal_pose.orientation.y = pose[4];  // qy
    goal_pose.orientation.z = pose[5];  // qz
    goal_pose.orientation.w = pose[6];  // qw

    return goal_pose;
}

void LeftArm::left_arm_timer_callback()
{
    geometry_msgs::msg::Pose left_goal_pose;
    bool triggered = false;

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (left_new_goal_received_) {
            left_new_goal_received_ = false;
            left_goal_pose = translate_pose_to_msg(left_pose_);
            // left_goal_pose = get_target_pose(left_pose_);
            triggered = true;
        }
    }

    if (triggered) {
        std::vector<double> ikJointValues = manipulatorIK(left_goal_pose);
        send_joint_cmd(ikJointValues);
    }

}

void LeftArm::send_joint_cmd(const std::vector<double>& pose)
{    
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data = pose;
    // left_arm_cmd_.position = pose;

    if(debugging_)
    {
        RCLCPP_INFO(this->get_logger(), "Command: [%f %f %f %f %f %f]",
        pose[0], pose[1], pose[2],
        pose[3], pose[4], pose[5]);
    }

    controller_cmd_pub_->publish(cmd_msg);
    // left_arm_cmd_publisher_->publish(left_arm_cmd_);
}

geometry_msgs::msg::Pose LeftArm::get_target_pose(const std::vector<double>& pose)
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

std::vector<double> LeftArm::manipulatorIK(geometry_msgs::msg::Pose target_pose)
{
    std::vector<double> ikJointValues;
    KDL::JntArray return_joints;
    double joint_angle_change;

    if(debugging_)
    {
            RCLCPP_INFO(this->get_logger(), "target_pose - Position: [%f, %f, %f], Orientation: [%f, %f, %f, %f]",
            target_pose.position.x,  // x position
            target_pose.position.y,  // y position
            target_pose.position.z,  // z position
            target_pose.orientation.x,  // x quaternion
            target_pose.orientation.y,  // y quaternion
            target_pose.orientation.z,  // z quaternion
            target_pose.orientation.w   // w quaternion
            );
    }

    double it[] = {target_pose.position.x, target_pose.position.y, target_pose.position.z};
    memcpy(desired_end_effector_pose_.p.data, it, sizeof(it));
    desired_end_effector_pose_.M = KDL::Rotation::Quaternion(
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z,
    target_pose.orientation.w
    );
    joint_seed_.data << desired_angle_.data[0],desired_angle_.data[1],desired_angle_.data[2],desired_angle_.data[3],desired_angle_.data[4],desired_angle_.data[5];
    
    if(debugging_)
    {
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

    auto node = std::make_shared<LeftArm>(rclcpp::NodeOptions());

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}