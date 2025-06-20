#include <rclcpp/rclcpp.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <trac_ik/trac_ik.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <vector>

class LeftArm: public rclcpp::Node
{
    public:
    explicit LeftArm(const rclcpp::NodeOptions & node_options);
    ~LeftArm() = default;

    private:
    void start_tcp_server();
    void parse_and_print(const std::string &message);
    std::vector<double> parse_pose(const std::string &s);
    bool is_pose_changed(const std::vector<double>& a, const std::vector<double>& b, double tol = 1e-3);
    geometry_msgs::msg::Pose translate_pose_to_msg(const std::vector<double>& pose);
    void left_arm_timer_callback();
    std::vector<double> manipulatorIK(geometry_msgs::msg::Pose target_pose);
    geometry_msgs::msg::Pose get_target_pose(const std::vector<double>& pose);
    void send_joint_cmd(const std::vector<double>& pose);
    void init_params();

    //tcp
    int port_;
    bool running_ = true;
    std::thread server_thread_;
    std::vector<double> left_pose_;

    sensor_msgs::msg::JointState left_arm_cmd_;
    rclcpp::TimerBase::SharedPtr left_arm_timer_;
    trajectory_msgs::msg::JointTrajectory traj_msg_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr controller_cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_cmd_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;


    std::vector<double> last_left_pose_;
    std::atomic<bool> left_new_goal_received_ = false;

    std::mutex pose_mutex_;

    bool is_joint_init_ = false;

    std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;
    KDL::Chain kdl_chain_;
    KDL::JntArray joint_min_;
    KDL::JntArray joint_max_;
    KDL::JntArray joint_seed_;
    KDL::Frame desired_end_effector_pose_;

    const double change_threshold_ = 100;
    std_msgs::msg::Float64MultiArray desired_angle_;
    std::vector<double> ee_init_point_;
    std::vector<double> hand_init_point_;
    float trans_ratio_;

    std::string urdf_path_;
    std::string chain_root_;
    std::string chain_tip_;
    double robot_arm_length_;
    double human_arm_length_;
    double z_init_;
    double Z_bias_;
    bool debugging_;
    bool show_tcp_data_;
};