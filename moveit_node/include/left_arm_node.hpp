#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

class LeftArm: public rclcpp::Node
{
    public:
    explicit LeftArm(const rclcpp::NodeOptions & node_options);
    ~LeftArm() = default;
    void moveit_init();

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
    std::vector<double> add_poistion_cmd(const std::vector<double>& qpos);

    bool is_pose_changed(const std::vector<double>& a, const std::vector<double>& b, double tol = 1e-3);
    geometry_msgs::msg::Pose translate_pose_to_msg(const std::vector<double>& Quaternion_pose);
    void left_arm_timer_callback();

    //tcp
    int port_;
    bool running_ = true;
    std::thread server_thread_;
    std::vector<double> left_pose_;

    //moveit
    rclcpp::TimerBase::SharedPtr left_arm_timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_move_group_interface_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr left_action_client_;

    std::vector<double> last_left_pose_;
    std::atomic<bool> left_new_goal_received_ = false;

    std::mutex pose_mutex_;

};