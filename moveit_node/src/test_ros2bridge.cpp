#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class TestROS2Bridge : public rclcpp::Node
{
public:
    TestROS2Bridge() : Node("test_ros2bridge")
    {
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("left_hand_cmd", 10);
        
        // Initialize joint state message
        joint_state_.name = {
            "right_hand_R_index_proximal_joint",
            "right_hand_R_index_intermediate_joint",
            "right_hand_R_middle_proximal_joint",
            "right_hand_R_middle_intermediate_joint",
            "right_hand_R_ring_proximal_joint",
            "right_hand_R_ring_intermediate_joint",
            "right_hand_R_pinky_proximal_joint",
            "right_hand_R_pinky_intermediate_joint",
        };

        size_t num_joints = joint_state_.name.size();
        joint_state_.position.resize(num_joints, 0.0);
        
        default_joints_ = {1.0, 1.0, 1.0, 1.0,1.0, 1.0, 1.0, 1.0};
        
        // Calculate max and min joints
        max_joints_.resize(num_joints);
        min_joints_.resize(num_joints);
        for (size_t i = 0; i < num_joints; ++i) {
            max_joints_[i] = default_joints_[i] + 0.5;
            min_joints_[i] = default_joints_[i] - 0.5;
        }
        
        // Store start time
        start_time_ = this->now();
        
        // Create timer
        timer_ = this->create_wall_timer(
            50ms, std::bind(&TestROS2Bridge::timer_callback, this));
    }

private:
    void timer_callback()
    {
        joint_state_.header.stamp = this->now();
        
        auto elapsed = (this->now() - start_time_).seconds();
        
        std::vector<double> joint_position(default_joints_.size());
        for (size_t i = 0; i < default_joints_.size(); ++i) {
            joint_position[i] = std::sin(elapsed) * (max_joints_[i] - min_joints_[i]) * 0.5 + default_joints_[i];
        }
        
        joint_state_.position = joint_position;
        
        // Publish the message
        publisher_->publish(joint_state_);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    sensor_msgs::msg::JointState joint_state_;
    std::vector<double> default_joints_;
    std::vector<double> max_joints_;
    std::vector<double> min_joints_;
    rclcpp::Time start_time_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestROS2Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}