#include <memory>
#include <string>
#include <vector>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class FilteredJointStatePublisher : public rclcpp::Node
{
public:
  FilteredJointStatePublisher()
  : Node("filtered_joint_state_publisher")
  {
    filtered_joints_ = {"left_shoulder_pan_joint", "left_shoulder_lift_joint","left_elbow_joint","left_wrist_1_joint","left_wrist_2_joint","left_wrist_3_joint",
                        "right_shoulder_pan_joint", "right_shoulder_lift_joint","right_elbow_joint","right_wrist_1_joint","right_wrist_2_joint","right_wrist_3_joint"};
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states_raw", 10,
      std::bind(&FilteredJointStatePublisher::jointStateCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto filtered_msg = sensor_msgs::msg::JointState();
    filtered_msg.header = msg->header;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string &joint_name = msg->name[i];
      if (filtered_joints_.count(joint_name)) {
        filtered_msg.name.push_back(joint_name);

        if (i < msg->position.size())
          filtered_msg.position.push_back(msg->position[i]);

        if (i < msg->velocity.size())
          filtered_msg.velocity.push_back(msg->velocity[i]);

        if (i < msg->effort.size())
          filtered_msg.effort.push_back(msg->effort[i]);
      }
    }

    pub_->publish(filtered_msg);
  }

  std::unordered_set<std::string> filtered_joints_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilteredJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
