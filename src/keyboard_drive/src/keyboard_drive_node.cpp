#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>


// | Letter | Value |
// | ------ | ----- |
// | A      | 1     |
// | B      | 2     |
// | C      | 3     |
// | D      | 4     |
// | E      | 5     |
// | F      | 6     |
// | G      | 7     |
// | H      | 8     |
// | I      | 9     |
// | J      | 10    |
// | K      | 11    |
// | L      | 12    |
// | M      | 13    |
// | N      | 14    |
// | O      | 15    |
// | P      | 16    |
// | Q      | 17    |
// | R      | 18    |
// | S      | 19    |
// | T      | 20    |
// | U      | 21    |
// | V      | 22    |
// | W      | 23    |
// | X      | 24    |
// | Y      | 25    |
// | Z      | 26    |


class KeyboardDriveNode : public rclcpp::Node
{
public:
  KeyboardDriveNode()
  : Node("keyboard_drive_node")
  {
    // Declare parameters with default values
    this->declare_parameter<int>("forward", 23);   // W
    this->declare_parameter<int>("release", 19);   // S
    this->declare_parameter<int>("backward", 24);  // X
    this->declare_parameter<int>("speed", 60);
    this->declare_parameter<std::string>("motor_name", "default_motor");

    // Get parameters
    forward_key_ = this->get_parameter("forward").as_int();
    backward_key_ = this->get_parameter("backward").as_int();
    release_key_ = this->get_parameter("release").as_int();
    speed_ = this->get_parameter("speed").as_int() / 100.;
    motor_name_ = this->get_parameter("motor_name").as_string();
    topic_name_ =  "/" + motor_name_ + "/cmd_vel";

    // Setup publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name_, 10);
    arduino_publisher_ = this->create_publisher<std_msgs::msg::String>("/arduino/motor_control", 10);

    // Setup subscriber
    subscriber_ = this->create_subscription<std_msgs::msg::UInt32>(
      "/keyboard", 10,
      std::bind(&KeyboardDriveNode::keyboard_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "KeyboardDriveNode started. Listening to key codes.");
  }

private:
  void keyboard_callback(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    geometry_msgs::msg::Twist twist;
    std_msgs::msg::String arduino_msg;
    std::string direction;

    if (msg->data == forward_key_) {
      twist.linear.x = speed_;
      direction = "F";
      RCLCPP_INFO(this->get_logger(), "Forward command issued");
    } else if (msg->data == backward_key_) {
      twist.linear.x = -speed_;
      direction = "B";
      RCLCPP_INFO(this->get_logger(), "Backward command issued");
    } else if (msg->data == release_key_) {
      twist.linear.x = 0.0;
      direction = "R";
      RCLCPP_INFO(this->get_logger(), "Stop command issued");
    } else {
      return;
    }

    publisher_->publish(twist);

    // Format and publish Arduino command
    arduino_msg.data = motor_name_ + ":" + direction + ":" + std::to_string(static_cast<int>(speed_ * 100));
    arduino_publisher_->publish(arduino_msg);
  }

  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_publisher_;

  uint32_t forward_key_;
  uint32_t backward_key_;
  uint32_t release_key_;
  double speed_;
  std::string motor_name_;
  std::string topic_name_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardDriveNode>());
  rclcpp::shutdown();
  return 0;
}

