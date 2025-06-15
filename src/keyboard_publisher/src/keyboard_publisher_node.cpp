#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include "keyboard_handler/keyboard_handler.hpp"
#include "keyboard_handler/keyboard_handler_base.hpp"

using namespace std::chrono_literals;

class KeyboardPrintNode : public rclcpp::Node
{
public:
  KeyboardPrintNode()
  : Node("keyboard_publisher_node")
  {
    using KeyCode = KeyboardHandlerBase::KeyCode;
    using KeyModifiers = KeyboardHandlerBase::KeyModifiers;

    std::vector<KeyCode> keys_to_listen = {
      KeyCode::A, KeyCode::B, KeyCode::C, KeyCode::D, KeyCode::E, KeyCode::F,
      KeyCode::G, KeyCode::H, KeyCode::I, KeyCode::J, KeyCode::K, KeyCode::L,
      KeyCode::M, KeyCode::N, KeyCode::O, KeyCode::P, KeyCode::Q, KeyCode::R,
      KeyCode::S, KeyCode::T, KeyCode::U, KeyCode::V, KeyCode::W, KeyCode::X,
      KeyCode::Y, KeyCode::Z
    };

    std::unordered_map<KeyCode, int> key_to_number;

    for (size_t i = 0; i < keys_to_listen.size(); ++i) {
        key_to_number[keys_to_listen[i]] = static_cast<int>(i) + 1;
    }

    keycode_publisher_ = this->create_publisher<std_msgs::msg::UInt32>("/keyboard", 10);

    handler_ = std::make_shared<KeyboardHandler>();

    for (KeyCode key : keys_to_listen)
    {
      handler_->add_key_press_callback(
        [this, key](KeyCode pressed_key, KeyModifiers mods) {
          if (pressed_key == key) {
            auto msg = std_msgs::msg::UInt32();
            msg.data = 1 + static_cast<int>(key) - static_cast<int>(KeyCode::A);
            keycode_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published key code: %u", msg.data);
          }
        },
        key,
        KeyModifiers::NONE
      );
    }

    RCLCPP_INFO(this->get_logger(), "Keyboard handler node started. Press or release 'k' to publish.");
  }

private:
  std::shared_ptr<KeyboardHandler> handler_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr keycode_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardPrintNode>());
  rclcpp::shutdown();
  return 0;
}


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

