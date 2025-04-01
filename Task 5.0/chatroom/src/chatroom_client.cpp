#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>
#include <thread>

class ChatClient : public rclcpp::Node {
public:
    ChatClient(std::string name) : Node(name), username_(name) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatroom", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatroom", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.rfind(username_ + ": ", 0) != 0) { 
                    RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
                }
            });
    }

    void send_message() {
        while (rclcpp::ok()) {
            std::string input;
            std::cout << "[" << username_ << "]: ";
            std::getline(std::cin, input);
            if (input.empty()) continue;

            auto msg = std_msgs::msg::String();
            msg.data = username_ + ": " + input;
            publisher_->publish(msg);
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string username_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    if (argc < 2) {
        std::cerr << "Usage: ros2 run chatroom chatroom_client <username>" << std::endl;
        return 1;
    }

    auto client = std::make_shared<ChatClient>(argv[1]);
    std::thread input_thread(&ChatClient::send_message, client);
    rclcpp::spin(client);
    input_thread.join();
    rclcpp::shutdown();
    return 0;
}

