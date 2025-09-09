#include <chrono>
#include <iostream>
#include <string>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

using namespace std::chrono_literals;

class LivePositionPublisher : public rclcpp::Node
{
public:
    LivePositionPublisher() : Node("live_position_pub")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&LivePositionPublisher::publish_pose, this));

        x_ = 0.0;
        y_ = 0.0;
        z_ = 2.0;

        RCLCPP_INFO(this->get_logger(), "LivePositionPublisher started. Type x:y:z or 'land', 'arm', 'ready'");
    }

    #include <algorithm>
#include <cctype>

// Helper function to trim spaces
static inline void trim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
        [](unsigned char ch){ return !std::isspace(ch); }));
    s.erase(std::find_if(s.rbegin(), s.rend(),
        [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
}

void run_input_loop()
{
    std::string line;
    while (rclcpp::ok())
    {
        std::getline(std::cin, line);

        // Trim spaces and convert to lowercase
        trim(line);
        std::transform(line.begin(), line.end(), line.begin(), ::tolower);

        if (line.empty()) continue;

        if (line == "land") { publish_land(); continue; }
        if (line == "ready") { publish_offBoard(); continue; }
        if (line == "arm") { publish_arm(); continue; }
        if (line == "exit") {
            RCLCPP_INFO(this->get_logger(), "Exit command received. Shutting down...");
            rclcpp::shutdown();
            break;
        }

        // Try parsing as x:y:z
        double nx, ny, nz;
        try {
            if (parse_line(line, nx, ny, nz)) {
                x_ = nx; y_ = ny; z_ = nz;
                RCLCPP_INFO(this->get_logger(), "New target: %.2f, %.2f, %.2f", x_, y_, z_);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Invalid input. Use x:y:z or commands 'land', 'arm', 'ready' or 'exit'");
            }
        }
        catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(),
                "Parsing failed: %s. Use x:y:z or commands 'land', 'arm', 'ready', 'exit'",
                e.what());
        }
    }
}

private:
    bool parse_line(const std::string &line, double &nx, double &ny, double &nz)
    {
        std::stringstream ss(line);
        std::string token;
        if (!std::getline(ss, token, ':')) return false;
        nx = std::stod(token);
        if (!std::getline(ss, token, ':')) return false;
        ny = std::stod(token);
        if (!std::getline(ss, token, ':')) return false;
        nz = std::stod(token);
        return true;
    }

    void publish_pose()
    {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = x_;
        msg.pose.position.y = y_;
        msg.pose.position.z = z_;
        msg.pose.orientation.w = 1.0;
        pub_->publish(msg);
    }

    void publish_land()
    {
        auto client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        if (!client->wait_for_service(5s)) { RCLCPP_ERROR(this->get_logger(), "SetMode unavailable"); return; }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "AUTO.LAND";

        // ✅ Changed to asynchronous callback (no spin_until_future_complete)
        auto result_future = client->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
            {
                auto response = future.get();
                if (response->mode_sent)
                    RCLCPP_INFO(this->get_logger(), "Landing mode sent successfully");
                else
                    RCLCPP_WARN(this->get_logger(), "Landing mode request failed");
            });
    }

    void publish_offBoard()
    {
        auto client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");    
        if (!client->wait_for_service(5s)) { RCLCPP_ERROR(this->get_logger(), "SetMode unavailable"); return; }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";

        // ✅ Changed to asynchronous callback
        auto result_future = client->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
            {
                auto response = future.get();
                if (response->mode_sent)
                    RCLCPP_INFO(this->get_logger(), "OFFBOARD mode sent successfully");
                else
                    RCLCPP_WARN(this->get_logger(), "OFFBOARD mode request failed");
            });
    }

    void publish_arm()
    {
        auto client = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        if (!client->wait_for_service(5s)) { RCLCPP_ERROR(this->get_logger(), "Arming unavailable"); return; }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        // ✅ Changed to asynchronous callback
        auto result_future = client->async_send_request(request,
            [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
            {
                auto response = future.get();
                if (response->success)
                    RCLCPP_INFO(this->get_logger(), "Vehicle ARMED successfully");
                else
                    RCLCPP_WARN(this->get_logger(), "ARM failed");
            });
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_, y_, z_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LivePositionPublisher>();


    std::thread input_thread([&node]() { node->run_input_loop(); });

    rclcpp::spin(node);
    input_thread.join();
    rclcpp::shutdown();
    return 0;
}
