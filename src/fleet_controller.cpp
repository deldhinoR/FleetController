#include <chrono>
#include <iostream>
#include <string>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

using namespace std::chrono_literals;

class FleetController : public rclcpp::Node
{
public:
    FleetController() : Node("fleet_controller")
    {
        // Publishers for 3 drones
        pub1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/drone1/setpoint_position/local", 10);
        pub2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/drone2/setpoint_position/local", 10);
        pub3_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/drone3/setpoint_position/local", 10);

        leader_orientation_.w = 1.0;

        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "drone1/local_position/pose",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            leader_orientation_ = msg->pose.orientation;
        });


        timer_ = this->create_wall_timer(
        100ms, std::bind(&FleetController::publish_pose, this));

         x_ = 0.0;
         y_ = 0.0;
         z_ = 2.0;

         RCLCPP_INFO(this->get_logger(), "FleetController started. Type x:y:z or 'land', 'arm', 'ready'");
    }

    #include <algorithm>
    #include <cctype>

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

            double nx, ny, nz;
            try {
                if (parse_line(line, nx, ny, nz)) {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    x_ = nx; y_ = ny; z_ = nz;
                    RCLCPP_INFO(this->get_logger(), "New leader target: %.2f, %.2f, %.2f", x_, y_, z_);
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

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

    void publish_pose() {
    
    std::lock_guard<std::mutex> lock(pose_mutex_);

    geometry_msgs::msg::PoseStamped msg1; msg1.header.stamp = this->now(); 
    msg1.header.frame_id = "map";
    msg1.pose.position.x = x_; 
    msg1.pose.position.y = y_; 
    msg1.pose.position.z = z_; msg1.pose.orientation.w = 1.0; 
    pub1_->publish(msg1);

    // Follower drones in triangular formation 
    geometry_msgs::msg::PoseStamped msg2 = msg1; 
    msg2.pose.position.x += 5;
    msg2.pose.position.y -= sqrt(3) * 5; 
    pub2_->publish(msg2);

    geometry_msgs::msg::PoseStamped msg3 = msg1; 
    msg3.pose.position.x -= 5; // diagonal offset 
    msg3.pose.position.y -= sqrt(3) * 5; // sqrt(3) for equilateral triangle 
    pub3_->publish(msg3);
    }

    void publish_land()
    {
    for (const auto &drone_ns : {"drone1", "drone2", "drone3"}) {
        auto client = this->create_client<mavros_msgs::srv::SetMode>(std::string(drone_ns) + "/set_mode");
        if (!client->wait_for_service(5s)) { 
            RCLCPP_ERROR(this->get_logger(), "%s SetMode unavailable", drone_ns); 
            continue; 
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "AUTO.LAND";

        client->async_send_request(request,
            [this, drone_ns](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
            {
                auto response = future.get();
                if (response->mode_sent)
                    RCLCPP_INFO(this->get_logger(), "%s Landing mode sent successfully", drone_ns);
                else
                    RCLCPP_WARN(this->get_logger(), "%s Landing mode request failed", drone_ns);
            });
        std::this_thread::sleep_for(1s);    
    }
}

void publish_offBoard()
{
    for (const auto &drone_ns : {"drone1", "drone2", "drone3"}) {
        auto client = this->create_client<mavros_msgs::srv::SetMode>(std::string(drone_ns) + "/set_mode");    
        if (!client->wait_for_service(5s)) { 
            RCLCPP_ERROR(this->get_logger(), "%s SetMode unavailable", drone_ns); 
            continue; 
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "OFFBOARD";

        client->async_send_request(request,
            [this, drone_ns](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
            {
                auto response = future.get();
                if (response->mode_sent)
                    RCLCPP_INFO(this->get_logger(), "%s OFFBOARD mode sent successfully", drone_ns);
                else
                    RCLCPP_WARN(this->get_logger(), "%s OFFBOARD mode request failed", drone_ns);
            });
            std::this_thread::sleep_for(1s); 
    }
}

void publish_arm()
{
    for (const auto &drone_ns : {"drone1", "drone2", "drone3"}) {
        auto client = this->create_client<mavros_msgs::srv::CommandBool>(std::string(drone_ns) + "/cmd/arming");
        if (!client->wait_for_service(5s)) { 
            RCLCPP_ERROR(this->get_logger(), "%s Arming unavailable", drone_ns); 
            continue; 
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        client->async_send_request(request,
            [this, drone_ns](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
            {
                auto response = future.get();
                if (response->success)
                    RCLCPP_INFO(this->get_logger(), "%s Vehicle ARMED successfully", drone_ns);
                else
                    RCLCPP_WARN(this->get_logger(), "%s ARM failed", drone_ns);
            });
            std::this_thread::sleep_for(1s); 
    }
}

        
    geometry_msgs::msg::Quaternion leader_orientation_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub1_, pub2_, pub3_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_, y_, z_;
    std::mutex pose_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FleetController>();

    std::thread input_thread([&node]() { node->run_input_loop(); });

    rclcpp::spin(node);
    input_thread.join();
    rclcpp::shutdown();
    return 0;
}
