#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

// Truck geometry (matches tractor.urdf)
static constexpr double WHEELBASE   = 3.0;   // front axle X=1.5 to rear axle X=-1.5
static constexpr double TRACK_WIDTH = 2.4;   // left hub Y=+1.2 to right hub Y=-1.2

// Ackermann steering:
//   fl_steer and fr_steer are attached to chassis-fixed mounts, so their
//   joint angles ARE the absolute wheel yaw angles in the chassis frame.
//
//     fl_steer = atan(L / (R - T/2))   outer wheel for left turn
//     fr_steer = atan(L / (R + T/2))   inner wheel for left turn
//
//   where R = L / tan(delta) is the rear-axle turning radius.

static double ackermann_left(double delta)
{
    if (std::abs(delta) < 1e-6) return 0.0;
    const double R = WHEELBASE / std::tan(delta);
    return std::atan(WHEELBASE / (R - TRACK_WIDTH / 2.0));
}

static double ackermann_right(double delta)
{
    if (std::abs(delta) < 1e-6) return 0.0;
    const double R = WHEELBASE / std::tan(delta);
    return std::atan(WHEELBASE / (R + TRACK_WIDTH / 2.0));
}

class SteeringControllerNode : public rclcpp::Node
{
public:
    SteeringControllerNode() : Node("steering_controller_node")
    {
        pub_fl_ = create_publisher<std_msgs::msg::Float64>("/truck/fl_steer/cmd_pos", 10);
        pub_fr_ = create_publisher<std_msgs::msg::Float64>("/truck/fr_steer/cmd_pos", 10);

        // /truck/steer: single center steering angle in radians (positive = left)
        sub_steer_ = create_subscription<std_msgs::msg::Float64>(
            "/truck/steer", 10,
            std::bind(&SteeringControllerNode::steerCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Ackermann steering controller ready (L=%.1fm T=%.1fm).",
                    WHEELBASE, TRACK_WIDTH);
    }

private:
    void steerCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        const double delta   = std::clamp(msg->data, -0.5236, 0.5236);

        std_msgs::msg::Float64 fl_msg, fr_msg;
        fl_msg.data = ackermann_left(delta);
        fr_msg.data = ackermann_right(delta);

        pub_fl_->publish(fl_msg);
        pub_fr_->publish(fr_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fr_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_steer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteeringControllerNode>());
    rclcpp::shutdown();
    return 0;
}
