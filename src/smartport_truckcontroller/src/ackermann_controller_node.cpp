// ============================================================================
//  ackermann_controller_node.cpp
//
//  Custom Ackermann steering controller running at 1000 Hz.
//
//  Subscriptions:
//    /truck/cmd_delta      [std_msgs/Float64]  – commanded centre delta from user/planner
//    /truck/vehicle_delta  [std_msgs/Float64]  – current estimated centre delta (from wheel_state_node)
//
//  Publications:
//    /truck/fl_steer/cmd_pos  [std_msgs/Float64]  – front-LEFT  wheel position command
//    /truck/fr_steer/cmd_pos  [std_msgs/Float64]  – front-RIGHT wheel position command
//
//  Ackermann geometry (positive delta = turn LEFT):
//    R       = L / tan(|δ_cmd|)
//    δ_inner = atan( L / (R − W/2) )   → assigned to inside wheel
//    δ_outer = atan( L / (R + W/2) )   → assigned to outside wheel
//    Turn left : left=inner, right=outer
//    Turn right: right=inner, left=outer
//
//  Rear-wheel drive: only steering, no drive command here.
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <cmath>

#include "smartport_truckcontroller/ackermann.hpp"

using namespace std::chrono_literals;

// ─── Constants (must match tractor.urdf) ────────────────────────────────────
static constexpr double DEFAULT_WHEELBASE   = 3.0;   // L [m]
static constexpr double DEFAULT_TRACK_WIDTH = 2.4;   // W [m]
static constexpr double DEFAULT_MAX_STEER   = 0.5236; // 30° [rad]
static constexpr double LOOP_HZ             = 1000.0;

// ────────────────────────────────────────────────────────────────────────────
class AckermannControllerNode : public rclcpp::Node
{
public:
    AckermannControllerNode()
    : Node("ackermann_controller_node"),
      cmd_delta_(0.0),
      vehicle_delta_(0.0)
    {
        // ── Parameters ───────────────────────────────────────────────────────
        declare_parameter("wheelbase",       DEFAULT_WHEELBASE);
        declare_parameter("track_width",     DEFAULT_TRACK_WIDTH);
        declare_parameter("max_steer_rad",   DEFAULT_MAX_STEER);
        declare_parameter("loop_rate_hz",    LOOP_HZ);

        geom_.wheelbase   = get_parameter("wheelbase").as_double();
        geom_.track_width = get_parameter("track_width").as_double();
        max_steer_        = get_parameter("max_steer_rad").as_double();
        const double hz   = get_parameter("loop_rate_hz").as_double();

        // ── Publishers ───────────────────────────────────────────────────────
        pub_fl_ = create_publisher<std_msgs::msg::Float64>("/truck/fl_steer/cmd_pos", 10);
        pub_fr_ = create_publisher<std_msgs::msg::Float64>("/truck/fr_steer/cmd_pos", 10);

        // ── Subscriptions ────────────────────────────────────────────────────
        // User / planner commanded centre delta
        sub_cmd_ = create_subscription<std_msgs::msg::Float64>(
            "/truck/cmd_delta", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                cmd_delta_ = std::clamp(msg->data, -max_steer_, max_steer_);
            });

        // Current actual vehicle centre delta (from wheel_state_node)
        // Stored for monitoring / future feedback; control uses cmd_delta_.
        sub_veh_ = create_subscription<std_msgs::msg::Float64>(
            "/truck/vehicle_delta", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                vehicle_delta_ = msg->data;
            });

        // ── 1000 Hz control loop ─────────────────────────────────────────────
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / hz));
        timer_ = create_wall_timer(period,
            std::bind(&AckermannControllerNode::controlLoop, this));

        RCLCPP_INFO(get_logger(),
            "[AckermannCtrl] Ready | L=%.2f m  W=%.2f m  max_steer=%.1f°  loop=%.0f Hz",
            geom_.wheelbase, geom_.track_width, max_steer_ * 180.0 / M_PI, hz);
    }

private:
    // ────────────────────────────────────────────────────────────────────────
    void controlLoop()
    {
        // Compute Ackermann geometry from commanded centre delta
        const auto ak = smartport::compute_ackermann(cmd_delta_, geom_);

        // Publish individual wheel position commands
        std_msgs::msg::Float64 fl_msg, fr_msg;
        fl_msg.data = ak.delta_fl;
        fr_msg.data = ak.delta_fr;

        pub_fl_->publish(fl_msg);
        pub_fr_->publish(fr_msg);
    }

    // ── Members ──────────────────────────────────────────────────────────────
    smartport::AckermannGeometry geom_;
    double cmd_delta_;      // latest commanded centre delta [rad]
    double vehicle_delta_;  // latest measured centre delta  [rad]
    double max_steer_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fr_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_veh_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    // Real-time spin: use a dedicated single-threaded executor
    auto node = std::make_shared<AckermannControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
