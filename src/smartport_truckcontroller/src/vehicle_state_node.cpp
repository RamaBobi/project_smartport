// ============================================================================
//  vehicle_state_node.cpp
//
//  Aggregates all available sensor data into a single VehicleState message.
//
//  State vector (27 scalars, 6-wheel truck):
//    x  y  z                     – position         (world frame) [m]
//    u  v  w                     – linear velocity   (body frame)  [m/s]
//    phi  theta  psi             – Euler angles      (RPY / ZYX)   [rad]
//    p   q   r                   – angular rates     (body frame)  [rad/s]
//    omega[6] = [fl fr ml mr rl rr]  – wheel spin speeds          [rad/s]
//    delta                       – commanded centre delta          [rad]
//    delta_fl  delta_fr          – actual steer joint angles       [rad]
//    u_throttle                  – drive command                   [rad/s]
//    p_brake                     – brake pressure  [0..1]
//    z_suspension[6]             – suspension deflections [fl fr ml mr rl rr] [m]
//
//  Subscriptions:
//    /model/heavy_tractor/odometry  nav_msgs/Odometry     – pose + body velocity
//    /imu                           sensor_msgs/Imu        – angular rates (p,q,r)
//    /joint_states                  sensor_msgs/JointState – omega, delta_fl/fr, z_susp
//    /truck/cmd_delta               std_msgs/Float64       – commanded centre delta
//    /truck/drive                   std_msgs/Float64       – drive velocity command
//
//  Publication:
//    /truck/vehicle_state           smartport_interfaces/VehicleState
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <string>
#include <unordered_map>
#include <array>

#include "smartport_interfaces/msg/vehicle_state.hpp"

// ─── Joint names expected from tractor.urdf ─────────────────────────────────
//  Wheel spin joints (velocity used)
static constexpr const char * OMEGA_JOINTS[6] = {
    "fl_hub_to_fl_wheel",
    "fr_hub_to_fr_wheel",
    "ml_hub_to_ml_wheel",
    "mr_hub_to_mr_wheel",
    "rl_hub_to_rl_wheel",
    "rr_hub_to_rr_wheel",
};
//  Steering joints (position used).
//  copy_truck.urdf names them fl_steering_joint / fr_steering_joint;
//  tractor.urdf still uses fl_steer / fr_steer.  We try the new names first
//  and fall back at call-site if they aren't present.
static constexpr const char * STEER_JOINTS[2]      = {"fl_steering_joint", "fr_steering_joint"};
static constexpr const char * STEER_JOINTS_LEGACY[2] = {"fl_steer", "fr_steer"};
//  Suspension joints (position used, deflection from spring reference=0)
static constexpr const char * SUSP_JOINTS[6] = {
    "fl_spring",
    "fr_spring",
    "left_rail_to_ml_spring",
    "right_rail_to_mr_spring",
    "left_rail_to_rl_spring",
    "right_rail_to_rr_spring",
};

// ────────────────────────────────────────────────────────────────────────────
class VehicleStateNode : public rclcpp::Node
{
public:
    VehicleStateNode() : Node("vehicle_state_node")
    {
        using std::placeholders::_1;

        // ── Publisher ─────────────────────────────────────────────────────────
        pub_state_ = create_publisher<smartport_interfaces::msg::VehicleState>(
            "/truck/vehicle_state", 10);

        // ── Subscriptions ─────────────────────────────────────────────────────

        // Odometry: pose (x,y,z,roll,pitch,yaw) + body-frame linear velocity (u,v,w)
        // Gazebo OdometryPublisher plugin publishes twist in the child/body frame.
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/heavy_tractor/odometry", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });

        // IMU: angular rates p,q,r in body frame
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            [this](sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });

        // Joint states: omega, steer angles, suspension deflections
        sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });

        // Commanded centre delta from user / planner
        sub_cmd_delta_ = create_subscription<std_msgs::msg::Float64>(
            "/truck/cmd_delta", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                state_.delta = msg->data;
            });

        // Drive velocity command (rad/s)
        sub_drive_ = create_subscription<std_msgs::msg::Float64>(
            "/truck/drive", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                state_.u_throttle = msg->data;
            });

        RCLCPP_INFO(get_logger(), "[VehicleState] Streaming /truck/vehicle_state (27 states, 6-wheel truck).");
    }

private:
    // ── Odometry callback ────────────────────────────────────────────────────
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Position
        state_.header = msg->header;
        state_.x = msg->pose.pose.position.x;
        state_.y = msg->pose.pose.position.y;
        state_.z = msg->pose.pose.position.z;

        // Euler angles from quaternion (RPY / ZYX convention)
        const auto & q = msg->pose.pose.orientation;
        quaternionToEuler(q.w, q.x, q.y, q.z,
                          state_.phi, state_.theta, state_.psi);

        // Body-frame linear velocity
        // Gazebo OdometryPublisher gives twist in child_frame (body frame).
        state_.u = msg->twist.twist.linear.x;
        state_.v = msg->twist.twist.linear.y;
        state_.w = msg->twist.twist.linear.z;

        publishState();
    }

    // ── IMU callback ─────────────────────────────────────────────────────────
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Angular rates in body frame
        state_.p = msg->angular_velocity.x;
        state_.q = msg->angular_velocity.y;
        state_.r = msg->angular_velocity.z;
    }

    // ── Joint state callback ─────────────────────────────────────────────────
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Rebuild index map if joint list changes
        if (msg->name.size() != last_js_size_) {
            js_index_.clear();
            for (size_t i = 0; i < msg->name.size(); ++i) {
                js_index_[msg->name[i]] = i;
            }
            last_js_size_ = msg->name.size();
        }

        // ── Wheel spin velocities [fl fr ml mr rl rr] ───────────────────────
        for (size_t k = 0; k < 6; ++k) {
            state_.omega[k] = extractVel(OMEGA_JOINTS[k], msg);
        }

        // ── Steering angles [fl, fr] ─────────────────────────────────────────
        // Try the copy_truck naming first; fall back to tractor.urdf names.
        state_.delta_fl = extractPos(STEER_JOINTS[0], msg);
        state_.delta_fr = extractPos(STEER_JOINTS[1], msg);
        if (state_.delta_fl == 0.0 && state_.delta_fr == 0.0) {
            state_.delta_fl = extractPos(STEER_JOINTS_LEGACY[0], msg);
            state_.delta_fr = extractPos(STEER_JOINTS_LEGACY[1], msg);
        }

        // ── Suspension deflections [fl fr ml mr rl rr] ──────────────────────
        for (size_t k = 0; k < 6; ++k) {
            state_.z_suspension[k] = extractPos(SUSP_JOINTS[k], msg);
        }
    }

    // ── Publish aggregated state ─────────────────────────────────────────────
    void publishState()
    {
        pub_state_->publish(state_);
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    /// Quaternion (w,x,y,z) → roll(φ), pitch(θ), yaw(ψ)  [ZYX convention]
    static void quaternionToEuler(
        double w, double x, double y, double z,
        double & phi, double & theta, double & psi)
    {
        // Roll (φ): rotation about X
        phi   = std::atan2(2.0 * (w * x + y * z),
                           1.0 - 2.0 * (x * x + y * y));
        // Pitch (θ): rotation about Y  — clamp to avoid singularity
        const double sinp = 2.0 * (w * y - z * x);
        theta = std::abs(sinp) >= 1.0
                ? std::copysign(M_PI / 2.0, sinp)
                : std::asin(sinp);
        // Yaw (ψ): rotation about Z
        psi   = std::atan2(2.0 * (w * z + x * y),
                           1.0 - 2.0 * (y * y + z * z));
    }

    double extractPos(const std::string & name,
                      const sensor_msgs::msg::JointState::SharedPtr & msg)
    {
        auto it = js_index_.find(name);
        if (it == js_index_.end()) return 0.0;
        const size_t idx = it->second;
        return (idx < msg->position.size()) ? msg->position[idx] : 0.0;
    }

    double extractVel(const std::string & name,
                      const sensor_msgs::msg::JointState::SharedPtr & msg)
    {
        auto it = js_index_.find(name);
        if (it == js_index_.end()) return 0.0;
        const size_t idx = it->second;
        return (idx < msg->velocity.size()) ? msg->velocity[idx] : 0.0;
    }

    // ── Members ──────────────────────────────────────────────────────────────
    smartport_interfaces::msg::VehicleState state_;

    rclcpp::Publisher<smartport_interfaces::msg::VehicleState>::SharedPtr pub_state_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  sub_js_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr        sub_cmd_delta_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr        sub_drive_;

    std::unordered_map<std::string, size_t> js_index_;
    size_t last_js_size_{0};
};

// ────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleStateNode>());
    rclcpp::shutdown();
    return 0;
}
