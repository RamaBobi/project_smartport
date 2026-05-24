// ============================================================================
//  wheel_state_node.cpp
//
//  Listens to /joint_states (from Gazebo JointStatePublisher bridge) and
//  extracts the front-wheel steering angles for the TRUCK only.
//
//  Publications:
//    /truck/fl_steer_angle  [std_msgs/Float64]  – actual fl_steer joint angle [rad]
//    /truck/fr_steer_angle  [std_msgs/Float64]  – actual fr_steer joint angle [rad]
//    /truck/vehicle_delta   [std_msgs/Float64]  – estimated centre delta (avg of fl+fr) [rad]
//
//  Joint names (from tractor.urdf):
//    fl_steer   – front-left  steering revolute joint
//    fr_steer   – front-right steering revolute joint
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <unordered_map>

// ────────────────────────────────────────────────────────────────────────────
class WheelStateNode : public rclcpp::Node
{
public:
    WheelStateNode() : Node("wheel_state_node")
    {
        // ── Publishers ───────────────────────────────────────────────────────
        pub_fl_    = create_publisher<std_msgs::msg::Float64>("/truck/fl_steer_angle",  10);
        pub_fr_    = create_publisher<std_msgs::msg::Float64>("/truck/fr_steer_angle",  10);
        pub_delta_ = create_publisher<std_msgs::msg::Float64>("/truck/vehicle_delta",   10);

        // ── Joint states subscription ────────────────────────────────────────
        // Gazebo JointStatePublisher → ros_gz_bridge → /joint_states
        sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&WheelStateNode::jointStateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "[WheelState] Listening to /joint_states for fl_steer & fr_steer.");
    }

private:
    // ────────────────────────────────────────────────────────────────────────
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Build name→index map on first call (or if layout changes)
        if (msg->name.size() != last_size_) {
            joint_index_.clear();
            for (size_t i = 0; i < msg->name.size(); ++i) {
                joint_index_[msg->name[i]] = i;
            }
            last_size_ = msg->name.size();
        }

        // Joint names per copy_truck.urdf: fl_steering_joint / fr_steering_joint
        // (older tractor.urdf used fl_steer / fr_steer; fall back if needed)
        double fl = extractPos("fl_steering_joint", msg);
        double fr = extractPos("fr_steering_joint", msg);
        if (fl == 0.0 && fr == 0.0) {
            fl = extractPos("fl_steer", msg);
            fr = extractPos("fr_steer", msg);
        }

        // Publish individual wheel angles
        std_msgs::msg::Float64 fl_msg, fr_msg, delta_msg;
        fl_msg.data    = fl;
        fr_msg.data    = fr;
        // Estimated centre delta = average of the two steer angles
        delta_msg.data = (fl + fr) / 2.0;

        pub_fl_->publish(fl_msg);
        pub_fr_->publish(fr_msg);
        pub_delta_->publish(delta_msg);
    }

    // ── Helper: look up position of a named joint; returns 0 if not found ───
    double extractPos(const std::string & name,
                      const sensor_msgs::msg::JointState::SharedPtr & msg)
    {
        auto it = joint_index_.find(name);
        if (it == joint_index_.end()) return 0.0;
        const size_t idx = it->second;
        if (idx >= msg->position.size()) return 0.0;
        return msg->position[idx];
    }

    // ── Members ──────────────────────────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fr_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_delta_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;

    std::unordered_map<std::string, size_t> joint_index_;
    size_t last_size_{0};
};

// ────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelStateNode>());
    rclcpp::shutdown();
    return 0;
}
