#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>

class RtgControllerNode : public rclcpp::Node {
public:
    RtgControllerNode() : Node("rtg_controller_node"), attached_(false) {
        // Publishers for the individual joint commands
        pub_x_carriage_ = this->create_publisher<std_msgs::msg::Float64>("/rtg/x_carriage/cmd_pos", 10);
        pub_trolley_ = this->create_publisher<std_msgs::msg::Float64>("/rtg/trolley/cmd_pos", 10);
        pub_hoist_mid_ = this->create_publisher<std_msgs::msg::Float64>("/rtg/hoist_mid/cmd_pos", 10);
        pub_spreader_ = this->create_publisher<std_msgs::msg::Float64>("/rtg/spreader/cmd_pos", 10);
        
        // Publisher for the auto-grabber DetachableJoint trigger
        pub_attach_ = this->create_publisher<std_msgs::msg::Empty>("/rtg/attach", 10);

        // Subscriber for the user's XYZ position target
        sub_cmd_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/rtg/cmd_pos", 10,
            std::bind(&RtgControllerNode::cmdPosCallback, this, std::placeholders::_1)
        );

        // TF2 Setup for proximity detection
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "RTG Auto-Grabber Controller Initialized.");
    }

private:
    void cmdPosCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        double x_target = msg->x;
        double y_target = msg->y;
        double z_target = msg->z;

        // Carriage X (limits: -4.0 to 4.0)
        auto x_msg = std_msgs::msg::Float64();
        if(x_target > 4.0) x_target = 4.0;
        if(x_target < -4.0) x_target = -4.0;
        x_msg.data = x_target;
        pub_x_carriage_->publish(x_msg);

        // Trolley Y (limits: -4.5 to 4.5)
        auto y_msg = std_msgs::msg::Float64();
        if(y_target > 4.5) y_target = 4.5;
        if(y_target < -4.5) y_target = -4.5;
        y_msg.data = y_target;
        pub_trolley_->publish(y_msg);

        // Clamp Z Target
        if (z_target > 0.0) z_target = 0.0;
        if (z_target < -9.6) z_target = -9.6;

        // ---- AUTO GRABBER PROXIMITY CHECK ----
        if (!attached_) {
            try {
                // Try to get the TF from the spreader link to the container link
                // The physical spreader visual is offset by -4.8m from the spreader link!
                geometry_msgs::msg::TransformStamped t;
                // PosePublisher broadcasts the frame name as 'container_brown'
                t = tf_buffer_->lookupTransform("spreader", "container_brown", tf2::TimePointZero);

                double dx = t.transform.translation.x;
                double dy = t.transform.translation.y;
                // Container is at Z=1.0. Spreader visual is at -4.8. 
                // The actual distance between the physical spreader box and the container top
                double dz = t.transform.translation.z + 4.8 - 1.0; 

                // Calculate horizontal error (must be aligned!)
                double h_dist = std::sqrt(dx*dx + dy*dy);
                
                if (h_dist < 0.5 && std::abs(dz) <= 0.15) {
                    RCLCPP_INFO(this->get_logger(), "PROXIMITY ALERT: Container detected within 0.1m!");
                    RCLCPP_INFO(this->get_logger(), "AUTO-GRABBER ACTIVATED. Freezing Z and locking joints...");
                    
                    // Freeze the spreader right where it is (override the user's command)
                    attached_ = true;
                    frozen_z_ = z_target; // Lock the current Z height
                    
                    // Fire the detachment/attachment joint trigger
                    pub_attach_->publish(std_msgs::msg::Empty());
                }
            } catch (const tf2::TransformException & ex) {
                // Ignore TF errors if the container hasn't broadcasted yet
            }
        }

        // If we are attached, override the user's downward command so we don't crush the container!
        // We still allow them to lift it UP (moving Z closer to 0).
        if (attached_) {
            if (z_target < frozen_z_) {
                z_target = frozen_z_;
            }
        }

        // Hoist Z (Spreader target). Split across the two telescoping joints.
        auto mid_msg = std_msgs::msg::Float64();
        auto spreader_msg = std_msgs::msg::Float64();
        mid_msg.data = z_target / 2.0;
        spreader_msg.data = z_target / 2.0;

        pub_hoist_mid_->publish(mid_msg);
        pub_spreader_->publish(spreader_msg);
    }

    bool attached_;
    double frozen_z_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_x_carriage_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_trolley_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hoist_mid_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_spreader_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_attach_;
    
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_cmd_pos_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RtgControllerNode>());
    rclcpp::shutdown();
    return 0;
}
