#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "smartport_interfaces/msg/vehicle_command.hpp"
#include "smartport_interfaces/msg/actuator_state.hpp"

class AckermannBridge : public rclcpp::Node {
public:
    AckermannBridge() : Node("ackermann_bridge") {
        // Declare and fetch parameters (Default to heavy tractor parameters)
        this->declare_parameter("vehicle.kinematics.wheelbase", 3.5);
        this->declare_parameter("vehicle.kinematics.track_width", 2.4);
        this->declare_parameter("vehicle.dynamics.mass", 35000.0);
        this->declare_parameter("vehicle.actuators.throttle_time_constant", 1.2);

        L_ = this->get_parameter("vehicle.kinematics.wheelbase").as_double();
        w_ = this->get_parameter("vehicle.kinematics.track_width").as_double();
        mass_ = this->get_parameter("vehicle.dynamics.mass").as_double();
        tau_v_ = this->get_parameter("vehicle.actuators.throttle_time_constant").as_double();

        RCLCPP_INFO(this->get_logger(), "Initialized lower-level bridge for vehicle mass: %.1f kg", mass_);

        // Subscribers & Publishers
        cmd_sub_ = this->create_subscription<smartport_interfaces::msg::VehicleCommand>(
            "/vehicle/cmd_bicycle", 10, std::bind(&AckermannBridge::command_callback, this, std::placeholders::_1));
        
        state_pub_ = this->create_publisher<smartport_interfaces::msg::ActuatorState>(
            "/vehicle/actuator_state", 10);

        // Core execution loop at 50Hz (20ms interval) to simulate vehicle physics
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&AckermannBridge::physics_loop, this));
        
        last_time_ = this->now();
    }

private:
    void command_callback(const smartport_interfaces::msg::VehicleCommand::SharedPtr msg) {
        target_velocity_ = msg->velocity;
        target_steering_angle_ = msg->steering_angle;
    }

    void physics_loop() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;
        last_time_ = current_time;

        // 1. Classical Ackermann Geometry Transformation
        double delta = target_steering_angle_;
        double delta_inner = 0.0;
        double delta_outer = 0.0;

        if (std::abs(delta) > 0.001) {
            delta_inner = std::atan((L_ * std::tan(delta)) / (L_ - (w_ / 2.0) * std::tan(delta)));
            delta_outer = std::atan((L_ * std::tan(delta)) / (L_ + (w_ / 2.0) * std::tan(delta)));
        }

        // 2. Simulate Powertrain Lag using a First-Order Low-Pass Filter
        // d(v)/dt = (v_target - v_current) / tau
        current_velocity_ += (dt / (tau_v_ + dt)) * (target_velocity_ - current_velocity_);

        // 3. Simple Fuel Consumption Model (Proportional to mass and acceleration drag)
        double acceleration = (target_velocity_ - current_velocity_) / tau_v_;
        double power_required = std::max(0.0, mass_ * acceleration * current_velocity_);
        double fuel_rate = (power_required * 0.00002) + 0.0005; // Base idle + load factor
        cumulative_fuel_ += fuel_rate * dt;

        // 4. Publish Telemetry
        auto state_msg = smartport_interfaces::msg::ActuatorState();
        state_msg.header.stamp = current_time;
        state_msg.actual_velocity = current_velocity_;
        state_msg.actual_steering_angle = delta; // Virtual collapsed center
        state_msg.fuel_consumption_rate = fuel_rate;
        state_msg.cumulative_fuel_used = cumulative_fuel_;

        state_pub_->publish(state_msg);
    }

    // Parameters
    double L_, w_, mass_, tau_v_;

    // Target inputs from Upper Layer
    double target_velocity_ = 0.0;
    double target_steering_angle_ = 0.0;

    // Simulated states
    double current_velocity_ = 0.0;
    double cumulative_fuel_ = 0.0;

    rclcpp::Subscription<smartport_interfaces::msg::VehicleCommand>::SharedPtr cmd_sub_;
    rclcpp::Publisher<smartport_interfaces::msg::ActuatorState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannBridge>());
    rclcpp::shutdown();
    return 0;
}
