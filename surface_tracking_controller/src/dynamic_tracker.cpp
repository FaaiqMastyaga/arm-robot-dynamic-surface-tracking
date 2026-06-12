#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "surface_tracking_controller/siso_controller_base.hpp"
#include "surface_tracking_controller/pid_controller.hpp"
#include "surface_tracking_controller/pid_ff_controller.hpp"

#include <chrono>
#include <vector>
#include <algorithm>
#include <memory>

using namespace std::chrono_literals;

namespace surface_tracking_controller {

class DynamicTracker : public rclcpp::Node {
public:
    DynamicTracker(const rclcpp::NodeOptions& options) : Node("dynamic_tracking", options) 
    {
        // Declare Parameter
        this->declare_parameter<std::string>("controller_type", "pid_ff");
        this->declare_parameter<double>("control_loop_rate", 100.0);
        this->declare_parameter<double>("kp", 1.0);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.0);
        this->declare_parameter<double>("k_ff", 0.0);
        this->declare_parameter<double>("max_linear_vel", 0.4);
        this->declare_parameter<double>("max_angular_vel", 0.8);
        this->declare_parameter<double>("max_integral", 1.0);

        // Fetch Parameter into class members
        controller_type_ = this->get_parameter("controller_type").as_string();
        double control_loop_rate = this->get_parameter("control_loop_rate").as_double();
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();
        k_ff_ = this->get_parameter("k_ff").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        max_integral_ = this->get_parameter("max_integral").as_double();

        double control_loop_period = 1000.0 / control_loop_rate;

        // Initialize controllers
        init_controllers();

        // Setup Parameter Callback for dynamic updates
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DynamicTracker::parameters_callback, this, std::placeholders::_1)
        );

        // Setup TF Buffer and Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Setup Publisher
        servo_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/tracking/delta_twist_cmds", 10);

        // Setup Subscriber
        planner_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/desired_drawing_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped& msg) {
                latest_target_pose_ = msg;
                has_target_pose_ = true;
            }
        );
        target_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/target_twist", 10, 
            [this](const geometry_msgs::msg::TwistStamped& msg) {
                latest_target_twist_ = msg;
                has_target_twist_ = true;
            }
        );

        last_time_ = this->get_clock()->now();

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(control_loop_period)),
            std::bind(&DynamicTracker::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Dynamic Tracking node loaded. Controller: %s", controller_type_.c_str());    
    }
    
private:
    // --- State Caches ---
    geometry_msgs::msg::PoseStamped latest_target_pose_;
    geometry_msgs::msg::TwistStamped latest_target_twist_;
    bool has_target_pose_ = false;
    bool has_target_twist_ = false;

    // --- Control Parameters ---
    std::string controller_type_;
    double kp_, ki_, kd_, k_ff_, max_linear_vel_, max_angular_vel_, max_integral_;

    // Vector of 6 controllers for cartesian space
    std::vector<std::unique_ptr<surface_tracking_controller::SISOControllerBase>> siso_controllers_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr planner_pose_sub_; 
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_sub_; 
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Time last_time_;

    void init_controllers() 
    {
        siso_controllers_.clear();
        
        if (controller_type_ == "pid") {
            for (int i = 0; i < 6; ++i) {
                siso_controllers_.push_back(std::make_unique<PidController>(kp_, ki_, kd_, max_integral_));
            }
        }
        else if (controller_type_ == "pid_ff") {
            for (int i = 0; i < 6; ++i) {
                siso_controllers_.push_back(std::make_unique<PidFeedforwardController>(kp_, ki_, kd_, k_ff_, max_integral_));
            }
        }
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters) 
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        bool reinit_controllers = false;

        for (const auto &param : parameters) {
            if (param.get_name() == "kp") {
                kp_ = param.as_double();
                reinit_controllers = true;
            } else if (param.get_name() == "ki") {
                ki_ = param.as_double();
                reinit_controllers = true;
            } else if (param.get_name() == "kd") {
                kd_ = param.as_double();
                reinit_controllers = true;
            } else if (param.get_name() == "k_ff") {
                k_ff_ = param.as_double();
                reinit_controllers = true;
            } else if (param.get_name() == "max_integral") {
                max_integral_ = param.as_double();
                reinit_controllers = true;
            } else if (param.get_name() == "max_linear_vel") {
                max_linear_vel_ = param.as_double();
            } else if (param.get_name() == "max_angular_vel") {
                max_angular_vel_ = param.as_double();
            } else if (param.get_name() == "controller_type") {
                controller_type_ = param.as_string();
                reinit_controllers = true;
            }
        }

        // Recreate the controllers if gain or type parameters changed
        if (reinit_controllers) {
            init_controllers();
            RCLCPP_INFO(this->get_logger(), "Updated controller parameters. (Type: %s, Kp: %.2f, Ki: %.2f, Kd: %.2f, K_ff: %.2f)", 
                        controller_type_.c_str(), kp_, ki_, kd_, k_ff_);
        }

        return result;
    }

    void control_loop()
    {
        // Don't act unless we have an active drawing target
        if (!has_target_pose_) return;

        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0) return;

        // Base frame for the robot (Fixed reference)
        const std::string base_frame = "elfin_base_link";
        // End effector frame (the moving part)
        const std::string ee_frame = "pen_tip_link";

        try {
            // Transform Target Pose to Base Frame (P_target)
            // Convert the desired pose into the robot's root coordinate system
            geometry_msgs::msg::TransformStamped target_transform = tf_buffer_->lookupTransform(
                base_frame, latest_target_pose_.header.frame_id, tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped target_in_base;
            tf2::doTransform(latest_target_pose_, target_in_base, target_transform);

            // Lookup current end effector pose directly from TF (P_current)
            geometry_msgs::msg::TransformStamped current_transform = tf_buffer_->lookupTransform(
                base_frame, ee_frame, tf2::TimePointZero);

            // Extract cartesian state (current)
            double current_x = current_transform.transform.translation.x;
            double current_y = current_transform.transform.translation.y;
            double current_z = current_transform.transform.translation.z;

            tf2::Quaternion q_current;
            tf2::fromMsg(current_transform.transform.rotation, q_current);

            // Extract cartesian setpoint (target)
            double setpoint_x = target_in_base.pose.position.x;
            double setpoint_y = target_in_base.pose.position.y;
            double setpoint_z = target_in_base.pose.position.z;

            tf2::Quaternion q_target;
            tf2::fromMsg(target_in_base.pose.orientation, q_target);

            // Calculate quaternion error
            tf2::Quaternion q_error = q_target * q_current.inverse();

            // Extract axis-angle from quaternion error
            tf2::Vector3 axis = q_error.getAxis();
            double angle = q_error.getAngle();

            // Normalize angle to [-pi, pi]
            if (angle > M_PI) {
                angle -= 2.0 * M_PI;
            } else if (angle < -M_PI) {
                angle += 2.0 * M_PI;
            }

            // Calculate vector of orientation error (scaled by angle)
            double err_rx = axis.x() * angle; // Roll error
            double err_ry = axis.y() * angle; // Pitch error
            double err_rz = axis.z() * angle; // Yaw error

            // Extract feedforward velocity (if available, else 0)
            double ff_vx = has_target_twist_ ? latest_target_twist_.twist.linear.x : 0.0;
            double ff_vy = has_target_twist_ ? latest_target_twist_.twist.linear.y : 0.0;
            double ff_vz = has_target_twist_ ? latest_target_twist_.twist.linear.z : 0.0;
            double ff_wx = has_target_twist_ ? latest_target_twist_.twist.angular.x : 0.0;
            double ff_wy = has_target_twist_ ? latest_target_twist_.twist.angular.y : 0.0;
            double ff_wz = has_target_twist_ ? latest_target_twist_.twist.angular.z : 0.0;
            
            // Calculate raw output from controllers (before clamping)
            double vx = siso_controllers_[0]->update_with_ff(current_x, setpoint_x, ff_vx, dt);
            double vy = siso_controllers_[1]->update_with_ff(current_y, setpoint_y, ff_vy, dt);
            double vz = siso_controllers_[2]->update_with_ff(current_z, setpoint_z, ff_vz, dt);

            double wx = siso_controllers_[3]->update_with_ff(0.0, err_rx, ff_wx, dt);
            double wy = siso_controllers_[4]->update_with_ff(0.0, err_ry, ff_wy, dt);
            double wz = siso_controllers_[5]->update_with_ff(0.0, err_rz, ff_wz, dt);
            
            vx = std::clamp(vx, -max_linear_vel_, max_linear_vel_);
            vy = std::clamp(vy, -max_linear_vel_, max_linear_vel_);
            vz = std::clamp(vz, -max_linear_vel_, max_linear_vel_);

            wx = std::clamp(wx, -max_angular_vel_, max_angular_vel_);
            wy = std::clamp(wy, -max_angular_vel_, max_angular_vel_);
            wz = std::clamp(wz, -max_angular_vel_, max_angular_vel_);

            // Publish the command twist
            geometry_msgs::msg::TwistStamped cmd_twist;
            cmd_twist.header.stamp = current_time;
            cmd_twist.header.frame_id = base_frame; 

            cmd_twist.twist.linear.x = vx;
            cmd_twist.twist.linear.y = vy;
            cmd_twist.twist.linear.z = vz;
            
            cmd_twist.twist.angular.x = wx;
            cmd_twist.twist.angular.y = wy;
            cmd_twist.twist.angular.z = wz;

            servo_twist_pub_->publish(cmd_twist);

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Tracking occluded! Waiting for Aimooe TF: %s", ex.what());

            for (auto& ctrl : siso_controllers_) {
                ctrl->reset();
            }

            geometry_msgs::msg::TwistStamped stop_twist;
            stop_twist.header.stamp = current_time;
            stop_twist.header.frame_id = base_frame;
            servo_twist_pub_->publish(stop_twist);
        }
    }
};

}  // namespace surface_tracking_controller

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    auto node = std::make_shared<surface_tracking_controller::DynamicTracker>(options);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}