#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "surface_tracking_controller/siso_controller_base.hpp"
#include "surface_tracking_controller/mimo_controller_base.hpp"
#include "surface_tracking_controller/pid_controller.hpp"
#include "surface_tracking_controller/pid_ff_controller.hpp"
// #include "surface_tracking_controller/mpc_controller.hpp"

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
        this->declare_parameter<double>("max_cmd_vel", 0.5);
        this->declare_parameter<double>("max_integral", 1.0);

        // Fetch Parameter into class members
        controller_type_ = this->get_parameter("controller_type").as_string();
        double control_loop_rate = this->get_parameter("control_loop_rate").as_double();
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();
        k_ff_ = this->get_parameter("k_ff").as_double();
        max_cmd_vel_ = this->get_parameter("max_cmd_vel").as_double();
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
    double kp_, ki_, kd_, k_ff_, max_cmd_vel_, max_integral_;

    // Vector of 6 controllers for cartesian space
    std::vector<std::unique_ptr<surface_tracking_controller::SISOControllerBase>> siso_controllers_;
    std::unique_ptr<surface_tracking_controller::MIMOControllerBase> mimo_controller_;

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
        else if (controller_type_ == "mpc") {
            // mimo_controller_ = std::make_unique<MpcController>();
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
            } else if (param.get_name() == "max_cmd_vel") {
                max_cmd_vel_ = param.as_double();
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

            tf2::Quaternion current_q;
            tf2::fromMsg(current_transform.transform.rotation, current_q);
            double cr, cp, cy;
            tf2::Matrix3x3(current_q).getRPY(cr, cp, cy);

            // Extract cartesian setpoint (target)
            double setpoint_x = target_in_base.pose.position.x;
            double setpoint_y = target_in_base.pose.position.y;
            double setpoint_z = target_in_base.pose.position.z;

            tf2::Quaternion target_q;
            tf2::fromMsg(target_in_base.pose.orientation, target_q);
            double tr, tp, ty;
            tf2::Matrix3x3(target_q).getRPY(tr, tp, ty);

            // Calculate the shortest angular distance (error)
            double err_r = std::atan2(std::sin(tr - cr), std::cos(tr - cr));
            double err_p = std::atan2(std::sin(tp - cp), std::cos(tp - cp));
            double err_y = std::atan2(std::sin(ty - cy), std::cos(ty - cy));

            // Create a modified target which is the shortest path away from current
            double tr_adj = cr + err_r;
            double tp_adj = cp + err_p;
            double ty_adj = cy + err_y;

            // Extract feedforward velocity (if available, else 0)
            double ff_vx = has_target_twist_ ? latest_target_twist_.twist.linear.x : 0.0;
            double ff_vy = has_target_twist_ ? latest_target_twist_.twist.linear.y : 0.0;
            double ff_vz = has_target_twist_ ? latest_target_twist_.twist.linear.z : 0.0;
            double ff_wx = has_target_twist_ ? latest_target_twist_.twist.angular.x : 0.0;
            double ff_wy = has_target_twist_ ? latest_target_twist_.twist.angular.y : 0.0;
            double ff_wz = has_target_twist_ ? latest_target_twist_.twist.angular.z : 0.0;
            
            // Calculate command twist
            geometry_msgs::msg::TwistStamped cmd_twist;
            cmd_twist.header.stamp = current_time;
            cmd_twist.header.frame_id = base_frame; 

            cmd_twist.twist.linear.x = std::clamp(siso_controllers_[0]->update_with_ff(current_x, setpoint_x, ff_vx, dt), -max_cmd_vel_, max_cmd_vel_);
            cmd_twist.twist.linear.y = std::clamp(siso_controllers_[1]->update_with_ff(current_y, setpoint_y, ff_vy, dt), -max_cmd_vel_, max_cmd_vel_);
            cmd_twist.twist.linear.z = std::clamp(siso_controllers_[2]->update_with_ff(current_z, setpoint_z, ff_vz, dt), -max_cmd_vel_, max_cmd_vel_);
            
            cmd_twist.twist.angular.x = std::clamp(siso_controllers_[3]->update_with_ff(cr, tr_adj, ff_wx, dt), -max_cmd_vel_, max_cmd_vel_);
            cmd_twist.twist.angular.y = std::clamp(siso_controllers_[4]->update_with_ff(cp, tp_adj, ff_wy, dt), -max_cmd_vel_, max_cmd_vel_);
            cmd_twist.twist.angular.z = std::clamp(siso_controllers_[5]->update_with_ff(cy, ty_adj, ff_wz, dt), -max_cmd_vel_, max_cmd_vel_);

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