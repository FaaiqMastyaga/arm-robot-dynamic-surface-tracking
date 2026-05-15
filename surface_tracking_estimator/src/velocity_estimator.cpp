#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "surface_tracking_estimator/velocity_filter_base.hpp"
#include "surface_tracking_estimator/ema_filter.hpp"
// #include "surface_tracking_estimator/kalman_filter.hpp"

using namespace std::chrono_literals;

class VelocityEstimator : public rclcpp::Node 
{
public:
    VelocityEstimator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("velocity_estimator", options) 
    {
        // Declare parameters
        this->declare_parameter<std::string>("filter_type", "ema");
        this->declare_parameter<double>("ema_alpha", 0.2);
        this->declare_parameter<std::string>("base_frame", "elfin_base_link");
        this->declare_parameter<std::string>("target_frame", "whiteboard");
        this->declare_parameter<double>("update_rate", 100.0);

        // Fetch parameters
        base_frame_ = this->get_parameter("base_frame").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        double update_rate = this->get_parameter("update_rate").as_double();
        
        // Initialize Filter
        std::string filter_type = this->get_parameter("filter_type").as_string();
        if (filter_type == "ema") {
            double alpha = this->get_parameter("ema_alpha").as_double();
            active_filter_ = std::make_unique<surface_tracking_estimator::EMAFilter>(alpha);
            RCLCPP_INFO(this->get_logger(), "Initialized EMA Filter with alpha: %.2f", alpha);
        } else if (filter_type == "kalman") {
            // Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6, 6) * 0.01;
            // Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6, 6) * 0.1;
            // active_filter_ = std::make_unique<surface_tracking_estimator::KalmanFilter>(Q, R);
        }

        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize Publisher
        raw_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/surface_tracking/raw_velocity", 10);
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/surface_tracking/estimated_velocity", 10);

        // Setup Timer
        last_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
        timer_ = this->create_wall_timer(
            timer_period, std::bind(&VelocityEstimator::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Velocity Estimator running at %.1f Hz. Tracking %s relative to %s", 
            update_rate, target_frame_.c_str(), base_frame_.c_str());
    }

private:
    std::unique_ptr<surface_tracking_estimator::VelocityFilterBase> active_filter_;
    rclcpp::Time last_time_;

    std::vector<double> prev_pose_vector_;
    tf2::Quaternion prev_q_;
    bool first_q_init_ = false;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr raw_twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string base_frame_;
    std::string target_frame_;

    void timer_callback() {
        geometry_msgs::msg::TransformStamped t_target_to_base;

        try {
            // Lookup the latest from robot base to the whiteboard
            t_target_to_base = tf_buffer_->lookupTransform(
                base_frame_, target_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s",
                         base_frame_.c_str(), target_frame_.c_str(), ex.what());
            return;
        }
        
        rclcpp::Time current_time = t_target_to_base.header.stamp;

        if (last_time_.nanoseconds() == 0) {
            last_time_ = current_time;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) return;

        last_time_ = current_time;

        // Extract current pose and quaternion
        std::vector<double> current_pose = {
            t_target_to_base.transform.translation.x,
            t_target_to_base.transform.translation.y,
            t_target_to_base.transform.translation.z
        };

        tf2::Quaternion current_q(
            t_target_to_base.transform.rotation.x,
            t_target_to_base.transform.rotation.y,
            t_target_to_base.transform.rotation.z,
            t_target_to_base.transform.rotation.w
        );

        // Calculate raw velocity
        if (first_q_init_) {
            std::vector<double> raw_velocity(6, 0.0);

            // Linear velocity
            for (size_t i = 0; i < 3; ++i) {
                raw_velocity[i] = (current_pose[i] - prev_pose_vector_[i]) / dt;
            }

            // Angular velocity
            tf2::Quaternion delta_q = current_q * prev_q_.inverse();

            double angle = delta_q.getAngle();
            if (angle > M_PI) {
                angle -= 2.0 * M_PI; // Normalize to shortest path
            }
            tf2::Vector3 axis = delta_q.getAxis();

            raw_velocity[3] = (angle / dt) * axis.x();
            raw_velocity[4] = (angle / dt) * axis.y();
            raw_velocity[5] = (angle / dt) * axis.z();

            // Publish raw velocity
            geometry_msgs::msg::TwistStamped raw_twist_msg; 
            raw_twist_msg.header.stamp = current_time;
            raw_twist_msg.header.frame_id = base_frame_;
            raw_twist_msg.twist.linear.x = raw_velocity[0];
            raw_twist_msg.twist.linear.y = raw_velocity[1];
            raw_twist_msg.twist.linear.z = raw_velocity[2];
            raw_twist_msg.twist.angular.x = raw_velocity[3];
            raw_twist_msg.twist.angular.y = raw_velocity[4];
            raw_twist_msg.twist.angular.z = raw_velocity[5];
            raw_twist_pub_->publish(raw_twist_msg);

            // Run filter
            std::vector<double> filtered_velocity = active_filter_->update(raw_velocity, dt);

            // Publish filtered velocity
            geometry_msgs::msg::TwistStamped twist_msg;
            twist_msg.header.stamp = current_time;
            twist_msg.header.frame_id = base_frame_;
            twist_msg.twist.linear.x = filtered_velocity[0];
            twist_msg.twist.linear.y = filtered_velocity[1];
            twist_msg.twist.linear.z = filtered_velocity[2];
            twist_msg.twist.angular.x = filtered_velocity[3];
            twist_msg.twist.angular.y = filtered_velocity[4];
            twist_msg.twist.angular.z = filtered_velocity[5];
            twist_pub_->publish(twist_msg);

        }

        // Store for next loop iteration
        prev_pose_vector_ = current_pose;
        prev_q_ = current_q;
        first_q_init_ = true;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityEstimator>());
    rclcpp::shutdown();
    return 0;
}