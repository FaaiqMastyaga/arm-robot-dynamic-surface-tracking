#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

namespace surface_tracking_estimator {

class ESKFVelocityEstimator : public rclcpp::Node {
public:
    ESKFVelocityEstimator(const rclcpp::NodeOptions& options) : Node("velocity_estimator_eskf", options) 
    {
        // --- Declare Parameters ---
        this->declare_parameter<std::string>("target_frame", "whiteboard");
        this->declare_parameter<std::string>("base_frame", "elfin_base_link");
        this->declare_parameter<double>("estimation_rate", 100.0);

        // Tuning Knobs: Measurement Noise (R) -> Distrust of the Aimooe Sensor
        this->declare_parameter<double>("r_trans_noise", 0.05); // High = Ignore optical jitter
        this->declare_parameter<double>("r_rot_noise", 0.05);

        // Tuning Knobs: Process Noise (Q) -> Distrust of the Constant Velocity Physics Model
        this->declare_parameter<double>("q_trans_pos", 0.001);
        this->declare_parameter<double>("q_trans_vel", 0.1);    // High = Board can accelerate quickly
        this->declare_parameter<double>("q_rot_pos", 0.001);
        this->declare_parameter<double>("q_rot_vel", 0.5);

        // --- Get Parameters ---
        target_frame_ = this->get_parameter("target_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        double rate = this->get_parameter("estimation_rate").as_double();
        dt_ = 1.0 / rate;

        // --- TF Setup ---
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- Publishers ---
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/estimated_target_twist/eskf", 10);
        pose_pub_  = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_target_pose/eskf", 10);

        // --- Initialize Filters ---
        init_filters();

        // --- High Frequency Timer ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&ESKFVelocityEstimator::filter_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Full-State Kinematic ESKF Initialized. Tracking [%s] relative to [%s]", 
                    target_frame_.c_str(), base_frame_.c_str());
    }

private:
    std::string target_frame_, base_frame_;
    double dt_;
    bool is_initialized_ = false;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // ==========================================
    // TRANSLATION LKF (6-State)
    // ==========================================
    Eigen::VectorXd x_trans_; // [x, y, z, vx, vy, vz]^T
    Eigen::MatrixXd P_trans_;
    Eigen::MatrixXd F_trans_;
    Eigen::MatrixXd H_trans_;
    Eigen::MatrixXd Q_trans_;
    Eigen::MatrixXd R_trans_;

    // ==========================================
    // ROTATION ESKF (Nominal + 6-State Error)
    // ==========================================
    Eigen::Quaterniond q_nom_;    // Nominal Orientation
    Eigen::Vector3d omega_nom_;   // Nominal Angular Velocity [wx, wy, wz]^T
    Eigen::MatrixXd P_rot_;       // Covariance of the 6D error state
    Eigen::MatrixXd F_rot_;
    Eigen::MatrixXd H_rot_;
    Eigen::MatrixXd Q_rot_;
    Eigen::MatrixXd R_rot_;

    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters) 
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters) {
            if (param.get_name() == "r_trans_noise") {
                double val = param.as_double();
                R_trans_ = Eigen::Matrix3d::Identity() * val;
                RCLCPP_INFO(this->get_logger(), "Updated Translation Sensor Noise (R) to %f", val);
            } 
            else if (param.get_name() == "r_rot_noise") {
                double val = param.as_double();
                R_rot_ = Eigen::Matrix3d::Identity() * val;
                RCLCPP_INFO(this->get_logger(), "Updated Rotation Sensor Noise (R) to %f", val);
            } 
            else if (param.get_name() == "q_trans_pos") {
                double val = param.as_double();
                Q_trans_.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() * val;
                RCLCPP_INFO(this->get_logger(), "Updated Translation Process Noise (Q_pos) to %f", val);
            } 
            else if (param.get_name() == "q_trans_vel") {
                double val = param.as_double();
                Q_trans_.bottomRightCorner(3, 3) = Eigen::Matrix3d::Identity() * val;
                RCLCPP_INFO(this->get_logger(), "Updated Translation Process Noise (Q_vel) to %f", val);
            }
            else if (param.get_name() == "q_rot_pos") {
                double val = param.as_double();
                Q_rot_.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() * val;
                RCLCPP_INFO(this->get_logger(), "Updated Rotation Process Noise (Q_pos) to %f", val);
            }
            else if (param.get_name() == "q_rot_vel") {
                double val = param.as_double();
                Q_rot_.bottomRightCorner(3, 3) = Eigen::Matrix3d::Identity() * val;
                RCLCPP_INFO(this->get_logger(), "Updated Rotation Process Noise (Q_vel) to %f", val);
            }
        }
        return result;
    }
    
    void init_filters() 
    {
        double r_t = this->get_parameter("r_trans_noise").as_double();
        double q_t_p = this->get_parameter("q_trans_pos").as_double();
        double q_t_v = this->get_parameter("q_trans_vel").as_double();

        double r_r = this->get_parameter("r_rot_noise").as_double();
        double q_r_p = this->get_parameter("q_rot_pos").as_double();
        double q_r_v = this->get_parameter("q_rot_vel").as_double();

        // 1. Setup Translation Filter
        x_trans_ = Eigen::VectorXd::Zero(6);
        P_trans_ = Eigen::MatrixXd::Identity(6, 6);

        F_trans_ = Eigen::MatrixXd::Identity(6, 6);
        F_trans_.topRightCorner(3, 3) = Eigen::Matrix3d::Identity() * dt_;

        H_trans_ = Eigen::MatrixXd::Zero(3, 6);
        H_trans_.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();

        Q_trans_ = Eigen::MatrixXd::Zero(6, 6);
        Q_trans_.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() * q_t_p;
        Q_trans_.bottomRightCorner(3, 3) = Eigen::Matrix3d::Identity() * q_t_v;

        R_trans_ = Eigen::Matrix3d::Identity() * r_t;

        // 2. Setup Rotation Filter
        q_nom_ = Eigen::Quaterniond::Identity();
        omega_nom_ = Eigen::Vector3d::Zero();
        P_rot_ = Eigen::MatrixXd::Identity(6, 6);

        F_rot_ = Eigen::MatrixXd::Identity(6, 6);
        F_rot_.topRightCorner(3, 3) = Eigen::Matrix3d::Identity() * dt_;

        H_rot_ = Eigen::MatrixXd::Zero(3, 6);
        H_rot_.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();

        Q_rot_ = Eigen::MatrixXd::Zero(6, 6);
        Q_rot_.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() * q_r_p;
        Q_rot_.bottomRightCorner(3, 3) = Eigen::Matrix3d::Identity() * q_r_v;

        R_rot_ = Eigen::Matrix3d::Identity() * r_r;

        // Setup Parameter Callback for live tuning
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ESKFVelocityEstimator::parameters_callback, this, std::placeholders::_1)
        );
    }

    void filter_loop()
    {
        geometry_msgs::msg::TransformStamped t_meas;
        try {
            // Get raw optical tracker data
            t_meas = tf_buffer_->lookupTransform(base_frame_, target_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            // If occluded, do not update. The physics model will Coast.
            return;
        }

        Eigen::Vector3d z_trans(t_meas.transform.translation.x, 
                                t_meas.transform.translation.y, 
                                t_meas.transform.translation.z);

        Eigen::Quaterniond z_rot(t_meas.transform.rotation.w, 
                                 t_meas.transform.rotation.x, 
                                 t_meas.transform.rotation.y, 
                                 t_meas.transform.rotation.z);

        if (!is_initialized_) {
            // Hot-start the filters
            x_trans_.head<3>() = z_trans;
            q_nom_ = z_rot;
            is_initialized_ = true;
            return;
        }

        // ==========================================
        // STEP 1: PREDICT (The Physics Model)
        // ==========================================
        
        // Translation Prediction
        Eigen::VectorXd x_trans_pred = F_trans_ * x_trans_;
        Eigen::MatrixXd P_trans_pred = F_trans_ * P_trans_ * F_trans_.transpose() + Q_trans_;

        // Rotation Prediction (Nominal State Kinematics)
        Eigen::Quaterniond q_pred;
        double omega_norm = omega_nom_.norm();
        if (omega_norm > 1e-6) {
            Eigen::AngleAxisd d_theta(omega_norm * dt_, omega_nom_.normalized());
            q_pred = q_nom_ * Eigen::Quaterniond(d_theta);
        } else {
            // First-order approximation to prevent div-by-zero
            q_pred = q_nom_ * Eigen::Quaterniond(1.0, omega_nom_.x()*dt_/2, omega_nom_.y()*dt_/2, omega_nom_.z()*dt_/2);
        }
        q_pred.normalize();

        // Error State Covariance Prediction
        Eigen::MatrixXd P_rot_pred = F_rot_ * P_rot_ * F_rot_.transpose() + Q_rot_;

        // ==========================================
        // STEP 2: MEASURE & UPDATE (Kalman Gain)
        // ==========================================

        // Translation Update
        Eigen::Vector3d y_trans = z_trans - (H_trans_ * x_trans_pred);
        Eigen::MatrixXd S_trans = H_trans_ * P_trans_pred * H_trans_.transpose() + R_trans_;
        Eigen::MatrixXd K_trans = P_trans_pred * H_trans_.transpose() * S_trans.inverse();
        
        x_trans_ = x_trans_pred + (K_trans * y_trans);
        P_trans_ = (Eigen::MatrixXd::Identity(6, 6) - K_trans * H_trans_) * P_trans_pred;

        // Rotation Update (ESKF)
        // 1. Calculate the Error Quaternion between prediction and sensor
        Eigen::Quaterniond q_err = q_pred.conjugate() * z_rot;
        
        // Ensure shortest path rotation (force w to be positive)
        if (q_err.w() < 0) {
            q_err.coeffs() *= -1.0;
        }

        // 2. Extract the 3D Error Vector (y)
        Eigen::Vector3d y_rot = 2.0 * q_err.vec();

        // 3. Compute Kalman Gain for the Error State
        Eigen::MatrixXd S_rot = H_rot_ * P_rot_pred * H_rot_.transpose() + R_rot_;
        Eigen::MatrixXd K_rot = P_rot_pred * H_rot_.transpose() * S_rot.inverse();

        // 4. Solve for the Error State: delta_x = [delta_theta, delta_omega]^T
        Eigen::VectorXd delta_x = K_rot * y_rot;
        Eigen::Vector3d delta_theta = delta_x.head<3>();
        Eigen::Vector3d delta_omega = delta_x.tail<3>();

        // Update Rotation Covariance
        P_rot_ = (Eigen::MatrixXd::Identity(6, 6) - K_rot * H_rot_) * P_rot_pred;

        // ==========================================
        // STEP 3: INJECTION (Fix the Nominal State)
        // ==========================================
        
        double d_theta_norm = delta_theta.norm();
        if (d_theta_norm > 1e-6) {
            Eigen::AngleAxisd dq(d_theta_norm, delta_theta.normalized());
            q_nom_ = q_pred * Eigen::Quaterniond(dq);
        } else {
            q_nom_ = q_pred * Eigen::Quaterniond(1.0, delta_theta.x()/2, delta_theta.y()/2, delta_theta.z()/2);
        }
        q_nom_.normalize();

        // Inject velocity error
        omega_nom_ = omega_nom_ + delta_omega;
        
        // Note: We implicitly set delta_x back to 0 here by not retaining it across loops.

        // ==========================================
        // STEP 4: PUBLISH RESULTS
        // ==========================================
        rclcpp::Time current_time = this->get_clock()->now();

        // Publish Pristine Twist (Velocity)
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = current_time;
        twist_msg.header.frame_id = base_frame_;
        twist_msg.twist.linear.x = x_trans_(3);
        twist_msg.twist.linear.y = x_trans_(4);
        twist_msg.twist.linear.z = x_trans_(5);
        twist_msg.twist.angular.x = omega_nom_.x();
        twist_msg.twist.angular.y = omega_nom_.y();
        twist_msg.twist.angular.z = omega_nom_.z();
        twist_pub_->publish(twist_msg);

        // Optional: Publish the Filtered Pose
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = base_frame_;
        pose_msg.pose.position.x = x_trans_(0);
        pose_msg.pose.position.y = x_trans_(1);
        pose_msg.pose.position.z = x_trans_(2);
        pose_msg.pose.orientation.w = q_nom_.w();
        pose_msg.pose.orientation.x = q_nom_.x();
        pose_msg.pose.orientation.y = q_nom_.y();
        pose_msg.pose.orientation.z = q_nom_.z();
        pose_pub_->publish(pose_msg);
    }
};

} // namespace surface_tracking_estimator

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<surface_tracking_estimator::ESKFVelocityEstimator>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}