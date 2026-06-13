#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Dense>
#include <sched.h>
#include "surface_tracking_controller/mpc_osqp_solver.hpp"

using namespace std::chrono_literals;

namespace surface_tracking_controller {

class MPCTracker : public rclcpp::Node {
public:
    MPCTracker(const rclcpp::NodeOptions& options) : Node("mpc_tracking", options) 
    {
        // Declare Parameter
        this->declare_parameter<double>("control_loop_rate", 100.0);
        this->declare_parameter<double>("max_linear_vel", 0.4);
        this->declare_parameter<double>("max_angular_vel", 0.8);
        this->declare_parameter<double>("z_plunge_depth", 0.0075);
        this->declare_parameter<int>("mpc_horizon", 10);
        // Declare MPC Weight Vectors with safe defaults
        this->declare_parameter<std::vector<double>>("mpc_q_diagonal", 
            {500.0, 500.0, 500.0, 1000.0, 1000.0, 800.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this->declare_parameter<std::vector<double>>("mpc_r_diagonal", 
            {10.0, 10.0, 50.0, 500.0, 500.0, 500.0});

        // Fetch Parameter into class members
        control_loop_rate_ = this->get_parameter("control_loop_rate").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        z_plunge_depth_ = this->get_parameter("z_plunge_depth").as_double();
        prediction_horizon_ = this->get_parameter("mpc_horizon").as_int();
        double control_loop_period = 1000.0 / control_loop_rate_;

        // Setup Parameter Callback for dynamic updates
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MPCTracker::parameters_callback, this, std::placeholders::_1)
        );

        // Setup TF Buffer and Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Setup Publisher
        actual_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tracking/state/actual_pose", 10);
        actual_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/tracking/state/actual_twist", 10);
        ref_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tracking/state/ref_pose", 10);
        ref_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/tracking/state/ref_twist", 10);
        command_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/tracking/state/command_twist", 10);

        servo_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/tracking/delta_twist_cmds", 10);

        // Setup Subscriber
        planner_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/desired_drawing_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped& msg) {
                latest_target_pose_ = msg;
                has_target_pose_ = true;
            }
        );
        planner_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/desired_drawing_path", 10,
            [this](const nav_msgs::msg::Path& msg) {
                latest_target_path_ = msg;
                has_target_path_ = true;
            }
        );
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/estimated_target_pose/eskf", 10, 
            [this](const geometry_msgs::msg::PoseStamped& msg) {
                latest_eskf_pose_ = msg;
                has_eskf_pose_ = true;
            }
        );
        target_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/estimated_target_twist/eskf", 10, 
            [this](const geometry_msgs::msg::TwistStamped& msg) {
                latest_target_twist_ = msg;
                has_target_twist_ = true;
            }
        );
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, 
            std::bind(&MPCTracker::joint_state_callback, this, std::placeholders::_1)
        );

        // Initialize MPC Solver
        init_mpc();

        last_time_ = this->get_clock()->now();

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(control_loop_period)),
            std::bind(&MPCTracker::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "MPC Tracking node loaded.");
    }
    
private:
    // --- State Caches ---
    nav_msgs::msg::Path latest_target_path_;
    geometry_msgs::msg::PoseStamped latest_target_pose_;
    geometry_msgs::msg::PoseStamped latest_eskf_pose_;
    geometry_msgs::msg::TwistStamped latest_target_twist_;
    geometry_msgs::msg::Twist cartesian_twist_;
    bool has_target_path_ = false;
    bool has_target_pose_ = false;
    bool has_eskf_pose_ = false;
    bool has_target_twist_ = false;

    // MPC Variables
    int prediction_horizon_ = 10;  // Number of future steps to predict
    Eigen::MatrixXd A_k_;  // State transition matrix
    Eigen::MatrixXd B_;    // Control input matrix
    Eigen::MatrixXd Q_;  // State weighting matrix
    Eigen::MatrixXd R_;  // Control weighting matrix
    std::unique_ptr<MpcOsqpSolver> mpc_solver_;

    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    std::shared_ptr<moveit::core::RobotState> robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;

    // --- Control Parameters ---
    double control_loop_rate_, max_linear_vel_, max_angular_vel_, max_integral_, z_plunge_depth_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // --- State Publishers ---
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr actual_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr actual_twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ref_twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_twist_pub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_pub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planner_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr planner_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr target_twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Time last_time_;

    void init_mpc()
    {
        prediction_horizon_ = this->get_parameter("mpc_horizon").as_int();
        std::vector<double> q_vec = this->get_parameter("mpc_q_diagonal").as_double_array();
        std::vector<double> r_vec = this->get_parameter("mpc_r_diagonal").as_double_array();

        // Define Diagonal Weighting Matrices for MPC Cost Function
        // State vector: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]

        // State weighting matrix (Q)
        Q_ = Eigen::MatrixXd::Zero(12, 12);
        Eigen::VectorXd Q_diag(12);
        if (q_vec.size() == 12) {
            for(int i=0; i<12; ++i) Q_diag(i) = q_vec[i];
        } else {
            RCLCPP_ERROR(this->get_logger(), "mpc_q_diagonal must have exactly 12 elements!");
        }
        Q_.diagonal() = Q_diag;

        // Control weighting matrix (R)
        R_ = Eigen::MatrixXd::Zero(6, 6);
        Eigen::VectorXd R_diag(6);
        if (r_vec.size() == 6) {
            for(int i=0; i<6; ++i) R_diag(i) = r_vec[i];
        } else {
            RCLCPP_ERROR(this->get_logger(), "mpc_r_diagonal must have exactly 6 elements!");
        }
        R_.diagonal() = R_diag;

        // Initialize the MPC Solver
        mpc_solver_ = std::make_unique<MpcOsqpSolver>(prediction_horizon_, 12, 6);
        mpc_solver_->setWeights(Q_, R_);
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters) 
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        bool update_weights = false;

        for (const auto &param : parameters) {
            if (param.get_name() == "max_linear_vel") {
                max_linear_vel_ = param.as_double();
            } else if (param.get_name() == "max_angular_vel") {
                max_angular_vel_ = param.as_double();
            } else if (param.get_name() == "mpc_horizon") {
                prediction_horizon_ = param.as_int();
            } else if (param.get_name() == "mpc_q_diagonal") {
                std::vector<double> q_vec = param.as_double_array();
                if (q_vec.size() == 12) {
                    for(int i=0; i<12; ++i) Q_(i,i) = q_vec[i];
                    update_weights = true;
                }
            } else if (param.get_name() == "mpc_r_diagonal") {
                std::vector<double> r_vec = param.as_double_array();
                if (r_vec.size() == 6) {
                    for(int i=0; i<6; ++i) R_(i,i) = r_vec[i];
                    update_weights = true;
                }
            }
        }

        // Push new weights to the solver instantly
        if (update_weights && mpc_solver_) {
            mpc_solver_->setWeights(Q_, R_);
            RCLCPP_INFO(this->get_logger(), "MPC Weights Updated Live!");
        }

        return result;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
        if (!robot_state_) return;

        // Prevent crashes if the hardware drops a velocity frame
        if (msg->velocity.empty()) return;

        // Safely map the incoming joint states to the MoveIt kinematic tree.
        // MoveIt handles the scrambled string matching internally.
        robot_state_->setVariablePositions(msg->name, msg->position);
        robot_state_->setVariableVelocities(msg->name, msg->velocity);

        // --- THE CRITICAL FIX ---
        // Force MoveIt to compute the Forward Kinematics for the new state
        robot_state_->update();

        // Extract the Jacobian for the pen tip
        Eigen::MatrixXd jacobian;
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        robot_state_->getJacobian(
            joint_model_group_, 
            kinematic_model_->getLinkModel("pen_tip_link"),
            reference_point_position,
            jacobian
        );

        // Extract the properly sorted joint velocities from MoveIt
        Eigen::VectorXd joint_velocities;
        robot_state_->copyJointGroupVelocities(joint_model_group_, joint_velocities);

        // Compute cartesian twist
        Eigen::VectorXd cartesian_twist = jacobian * joint_velocities;

        // Extract the 6D twist (linear + angular)
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->get_clock()->now();
        twist_msg.header.frame_id = "elfin_base_link";

        twist_msg.twist.linear.x = cartesian_twist(0);
        twist_msg.twist.linear.y = cartesian_twist(1);
        twist_msg.twist.linear.z = cartesian_twist(2);
        twist_msg.twist.angular.x = cartesian_twist(3);
        twist_msg.twist.angular.y = cartesian_twist(4);
        twist_msg.twist.angular.z = cartesian_twist(5);

        cartesian_twist_ = twist_msg.twist;

        // Publish
        actual_twist_pub_->publish(twist_msg);
    }

    void control_loop()
    {
        if (!robot_model_loader_) {
            robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
                this->shared_from_this(), "robot_description");
            kinematic_model_ = robot_model_loader_->getModel();
            if (kinematic_model_) {
                robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
                joint_model_group_ = kinematic_model_->getJointModelGroup("elfin_arm");
                RCLCPP_INFO(this->get_logger(), "Robot model loaded successfully for Jacobian calculations.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to load robot model. Jacobian twist logging will not work.");
            }
        }

        // ==========================================
        // TARGET SELECTION LOGIC
        // ==========================================
        // If we have a direct Pose goal (Hover), prioritize it over the Path
        if (has_target_pose_) {
            // Treat the single Pose as a Path of length 1
            latest_target_path_.poses.clear();
            latest_target_path_.poses.push_back(latest_target_pose_);
            latest_target_path_.header = latest_target_pose_.header;
            has_target_path_ = true;
            // Clear the pose so we don't re-process it every tick
            has_target_pose_ = false; 
        }
        
        // Don't act unless we have an active drawing target
        if (!has_target_path_ || latest_target_path_.poses.empty()) return;

        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0) return;

        // Base frame for the robot (Fixed reference)
        const std::string base_frame = "elfin_base_link";
        // End effector frame (the moving part)
        const std::string ee_frame = "pen_tip_link";

        try {
            // ==========================================
            // GET CURRENT ROBOT STATE (x_k)
            // ==========================================
            geometry_msgs::msg::TransformStamped current_transform = tf_buffer_->lookupTransform(
                base_frame, ee_frame, tf2::TimePointZero);

            double current_x = current_transform.transform.translation.x;
            double current_y = current_transform.transform.translation.y;
            double current_z = current_transform.transform.translation.z;

            tf2::Quaternion q_current;
            tf2::fromMsg(current_transform.transform.rotation, q_current);

            tf2::Matrix3x3 m_current(q_current);
            double phi_k, theta_k, psi_k;
            m_current.getRPY(phi_k, theta_k, psi_k);

            double current_vx = cartesian_twist_.linear.x;
            double current_vy = cartesian_twist_.linear.y;
            double current_vz = cartesian_twist_.linear.z;
            double current_wx = cartesian_twist_.angular.x;
            double current_wy = cartesian_twist_.angular.y;
            double current_wz = cartesian_twist_.angular.z;

            Eigen::VectorXd x_k(12);
            x_k << current_x, current_y, current_z, phi_k, theta_k, psi_k, 
                    current_vx, current_vy, current_vz, current_wx, current_wy, current_wz;

            // ==========================================
            // GET TARGET STATE & GENERATE HORIZON (x_ref)
            // ==========================================
            Eigen::MatrixXd R_ref = Eigen::MatrixXd::Zero(12, prediction_horizon_);

            if (!has_eskf_pose_ || !has_target_twist_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "Waiting for ESKF data...");

                geometry_msgs::msg::TwistStamped stop_twist;
                stop_twist.header.stamp = current_time;
                stop_twist.header.frame_id = base_frame;
                servo_twist_pub_->publish(stop_twist);
                command_twist_pub_->publish(stop_twist);
                return;
            }

            // If the ESKF data is older than 0.1 seconds, tracking is lost.
            double data_age = (current_time - rclcpp::Time(latest_eskf_pose_.header.stamp)).seconds();
            if (data_age > 0.1) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "ESKF data is stale (age: %.3f s). Target occluded or node died. Halting.", data_age);
                
                geometry_msgs::msg::TwistStamped stop_twist;
                stop_twist.header.stamp = current_time;
                stop_twist.header.frame_id = base_frame;
                servo_twist_pub_->publish(stop_twist);
                command_twist_pub_->publish(stop_twist);
                return;
            }
            
            // Extract the moving whiteboard's velocity (from the Kalman Filter)
            double wb_vx = has_target_twist_ ? latest_target_twist_.twist.linear.x : 0.0;
            double wb_vy = has_target_twist_ ? latest_target_twist_.twist.linear.y : 0.0;
            double wb_vz = has_target_twist_ ? latest_target_twist_.twist.linear.z : 0.0;
            double wb_wx = has_target_twist_ ? latest_target_twist_.twist.angular.x : 0.0;
            double wb_wy = has_target_twist_ ? latest_target_twist_.twist.angular.y : 0.0;
            double wb_wz = has_target_twist_ ? latest_target_twist_.twist.angular.z : 0.0;
            
            // Determine how many points we can safely pull from the path message
            int path_len = std::min(prediction_horizon_, (int)latest_target_path_.poses.size());
            
            // Manually build the transform from the ESKF pose
            geometry_msgs::msg::TransformStamped target_transform;
            target_transform.header.stamp = current_time;
            target_transform.header.frame_id = base_frame;
            target_transform.child_frame_id = latest_target_path_.header.frame_id;

            target_transform.transform.translation.x = latest_eskf_pose_.pose.position.x;
            target_transform.transform.translation.y = latest_eskf_pose_.pose.position.y;
            target_transform.transform.translation.z = latest_eskf_pose_.pose.position.z;
            target_transform.transform.rotation = latest_eskf_pose_.pose.orientation;

            for (int i = 0; i < path_len; ++i) {
                double future_time = i * dt;

                geometry_msgs::msg::PoseStamped target_in_base;
                tf2::doTransform(latest_target_path_.poses[i], target_in_base, target_transform);
    
                tf2::Quaternion q_target;
                tf2::fromMsg(target_in_base.pose.orientation, q_target);
    
                // Apply z-axis plunge depth
                double preload_offset = -z_plunge_depth_;
                tf2::Vector3 z_axis_local(0, 0, 1);
                tf2::Vector3 z_axis_global = tf2::quatRotate(q_target, z_axis_local);
    
                // 1. Shift the spatial setpoint by the whiteboard's predicted movement over time 
                double setpoint_x = target_in_base.pose.position.x + (preload_offset * z_axis_global.x()) + (wb_vx * future_time);
                double setpoint_y = target_in_base.pose.position.y + (preload_offset * z_axis_global.y()) + (wb_vy * future_time);
                double setpoint_z = target_in_base.pose.position.z + (preload_offset * z_axis_global.z()) + (wb_vz * future_time);
    
                // 2. Shift the orientation setpoint
                tf2::Matrix3x3 m_target(q_target);
                double base_phi, base_theta, base_psi;
                m_target.getRPY(base_phi, base_theta, base_psi);

                double target_phi = base_phi + (wb_wx * future_time);
                double target_theta = base_theta + (wb_wy * future_time);
                double target_psi = base_psi + (wb_wz * future_time);

                // 3. Calculate combined target velocity (board speed + drawing speed)
                double total_vx = wb_vx;
                double total_vy = wb_vy;
                double total_vz = wb_vz;

                if (i > 0 && dt > 0.001) {
                    total_vx = (setpoint_x - R_ref(0, i-1)) / dt;
                    total_vy = (setpoint_y - R_ref(1, i-1)) / dt;
                    total_vz = (setpoint_z - R_ref(2, i-1)) / dt;
                }

                R_ref.col(i) << setpoint_x, setpoint_y, setpoint_z,
                                target_phi, target_theta, target_psi,
                                total_vx, total_vy, total_vz,
                                wb_wx, wb_wy, wb_wz;
            }

            // Pad the remaining horizon if the path ends
            for (int i = path_len; i < prediction_horizon_; ++i) {
                // Start by copying the previous time step
                R_ref.col(i) = R_ref.col(i - 1); 
                
                // Shift the spatial coordinates by the board's ongoing movement
                R_ref(0, i) += wb_vx * (1.0 / control_loop_rate_);
                R_ref(1, i) += wb_vy * (1.0 / control_loop_rate_);
                R_ref(2, i) += wb_vz * (1.0 / control_loop_rate_);
                R_ref(3, i) += wb_wx * (1.0 / control_loop_rate_);
                R_ref(4, i) += wb_wy * (1.0 / control_loop_rate_);
                R_ref(5, i) += wb_wz * (1.0 / control_loop_rate_);

                // The required velocity is now exactly the board's velocity
                R_ref(6, i) = wb_vx;
                R_ref(7, i) = wb_vy;
                R_ref(8, i) = wb_vz;
                R_ref(9, i) = wb_wx;
                R_ref(10, i) = wb_wy;
                R_ref(11, i) = wb_wz;
            }

            // ==========================================
            // SUCCESSIVE LINEARIZATION
            // ==========================================
            if (std::abs(theta_k) > 1.50) {
                theta_k = (theta_k > 0) ? 1.50 : -1.50;
            }

            double sin_phi = std::sin(phi_k);
            double cos_phi = std::cos(phi_k);
            double tan_theta = std::tan(theta_k);
            double sec_theta = 1.0 / std::cos(theta_k);

            Eigen::Matrix3d T;
            T << 1,  sin_phi * tan_theta,  cos_phi * tan_theta,
                 0,  cos_phi,             -sin_phi,
                 0,  sin_phi * sec_theta,  cos_phi * sec_theta;

            Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6, 6);
            M.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
            M.bottomRightCorner(3, 3) = T;

            double tau = 0.05;
            double alpha = dt / tau;

            A_k_ = Eigen::MatrixXd::Zero(12, 12);
            A_k_.topLeftCorner(6, 6) = Eigen::MatrixXd::Identity(6, 6);
            A_k_.topRightCorner(6, 6) = M * dt;
            A_k_.bottomRightCorner(6, 6) = (1.0 - alpha) * Eigen::MatrixXd::Identity(6, 6);

            B_ = Eigen::MatrixXd::Zero(12, 6);
            B_.bottomRows(6) =  alpha * Eigen::MatrixXd::Identity(6, 6);

            // ==========================================
            // MPC OPTIMIZATION PROBLEM
            // ==========================================
            Eigen::VectorXd u_optimal(6);
            Eigen::VectorXd u_max(6);
            u_max << max_linear_vel_, max_linear_vel_, max_linear_vel_,
                     max_angular_vel_, max_angular_vel_, max_angular_vel_;
            Eigen::VectorXd u_min = -u_max;

            bool success = mpc_solver_->solve(A_k_, B_, x_k, R_ref, u_min, u_max, u_optimal);
            
            if (!success) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "MPC Solver Failed!");
                u_optimal = Eigen::VectorXd::Zero(6);
            }

            // ==========================================
            // STATE & COMMAND PUBLISHING
            // ==========================================
            geometry_msgs::msg::PoseStamped actual_pose_msg;
            actual_pose_msg.header.stamp = current_time;
            actual_pose_msg.header.frame_id = base_frame;
            actual_pose_msg.pose.position.x = current_x;
            actual_pose_msg.pose.position.y = current_y;
            actual_pose_msg.pose.position.z = current_z;
            actual_pose_msg.pose.orientation = current_transform.transform.rotation;
            actual_pose_pub_->publish(actual_pose_msg);

            geometry_msgs::msg::PoseStamped ref_pose_msg;
            ref_pose_msg.header.stamp = current_time;
            ref_pose_msg.header.frame_id = base_frame;
            ref_pose_msg.pose.position.x = R_ref(0, 0);
            ref_pose_msg.pose.position.y = R_ref(1, 0);
            ref_pose_msg.pose.position.z = R_ref(2, 0);
            
            // Reconstruct orientation for publisher
            tf2::Quaternion q_ref;
            q_ref.setRPY(R_ref(3, 0), R_ref(4, 0), R_ref(5, 0));
            ref_pose_msg.pose.orientation = tf2::toMsg(q_ref);
            ref_pose_pub_->publish(ref_pose_msg);

            geometry_msgs::msg::TwistStamped ref_twist_msg;
            ref_twist_msg.header.stamp = current_time;
            ref_twist_msg.header.frame_id = base_frame;
            ref_twist_msg.twist.linear.x = R_ref(6, 0);
            ref_twist_msg.twist.linear.y = R_ref(7, 0);
            ref_twist_msg.twist.linear.z = R_ref(8, 0);
            ref_twist_msg.twist.angular.x = R_ref(9, 0);
            ref_twist_msg.twist.angular.y = R_ref(10, 0);
            ref_twist_msg.twist.angular.z = R_ref(11, 0);
            ref_twist_pub_->publish(ref_twist_msg);

            // ==========================================
            // COMMAND CLAMPING & PUBLISHING
            // ==========================================
            geometry_msgs::msg::TwistStamped cmd_twist;
            cmd_twist.header.stamp = current_time;
            cmd_twist.header.frame_id = base_frame; 

            cmd_twist.twist.linear.x = std::clamp(u_optimal(0), -max_linear_vel_, max_linear_vel_);
            cmd_twist.twist.linear.y = std::clamp(u_optimal(1), -max_linear_vel_, max_linear_vel_);
            cmd_twist.twist.linear.z = std::clamp(u_optimal(2), -max_linear_vel_, max_linear_vel_);
            
            cmd_twist.twist.angular.x = std::clamp(u_optimal(3), -max_angular_vel_, max_angular_vel_);
            cmd_twist.twist.angular.y = std::clamp(u_optimal(4), -max_angular_vel_, max_angular_vel_);
            cmd_twist.twist.angular.z = std::clamp(u_optimal(5), -max_angular_vel_, max_angular_vel_);

            servo_twist_pub_->publish(cmd_twist);
            command_twist_pub_->publish(cmd_twist);

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Robot Kinematics TF missing! Cannot find end-effector: %s", ex.what());

            geometry_msgs::msg::TwistStamped stop_twist;
            stop_twist.header.stamp = current_time;
            stop_twist.header.frame_id = base_frame;
            servo_twist_pub_->publish(stop_twist);
            command_twist_pub_->publish(stop_twist);
        }
    }
};

}  // namespace surface_tracking_controller

int main(int argc, char * argv[])
{
    // --- NATIVE REAL-TIME THREAD ELEVATION ---
    struct sched_param param;
    param.sched_priority = 90; 
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("Failed to set Real-Time Priority for MPC Tracker. Are you running as root?");
    }
    // -----------------------------------------
    
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<surface_tracking_controller::MPCTracker>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}