#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "aimooe_msgs/msg/tool_array.hpp"
#include "surface_tracking_interfaces/msg/alignment_telemetry.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

namespace surface_tracking_aligner {

class RigidBodyAligner : public rclcpp::Node {
public:
    RigidBodyAligner() : Node("rigid_body_aligner") {
        // --- Declare Parameters ---
        this->declare_parameter<std::string>("tool_name", "");
        this->declare_parameter<std::vector<double>>("ordered_cad_points", {});
        this->declare_parameter<std::vector<double>>("ordered_marker_distances", {});
        this->declare_parameter<std::string>("align_camera", "");
        
        // --- Get Parameters ---
        tool_name = this->get_parameter("tool_name").as_string();
        align_camera = this->get_parameter("align_camera").as_string();
        actual_distances = this->get_parameter("ordered_marker_distances").as_double_array();

        std::vector<double> cad_points_flat = this->get_parameter("ordered_cad_points").as_double_array();
        if (cad_points_flat.empty() || cad_points_flat.size() % 3 != 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid or missing CAD points.");
            throw std::runtime_error("Missing calibrated CAD points.");
        }

        // Reshape flat vector into Nx3 matrix
        total_markers = cad_points_flat.size() / 3;
        cad_points.resize(total_markers, 3);
        for (int i = 0; i < total_markers; ++i) {
            cad_points(i, 0) = cad_points_flat[i*3 + 0];
            cad_points(i, 1) = cad_points_flat[i*3 + 1];
            cad_points(i, 2) = cad_points_flat[i*3 + 2];
        }

        // --- Setup TF2 ---
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer); 

        // --- Subscriber ---
        tool_info_sub = this->create_subscription<aimooe_msgs::msg::ToolArray>(
            "/aimooe/tool_info", 10,
            std::bind(&RigidBodyAligner::tool_info_callback, this, std::placeholders::_1)
        );

        // --- Publisher ---
        telemetry_pub = this->create_publisher<surface_tracking_interfaces::msg::AlignmentTelemetry>("~/tracking_telemetry", 10);

        RCLCPP_INFO(this->get_logger(), "C++ Aligner Started for: %s | Camera Mode: %s", tool_name.c_str(), align_camera.c_str());
    }

private:
    std::string tool_name;
    Eigen::MatrixXd cad_points;
    std::vector<double> actual_distances;
    int total_markers;
    std::string align_camera;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Subscription<aimooe_msgs::msg::ToolArray>::SharedPtr tool_info_sub;
    rclcpp::Publisher<surface_tracking_interfaces::msg::AlignmentTelemetry>::SharedPtr telemetry_pub;

    // Implementation of the Absolute Euclidean Error (MAE) from image_11e3bc.png
    double calculate_scale_mae(const std::vector<Eigen::Vector3d>& sensor_markers, const std::vector<int>& valid_indices) {
        if (actual_distances.empty() || actual_distances.size() != static_cast<size_t>(total_markers * total_markers)) {
            return -1.0; // Fail gracefully if YAML matrix is missing or wrong size
        }

        double sum_absolute_error = 0.0;
        int K = 0;

        // Iterate through all UNIQUE pairs of VISIBLE markers
        for (size_t i = 0; i < sensor_markers.size(); i++) {
            for (size_t j = i + 1; j < sensor_markers.size(); j++) {
                
                // 1. Calculate d_sensor (Live Euclidean distance)
                double d_sensor = (sensor_markers[i] - sensor_markers[j]).norm();

                // 2. Fetch d_actual safely using original CAD indices
                int idx_a = valid_indices[i];
                int idx_b = valid_indices[j];
                double d_actual = actual_distances[idx_a * total_markers + idx_b];

                // 3. Accumulate Error
                sum_absolute_error += std::abs(d_sensor - d_actual);
                K++;
            }
        }

        return (K > 0) ? (sum_absolute_error / K) : -1.0;
    }

    bool calculate_transform(const Eigen::MatrixXd& cad_pts, const Eigen::MatrixXd& cam_pts, Eigen::Isometry3d& transform_out, double& rms_error) {
        if (cad_pts.rows() != cam_pts.rows() || cad_pts.rows() < 3) return false;

        Eigen::RowVector3d centroid_cad = cad_pts.colwise().mean();
        Eigen::RowVector3d centroid_cam = cam_pts.colwise().mean();

        Eigen::MatrixXd centered_cad = cad_pts.rowwise() - centroid_cad;
        Eigen::MatrixXd centered_cam = cam_pts.rowwise() - centroid_cam;

        Eigen::Matrix3d H = centered_cad.transpose() * centered_cam;

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

        if (R.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1;
            R = V * svd.matrixU().transpose();
        }

        Eigen::Vector3d t = centroid_cam.transpose() - R * centroid_cad.transpose();

        transform_out.setIdentity();
        transform_out.linear() = R;
        transform_out.translation() = t;

        // Calulate RMS Error
        double squared_error_sum = 0.0;
        for (int i = 0; i < cad_pts.rows(); ++i) {
            Eigen::Vector3d transformed_cad = R * cad_pts.row(i).transpose() + t;
            squared_error_sum += (transformed_cad - cam_pts.row(i).transpose()).squaredNorm();
        }

        // Final RMSE
        rms_error = std::sqrt(squared_error_sum / cad_pts.rows());

        return true;
    }

    void tool_info_callback(const aimooe_msgs::msg::ToolArray::SharedPtr msg) {
        for (const auto& tool: msg->tools) {
            if (tool.tool_name == tool_name) {
                std::vector<Eigen::Vector3d> valid_cam_vec;
                std::vector<Eigen::Vector3d> valid_cad_vec;
                std::vector<int> valid_indices; // Track which specific markers are visible

                int iter_idx = 0;
                for (auto const& pt : tool.marker_points) {
                    if (!(pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0)) {
                        valid_cam_vec.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
                        valid_cad_vec.push_back(cad_points.row(iter_idx));
                        valid_indices.push_back(iter_idx);
                    }
                    iter_idx++;
                }

                if (valid_cam_vec.size() >= 3) {
                    Eigen::MatrixXd valid_cam_points(valid_cam_vec.size(), 3);
                    Eigen::MatrixXd valid_cad_points(valid_cad_vec.size(), 3);
                    for (size_t i = 0; i < valid_cam_vec.size(); ++i) {
                        valid_cam_points.row(i) = valid_cam_vec[i];
                        valid_cad_points.row(i) = valid_cad_vec[i];
                    }

                    Eigen::Isometry3d T_cad_to_cam;
                    double rms_error = 0.0;

                    if (!calculate_transform(valid_cad_points, valid_cam_points, T_cad_to_cam, rms_error)) {
                        return;
                    }

                    // --- NEW: Calculate the Scale MAE ---
                    double scale_mae = calculate_scale_mae(valid_cam_vec, valid_indices);

                    if (rms_error < 0.005) {
                        
                        // Print the validation metrics so you can verify them live!
                        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "[%s] RMSE: %.5f m | Scale MAE: %.5f m", tool_name.c_str(), rms_error, scale_mae);

                        // 1. Broadcast standard tool tracking
                        geometry_msgs::msg::TransformStamped t_tool;
                        t_tool.header.stamp = msg->header.stamp;
                        t_tool.header.frame_id = "aimooe_camera_link";
                        t_tool.child_frame_id = tool_name + "_aligned";
                        
                        t_tool.transform.translation.x = T_cad_to_cam.translation().x();
                        t_tool.transform.translation.y = T_cad_to_cam.translation().y();
                        t_tool.transform.translation.z = T_cad_to_cam.translation().z();
    
                        Eigen::Quaterniond q_tool(T_cad_to_cam.linear());
                        t_tool.transform.rotation.x = q_tool.x();
                        t_tool.transform.rotation.y = q_tool.y();
                        t_tool.transform.rotation.z = q_tool.z();
                        t_tool.transform.rotation.w = q_tool.w();
                        
                        tf_broadcaster->sendTransform(t_tool);
                        
                        // Publish telemetry data
                        surface_tracking_interfaces::msg::AlignmentTelemetry telemetry_msg;
                        telemetry_msg.header.stamp = msg->header.stamp;
                        telemetry_msg.header.frame_id = tool_name;

                        telemetry_msg.pose.position.x = T_cad_to_cam.translation().x();
                        telemetry_msg.pose.position.y = T_cad_to_cam.translation().y();
                        telemetry_msg.pose.position.z = T_cad_to_cam.translation().z();

                        telemetry_msg.pose.orientation.x = q_tool.x();
                        telemetry_msg.pose.orientation.y = q_tool.y();
                        telemetry_msg.pose.orientation.z = q_tool.z();
                        telemetry_msg.pose.orientation.w = q_tool.w();

                        telemetry_msg.rms_error = rms_error;

                        telemetry_msg.mean_abs_error = scale_mae;

                        telemetry_pub->publish(telemetry_msg);

                        // 2. Camera Alignment Logic
                        if (!align_camera.empty()) {
                            Eigen::Isometry3d T_cam_to_cad = T_cad_to_cam.inverse();
                            Eigen::Isometry3d final_camera_transform;
    
                            if (align_camera == "direct") {
                                final_camera_transform = T_cam_to_cad;
    
                            } else if (align_camera == "pivot") {
                                try {
                                    // Match the TF lookup to the exact camera timestamp
                                    geometry_msgs::msg::TransformStamped tf_msg = tf_buffer->lookupTransform(
                                        "elfin_base_link", "elfin_end_link", msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
                                    
                                    // Convert ROS msg to Eigen
                                    Eigen::Isometry3d T_ee_to_base = tf2::transformToEigen(tf_msg);
                                    
                                    // T_base_camera = T_base_ee * T_ee_camera
                                    final_camera_transform = T_ee_to_base * T_cam_to_cad;
                                    
                                } catch (const tf2::TransformException & ex) {
                                    RCLCPP_DEBUG(this->get_logger(), "Pivot TF Lookup failed: %s", ex.what());
                                    return;
                                }
                            }
    
                            // Broadcast Camera Base
                            geometry_msgs::msg::TransformStamped t_cam;
                            t_cam.header.stamp = msg->header.stamp;
                            t_cam.header.frame_id = "elfin_base_link";
                            t_cam.child_frame_id = "aimooe_camera_link";
    
                            t_cam.transform.translation.x = final_camera_transform.translation().x();
                            t_cam.transform.translation.y = final_camera_transform.translation().y();
                            t_cam.transform.translation.z = final_camera_transform.translation().z();
    
                            Eigen::Quaterniond q_cam(final_camera_transform.linear());
                            t_cam.transform.rotation.x = q_cam.x();
                            t_cam.transform.rotation.y = q_cam.y();
                            t_cam.transform.rotation.z = q_cam.z();
                            t_cam.transform.rotation.w = q_cam.w();
                            
                            tf_broadcaster->sendTransform(t_cam);
                        }
                    } else {
                        RCLCPP_DEBUG(this->get_logger(), "Tracking skipped. RMS Error too high: %.4f m", rms_error);
                    }
                }
            }
        }
    }
};

} // namespace surface_tracking_aligner

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<surface_tracking_aligner::RigidBodyAligner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}