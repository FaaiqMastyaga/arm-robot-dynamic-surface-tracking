#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp> // REQUIRED for TF to Eigen conversion
#include "aimooe_msgs/msg/tool_array.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>

using namespace std::chrono_literals;

namespace surface_tracking_aligner {

class RigidBodyAligner : public rclcpp::Node {
public:
    RigidBodyAligner() : Node("rigid_body_aligner") {
        // --- Declare Parameters ---
        this->declare_parameter<std::string>("tool_name", "");
        this->declare_parameter<std::vector<double>>("ordered_cad_points", {});
        this->declare_parameter<std::string>("align_camera", "");
        
        // --- Get Parameters ---
        tool_name = this->get_parameter("tool_name").as_string();
        align_camera = this->get_parameter("align_camera").as_string();

        std::vector<double> cad_points_flat = this->get_parameter("ordered_cad_points").as_double_array();
        if (cad_points_flat.empty() || cad_points_flat.size() % 3 != 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid or missing CAD points. Must be a flat array of 3D points.");
            throw std::runtime_error("Missing calibrated CAD points.");
        }

        // Reshape flat vector into Nx3 matrix
        int num_points = cad_points_flat.size() / 3;
        cad_points.resize(num_points, 3);
        for (int i = 0; i < num_points; ++i) {
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

        RCLCPP_INFO(this->get_logger(), "C++ Aligner Started for: %s | Camera Mode: %s", tool_name.c_str(), align_camera.c_str());
    }

private:
    std::string tool_name;
    Eigen::MatrixXd cad_points;
    std::string align_camera;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::Subscription<aimooe_msgs::msg::ToolArray>::SharedPtr tool_info_sub;

    // Kabsch algorithm implementation
    bool calculate_transform(const Eigen::MatrixXd& cad_pts, const Eigen::MatrixXd& cam_pts, Eigen::Isometry3d& transform_out) {
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

        return true;
    }

    void tool_info_callback(const aimooe_msgs::msg::ToolArray::SharedPtr msg) {
        for (const auto& tool: msg->tools) {
            if (tool.tool_name == tool_name) {
                std::vector<Eigen::Vector3d> valid_cam_vec;
                std::vector<Eigen::Vector3d> valid_cad_vec;

                int iter_idx = 0;
                for (auto const& pt : tool.marker_points) {
                    if (!(pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0)) {
                        valid_cam_vec.push_back(Eigen::Vector3d(pt.x / 1000.0, pt.y / 1000.0, pt.z / 1000.0));
                        valid_cad_vec.push_back(cad_points.row(iter_idx));
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
                    if (!calculate_transform(valid_cad_points, valid_cam_points, T_cad_to_cam)) {
                        return;
                    }

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