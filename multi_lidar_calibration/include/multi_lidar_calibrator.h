#ifndef PROJECT_MULTI_LIDAR_CALIBRATOR_H
#define PROJECT_MULTI_LIDAR_CALIBRATOR_H

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <fstream>
#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#define __APP_NAME__ "multi_lidar_calibrator"

class ROSMultiLidarCalibratorApp : public rclcpp::Node
{
public:
    ROSMultiLidarCalibratorApp();

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr calibrated_cloud_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    int child_topic_num_;

    std::map<std::string, std::vector<double>> transfer_map_;

    std::string points_parent_topic_str_, points_child_topic_str_;

    double voxel_size_;
    double ndt_epsilon_;
    double ndt_step_size_;
    double ndt_resolution_;
    std::vector<double> ndt_resolution_levels_;
    bool use_multi_resolution_;

    double initial_x_;
    double initial_y_;
    double initial_z_;
    double initial_roll_;
    double initial_pitch_;
    double initial_yaw_;

    int ndt_iterations_;
    int max_rejections_before_reset_;
    int consecutive_rejections_;

    std::string parent_frame_;
    std::string child_frame_;

    Eigen::Matrix4f current_guess_;
    Eigen::Matrix4f configured_initial_guess_;
    Eigen::Matrix4f best_transformation_;
    double best_fitness_score_;
    double best_robust_score_;
    bool has_initialized_guess_;

    double max_fitness_score_;
    double min_inlier_ratio_;
    double min_transform_probability_;
    double max_correspondence_distance_;
    double max_translation_jump_;
    double max_rotation_jump_deg_;

    typedef pcl::PointXYZ PointT;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2> SyncPolicyT;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_parent_subscriber_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_child_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicyT>> cloud_synchronizer_;

    pcl::PointCloud<PointT>::Ptr in_parent_cloud_;
    pcl::PointCloud<PointT>::Ptr in_parent_filtered_cloud_;
    pcl::PointCloud<PointT>::Ptr in_child_cloud_;
    pcl::PointCloud<PointT>::Ptr in_child_filtered_cloud_;

    /*!
     * Receives 2 synchronized point cloud messages.
     * @param[in] in_parent_cloud_msg Message containing pointcloud classified as ground.
     * @param[in] in_child_cloud_msg Message containing pointcloud classified as obstacle.
     */
    void PointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr in_parent_cloud_msg,
                        const sensor_msgs::msg::PointCloud2::ConstSharedPtr in_child_cloud_msg);

    /*!
     * Obtains parameters, initializes subscribers and publishers.
     */
    void InitializeROSIo();

    /*!
     * Applies a Voxel Grid filter to the point cloud
     * @param in_cloud_ptr point cloud to downsample
     * @param out_cloud_ptr downsampled point cloud
     * @param in_leaf_size voxel side size
     */
    void DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                         pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
                         double in_leaf_size);

    /*!
     * Publishes a PointCloud
     * @param in_cloud_to_publish_ptr Cloud to Publish
     */
    void PublishCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr);

    void MatrixToTransform(const Eigen::Matrix4f& matrix, geometry_msgs::msg::TransformStamped& transform_stamped);

    Eigen::Matrix4f BuildConfiguredInitialGuess() const;

    std::vector<double> GetNdtResolutionLevels() const;

    double ComputeInlierRatio(const pcl::PointCloud<PointT>::ConstPtr& source_cloud,
                              const pcl::PointCloud<PointT>::ConstPtr& target_cloud,
                              double max_correspondence_distance) const;

    bool IsTransformJumpAcceptable(const Eigen::Matrix4f& previous_transform,
                                   const Eigen::Matrix4f& candidate_transform,
                                   double* translation_jump,
                                   double* rotation_jump_deg) const;

    double ComputeRobustScore(double fitness_score, double inlier_ratio, double transform_probability) const;

    void PerformNdtOptimize();

    void TimerCallback();
};

#endif //PROJECT_MULTI_LIDAR_CALIBRATOR_H
