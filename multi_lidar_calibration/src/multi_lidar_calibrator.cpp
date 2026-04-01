#include "multi_lidar_calibrator.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl/kdtree/kdtree_flann.h>

ROSMultiLidarCalibratorApp::ROSMultiLidarCalibratorApp()
    : Node(__APP_NAME__)
{
    current_guess_ = Eigen::Matrix4f::Identity();
    configured_initial_guess_ = Eigen::Matrix4f::Identity();
    best_transformation_ = Eigen::Matrix4f::Identity();
    best_fitness_score_ = std::numeric_limits<double>::max();
    best_robust_score_ = std::numeric_limits<double>::max();
    has_initialized_guess_ = false;
    consecutive_rejections_ = 0;
    in_parent_cloud_ = nullptr;
    in_parent_filtered_cloud_ = nullptr;
    in_child_cloud_ = nullptr;
    in_child_filtered_cloud_ = nullptr;

    InitializeROSIo();

    RCLCPP_INFO(this->get_logger(), "Ready. Waiting for data...");
}

void ROSMultiLidarCalibratorApp::PublishCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header.frame_id = parent_frame_;
    cloud_msg.header.stamp = this->now();
    calibrated_cloud_publisher_->publish(cloud_msg);
}

void ROSMultiLidarCalibratorApp::MatrixToTransform(const Eigen::Matrix4f& matrix,
                                                    geometry_msgs::msg::TransformStamped& transform_stamped)
{
    transform_stamped.transform.translation.x = static_cast<double>(matrix(0, 3));
    transform_stamped.transform.translation.y = static_cast<double>(matrix(1, 3));
    transform_stamped.transform.translation.z = static_cast<double>(matrix(2, 3));

    Eigen::Matrix3f rotation_matrix = matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation_matrix);

    transform_stamped.transform.rotation.x = static_cast<double>(quat.x());
    transform_stamped.transform.rotation.y = static_cast<double>(quat.y());
    transform_stamped.transform.rotation.z = static_cast<double>(quat.z());
    transform_stamped.transform.rotation.w = static_cast<double>(quat.w());
}

Eigen::Matrix4f ROSMultiLidarCalibratorApp::BuildConfiguredInitialGuess() const
{
    const auto map_it = transfer_map_.find(points_child_topic_str_);
    if (map_it == transfer_map_.end() || map_it->second.size() != 6) {
        RCLCPP_WARN(this->get_logger(),
                    "No valid initial guess found for topic '%s'. Falling back to identity.",
                    points_child_topic_str_.c_str());
        return Eigen::Matrix4f::Identity();
    }

    const std::vector<double>& t = map_it->second;
    Eigen::Translation3f init_translation(t[0], t[1], t[2]);
    Eigen::AngleAxisf init_rotation_x(t[5], Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(t[4], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(t[3], Eigen::Vector3f::UnitZ());

    return (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
}

std::vector<double> ROSMultiLidarCalibratorApp::GetNdtResolutionLevels() const
{
    std::vector<double> levels;

    if (use_multi_resolution_) {
        if (!ndt_resolution_levels_.empty()) {
            levels = ndt_resolution_levels_;
        } else {
            levels = {ndt_resolution_ * 4.0, ndt_resolution_ * 2.0, ndt_resolution_};
        }
    } else {
        levels = {ndt_resolution_};
    }

    levels.erase(std::remove_if(levels.begin(), levels.end(), [](double value) {
        return value <= 0.0;
    }), levels.end());

    std::sort(levels.begin(), levels.end(), std::greater<double>());
    levels.erase(std::unique(levels.begin(), levels.end(), [](double lhs, double rhs) {
        return std::fabs(lhs - rhs) < 1e-6;
    }), levels.end());

    if (levels.empty()) {
        levels.push_back(ndt_resolution_ > 0.0 ? ndt_resolution_ : 1.0);
    }

    return levels;
}

double ROSMultiLidarCalibratorApp::ComputeInlierRatio(const pcl::PointCloud<PointT>::ConstPtr& source_cloud,
                                                       const pcl::PointCloud<PointT>::ConstPtr& target_cloud,
                                                       double max_correspondence_distance) const
{
    if (source_cloud == nullptr || target_cloud == nullptr || source_cloud->empty() || target_cloud->empty()) {
        return 0.0;
    }

    const double max_distance_square = max_correspondence_distance * max_correspondence_distance;
    pcl::KdTreeFLANN<PointT> target_tree;
    target_tree.setInputCloud(target_cloud);

    int valid_points = 0;
    int inlier_points = 0;
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_sq_dist(1);

    for (const PointT& point : source_cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        ++valid_points;
        if (target_tree.nearestKSearch(point, 1, nearest_index, nearest_sq_dist) > 0
            && nearest_sq_dist[0] <= max_distance_square) {
            ++inlier_points;
        }
    }

    if (valid_points == 0) {
        return 0.0;
    }

    return static_cast<double>(inlier_points) / static_cast<double>(valid_points);
}

bool ROSMultiLidarCalibratorApp::IsTransformJumpAcceptable(const Eigen::Matrix4f& previous_transform,
                                                           const Eigen::Matrix4f& candidate_transform,
                                                           double* translation_jump,
                                                           double* rotation_jump_deg) const
{
    const Eigen::Vector3f prev_t = previous_transform.block<3, 1>(0, 3);
    const Eigen::Vector3f curr_t = candidate_transform.block<3, 1>(0, 3);
    const double translation_delta = static_cast<double>((curr_t - prev_t).norm());

    const Eigen::Matrix3f prev_r = previous_transform.block<3, 3>(0, 0);
    const Eigen::Matrix3f curr_r = candidate_transform.block<3, 3>(0, 0);
    const Eigen::Matrix3f delta_r = prev_r.transpose() * curr_r;

    double cos_theta = static_cast<double>((delta_r.trace() - 1.0f) * 0.5f);
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    constexpr double kRadToDeg = 57.295779513082320876;
    const double rotation_delta_deg = std::acos(cos_theta) * kRadToDeg;

    if (translation_jump != nullptr) {
        *translation_jump = translation_delta;
    }

    if (rotation_jump_deg != nullptr) {
        *rotation_jump_deg = rotation_delta_deg;
    }

    return translation_delta <= max_translation_jump_ && rotation_delta_deg <= max_rotation_jump_deg_;
}

double ROSMultiLidarCalibratorApp::ComputeRobustScore(double fitness_score,
                                                       double inlier_ratio,
                                                       double transform_probability) const
{
    const double safe_inlier_ratio = std::max(inlier_ratio, 1e-3);
    const double safe_probability = std::max(transform_probability, 1e-6);
    return fitness_score / (safe_inlier_ratio * safe_probability);
}

void ROSMultiLidarCalibratorApp::PerformNdtOptimize()
{
    if (in_parent_cloud_ == nullptr || in_child_cloud_ == nullptr
        || in_parent_filtered_cloud_ == nullptr || in_child_filtered_cloud_ == nullptr) {
        return;
    }

    if (!has_initialized_guess_) {
        configured_initial_guess_ = BuildConfiguredInitialGuess();
        current_guess_ = configured_initial_guess_;
        has_initialized_guess_ = true;
    }

    const Eigen::Matrix4f initial_guess = current_guess_;
    Eigen::Matrix4f candidate_transform = initial_guess;

    const pcl::PointCloud<PointT>::ConstPtr target_cloud =
        (in_parent_filtered_cloud_ != nullptr && !in_parent_filtered_cloud_->empty())
            ? in_parent_filtered_cloud_
            : in_parent_cloud_;

    const std::vector<double> resolution_levels = GetNdtResolutionLevels();
    pcl::PointCloud<PointT>::Ptr aligned_filtered_cloud(new pcl::PointCloud<PointT>);

    bool converged = false;
    double current_fitness_score = std::numeric_limits<double>::max();
    double current_probability = 0.0;

    for (double resolution : resolution_levels) {
        pcl::NormalDistributionsTransform<PointT, PointT> ndt;

        ndt.setTransformationEpsilon(ndt_epsilon_);
        ndt.setStepSize(ndt_step_size_);
        ndt.setResolution(resolution);
        ndt.setMaximumIterations(ndt_iterations_);
        ndt.setInputSource(in_child_filtered_cloud_);
        ndt.setInputTarget(target_cloud);

        pcl::PointCloud<PointT>::Ptr stage_output_cloud(new pcl::PointCloud<PointT>);
        ndt.align(*stage_output_cloud, candidate_transform);

        candidate_transform = ndt.getFinalTransformation();
        converged = ndt.hasConverged();
        current_fitness_score = ndt.getFitnessScore(max_correspondence_distance_);
        current_probability = ndt.getTransformationProbability();
        aligned_filtered_cloud = stage_output_cloud;
    }

    if (aligned_filtered_cloud == nullptr || aligned_filtered_cloud->empty()) {
        aligned_filtered_cloud.reset(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*in_child_filtered_cloud_, *aligned_filtered_cloud, candidate_transform);
    }

    const double inlier_ratio = ComputeInlierRatio(aligned_filtered_cloud, target_cloud, max_correspondence_distance_);
    const double robust_score = ComputeRobustScore(current_fitness_score, inlier_ratio, current_probability);

    double translation_jump = 0.0;
    double rotation_jump_deg = 0.0;
    const bool jump_ok = IsTransformJumpAcceptable(initial_guess, candidate_transform,
                                                   &translation_jump, &rotation_jump_deg);

    const bool score_ok = std::isfinite(current_fitness_score) && current_fitness_score <= max_fitness_score_;
    const bool inlier_ok = inlier_ratio >= min_inlier_ratio_;
    const bool probability_ok = current_probability >= min_transform_probability_;
    const bool accepted = converged && score_ok && inlier_ok && probability_ok && jump_ok;

    if (accepted) {
        current_guess_ = candidate_transform;
        consecutive_rejections_ = 0;
    } else {
        ++consecutive_rejections_;

        RCLCPP_WARN(this->get_logger(),
                    "NDT result rejected: converged=%d fitness=%.4f inlier=%.3f prob=%.4f jump_t=%.3f jump_r=%.2fdeg",
                    converged, current_fitness_score, inlier_ratio, current_probability,
                    translation_jump, rotation_jump_deg);

        if (consecutive_rejections_ >= max_rejections_before_reset_) {
            current_guess_ = configured_initial_guess_;
            consecutive_rejections_ = 0;
            RCLCPP_WARN(this->get_logger(),
                        "Too many rejected alignments. Reset current guess to configured initial transform.");
        }
    }

    // 只有当候选位姿通过门控，且鲁棒得分更好时才更新最优结果
    if (accepted && robust_score < best_robust_score_) {
        best_robust_score_ = robust_score;
        best_fitness_score_ = current_fitness_score;
        best_transformation_ = candidate_transform;

        std::cout << "========== Best Registration Found ==========" << std::endl;
        std::cout << "Normal Distributions Transform converged:" << converged
                  << " score: " << current_fitness_score
                  << " inlier_ratio: " << inlier_ratio
                  << " prob: " << current_probability
                  << " robust_score: " << robust_score
                  << std::endl;
        std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;

        Eigen::Matrix3f rotation_matrix = best_transformation_.block(0, 0, 3, 3);
        Eigen::Vector3f translation_vector = best_transformation_.block(0, 3, 3, 1);

        std::cout << "This transformation can be replicated using:" << std::endl;
        std::cout << "ros2 run tf2_ros static_transform_publisher --x " << translation_vector(0)
                  << " --y " << translation_vector(1)
                  << " --z " << translation_vector(2)
                  << " --yaw " << rotation_matrix.eulerAngles(2, 1, 0)(0)
                  << " --pitch " << rotation_matrix.eulerAngles(2, 1, 0)(1)
                  << " --roll " << rotation_matrix.eulerAngles(2, 1, 0)(2)
                  << " --frame-id " << parent_frame_
                  << " --child-frame-id " << child_frame_ << std::endl;

        std::cout << "Corresponding transformation matrix:" << std::endl
                  << std::endl << best_transformation_ << std::endl;
        std::cout << "==============================================" << std::endl << std::endl;
    }

    // Transforming unfiltered, input cloud using found transform.
    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*in_child_cloud_, *output_cloud, current_guess_);

    PublishCloud(output_cloud);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = parent_frame_;
    transform_stamped.child_frame_id = child_frame_;
    MatrixToTransform(current_guess_, transform_stamped);
    tf_broadcaster_->sendTransform(transform_stamped);
}

void ROSMultiLidarCalibratorApp::PointsCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr in_parent_cloud_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr in_child_cloud_msg)
{
    pcl::PointCloud<PointT>::Ptr parent_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr child_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr parent_filtered_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr child_filtered_cloud(new pcl::PointCloud<PointT>);

    pcl::fromROSMsg(*in_parent_cloud_msg, *parent_cloud);
    pcl::fromROSMsg(*in_child_cloud_msg, *child_cloud);

    parent_frame_ = in_parent_cloud_msg->header.frame_id;
    child_frame_ = in_child_cloud_msg->header.frame_id;

    DownsampleCloud(parent_cloud, parent_filtered_cloud, voxel_size_);
    DownsampleCloud(child_cloud, child_filtered_cloud, voxel_size_);
    in_parent_cloud_ = parent_cloud;
    in_parent_filtered_cloud_ = parent_filtered_cloud;
    in_child_cloud_ = child_cloud;
    in_child_filtered_cloud_ = child_filtered_cloud;
}

void ROSMultiLidarCalibratorApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                                                  pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
                                                  double in_leaf_size)
{
    pcl::VoxelGrid<PointT> voxelized;
    voxelized.setInputCloud(in_cloud_ptr);
    voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    voxelized.filter(*out_cloud_ptr);
}

void ROSMultiLidarCalibratorApp::InitializeROSIo()
{
    // Declare and get parameters
    this->declare_parameter<std::string>("init_params_file_path", "");
    this->declare_parameter<std::string>("points_parent_src", "points_raw");
    this->declare_parameter<std::string>("points_child_src", "points_raw");
    this->declare_parameter<double>("voxel_size", 0.1);
    this->declare_parameter<double>("ndt_epsilon", 0.01);
    this->declare_parameter<double>("ndt_step_size", 0.1);
    this->declare_parameter<double>("ndt_resolution", 1.0);
    this->declare_parameter<bool>("use_multi_resolution", true);
    this->declare_parameter<std::vector<double>>("ndt_resolution_levels", std::vector<double>{});
    this->declare_parameter<int>("ndt_iterations", 400);
    this->declare_parameter<double>("max_fitness_score", 1.0);
    this->declare_parameter<double>("max_correspondence_distance", 1.0);
    this->declare_parameter<double>("min_inlier_ratio", 0.25);
    this->declare_parameter<double>("min_transform_probability", 0.02);
    this->declare_parameter<double>("max_translation_jump", 0.8);
    this->declare_parameter<double>("max_rotation_jump_deg", 12.0);
    this->declare_parameter<int>("max_rejections_before_reset", 20);

    std::string init_file_path = this->get_parameter("init_params_file_path").as_string();
    points_parent_topic_str_ = this->get_parameter("points_parent_src").as_string();
    points_child_topic_str_ = this->get_parameter("points_child_src").as_string();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    ndt_epsilon_ = this->get_parameter("ndt_epsilon").as_double();
    ndt_step_size_ = this->get_parameter("ndt_step_size").as_double();
    ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();
    use_multi_resolution_ = this->get_parameter("use_multi_resolution").as_bool();
    ndt_resolution_levels_ = this->get_parameter("ndt_resolution_levels").as_double_array();
    ndt_iterations_ = this->get_parameter("ndt_iterations").as_int();
    max_fitness_score_ = this->get_parameter("max_fitness_score").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
    min_inlier_ratio_ = this->get_parameter("min_inlier_ratio").as_double();
    min_transform_probability_ = this->get_parameter("min_transform_probability").as_double();
    max_translation_jump_ = this->get_parameter("max_translation_jump").as_double();
    max_rotation_jump_deg_ = this->get_parameter("max_rotation_jump_deg").as_double();
    max_rejections_before_reset_ = this->get_parameter("max_rejections_before_reset").as_int();

    RCLCPP_INFO(this->get_logger(), "points_parent_src: %s", points_parent_topic_str_.c_str());
    RCLCPP_INFO(this->get_logger(), "points_child_src: %s", points_child_topic_str_.c_str());
    RCLCPP_INFO(this->get_logger(), "voxel_size: %.2f", voxel_size_);
    RCLCPP_INFO(this->get_logger(), "ndt_epsilon: %.2f", ndt_epsilon_);
    RCLCPP_INFO(this->get_logger(), "ndt_step_size: %.2f", ndt_step_size_);
    RCLCPP_INFO(this->get_logger(), "ndt_resolution: %.2f", ndt_resolution_);
    RCLCPP_INFO(this->get_logger(), "use_multi_resolution: %s", use_multi_resolution_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "ndt_iterations: %d", ndt_iterations_);
    RCLCPP_INFO(this->get_logger(), "max_fitness_score: %.2f", max_fitness_score_);
    RCLCPP_INFO(this->get_logger(), "max_correspondence_distance: %.2f", max_correspondence_distance_);
    RCLCPP_INFO(this->get_logger(), "min_inlier_ratio: %.2f", min_inlier_ratio_);
    RCLCPP_INFO(this->get_logger(), "min_transform_probability: %.4f", min_transform_probability_);
    RCLCPP_INFO(this->get_logger(), "max_translation_jump: %.2f", max_translation_jump_);
    RCLCPP_INFO(this->get_logger(), "max_rotation_jump_deg: %.2f", max_rotation_jump_deg_);
    RCLCPP_INFO(this->get_logger(), "max_rejections_before_reset: %d", max_rejections_before_reset_);

    // Read initial parameters from file
    if (!init_file_path.empty()) {
        std::ifstream ifs(init_file_path);
        if (ifs.is_open()) {
            ifs >> child_topic_num_;

            for (int j = 0; j < child_topic_num_; ++j) {
                std::string child_name;
                ifs >> child_name;
                std::vector<double> tmp_transfer;
                for (int k = 0; k < 6; ++k) {
                    double tmp_xyzypr;
                    ifs >> tmp_xyzypr;
                    tmp_transfer.push_back(tmp_xyzypr);
                }
                transfer_map_.insert(std::pair<std::string, std::vector<double>>(child_name, tmp_transfer));
            }
            ifs.close();
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not open init params file: %s", init_file_path.c_str());
        }
    }

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Generate subscribers with message_filters
    cloud_parent_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, points_parent_topic_str_);
    RCLCPP_INFO(this->get_logger(), "Subscribing to... %s", points_parent_topic_str_.c_str());

    cloud_child_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, points_child_topic_str_);
    RCLCPP_INFO(this->get_logger(), "Subscribing to... %s", points_child_topic_str_.c_str());

    // Publisher
    std::string calibrated_points_topic_str = "/points_calibrated";
    calibrated_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        calibrated_points_topic_str, 1);
    RCLCPP_INFO(this->get_logger(), "Publishing PointCloud to... %s", calibrated_points_topic_str.c_str());

    // Synchronizer
    cloud_synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicyT>>(
        SyncPolicyT(100), *cloud_parent_subscriber_, *cloud_child_subscriber_);
    cloud_synchronizer_->registerCallback(
        std::bind(&ROSMultiLidarCalibratorApp::PointsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Timer for NDT optimization (10 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ROSMultiLidarCalibratorApp::TimerCallback, this));
}

void ROSMultiLidarCalibratorApp::TimerCallback()
{
    PerformNdtOptimize();
}
