#include "multi_lidar_calibrator.h"

ROSMultiLidarCalibratorApp::ROSMultiLidarCalibratorApp()
    : Node(__APP_NAME__)
{
    current_guess_ = Eigen::Matrix4f::Identity();
    in_parent_cloud_ = nullptr;
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

void ROSMultiLidarCalibratorApp::PerformNdtOptimize()
{
    if (in_parent_cloud_ == nullptr || in_child_cloud_ == nullptr) {
        return;
    }

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;

    ndt.setTransformationEpsilon(ndt_epsilon_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setResolution(ndt_resolution_);

    ndt.setMaximumIterations(ndt_iterations_);

    ndt.setInputSource(in_child_filtered_cloud_);
    ndt.setInputTarget(in_parent_cloud_);

    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

    if (current_guess_ == Eigen::Matrix4f::Identity()) {
        Eigen::Translation3f init_translation(
            transfer_map_[points_child_topic_str_][0],
            transfer_map_[points_child_topic_str_][1],
            transfer_map_[points_child_topic_str_][2]);
        Eigen::AngleAxisf init_rotation_x(transfer_map_[points_child_topic_str_][5], Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(transfer_map_[points_child_topic_str_][4], Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(transfer_map_[points_child_topic_str_][3], Eigen::Vector3f::UnitZ());

        Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

        current_guess_ = init_guess;
    }

    ndt.align(*output_cloud, current_guess_);

    std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore() << " prob:" << ndt.getTransformationProbability() << std::endl;
    std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud(*in_child_cloud_, *output_cloud, ndt.getFinalTransformation());

    current_guess_ = ndt.getFinalTransformation();

    Eigen::Matrix3f rotation_matrix = current_guess_.block(0, 0, 3, 3);
    Eigen::Vector3f translation_vector = current_guess_.block(0, 3, 3, 1);

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
              << std::endl << current_guess_ << std::endl << std::endl;

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
    pcl::PointCloud<PointT>::Ptr child_filtered_cloud(new pcl::PointCloud<PointT>);

    pcl::fromROSMsg(*in_parent_cloud_msg, *parent_cloud);
    pcl::fromROSMsg(*in_child_cloud_msg, *child_cloud);

    parent_frame_ = in_parent_cloud_msg->header.frame_id;
    child_frame_ = in_child_cloud_msg->header.frame_id;

    DownsampleCloud(child_cloud, child_filtered_cloud, voxel_size_);
    in_parent_cloud_ = parent_cloud;
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
    this->declare_parameter<int>("ndt_iterations", 400);

    std::string init_file_path = this->get_parameter("init_params_file_path").as_string();
    points_parent_topic_str_ = this->get_parameter("points_parent_src").as_string();
    points_child_topic_str_ = this->get_parameter("points_child_src").as_string();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    ndt_epsilon_ = this->get_parameter("ndt_epsilon").as_double();
    ndt_step_size_ = this->get_parameter("ndt_step_size").as_double();
    ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();
    ndt_iterations_ = this->get_parameter("ndt_iterations").as_int();

    RCLCPP_INFO(this->get_logger(), "points_parent_src: %s", points_parent_topic_str_.c_str());
    RCLCPP_INFO(this->get_logger(), "points_child_src: %s", points_child_topic_str_.c_str());
    RCLCPP_INFO(this->get_logger(), "voxel_size: %.2f", voxel_size_);
    RCLCPP_INFO(this->get_logger(), "ndt_epsilon: %.2f", ndt_epsilon_);
    RCLCPP_INFO(this->get_logger(), "ndt_step_size: %.2f", ndt_step_size_);
    RCLCPP_INFO(this->get_logger(), "ndt_resolution: %.2f", ndt_resolution_);
    RCLCPP_INFO(this->get_logger(), "ndt_iterations: %d", ndt_iterations_);

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
