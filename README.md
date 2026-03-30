# Multi-LiDAR Calibrator - 多激光雷达外参自动标定工具

基于 ROS2 Humble 的多激光雷达外参自动标定程序，使用 NDT（Normal Distributions Transform）算法实现快速、高精度的激光雷达外参标定。

## 目录

- [功能介绍](#功能介绍)
- [环境要求](#环境要求)
- [安装步骤](#安装步骤)
- [使用方法](#使用方法)
- [参数配置](#参数配置)
- [初始参数文件格式](#初始参数文件格式)
- [输出结果](#输出结果)
- [使用示例](#使用示例)
- [常见问题](#常见问题)

## 功能介绍

本工具用于计算多个激光雷达之间的外参变换矩阵（extrinsic calibration），主要特点：

- 基于 NDT 配准算法，自动计算子激光雷达相对于父激光雷达的 6DoF 变换（x, y, z, roll, pitch, yaw）
- 支持设置初始位姿估计，加速收敛
- 实时发布标定后的点云和 TF 变换
- 自动输出可直接使用的 `static_transform_publisher` 命令

## 环境要求

- **操作系统**: Ubuntu 22.04
- **ROS 版本**: ROS2 Humble

### 安装依赖

```bash
sudo apt update
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-message-filters \
    libpcl-dev
```

## 安装步骤

### 1. 创建工作空间

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. 克隆

将 `multi_lidar_calibrator` 包复制到 `src` 目录下。

### 3. 编译

```bash
cd ~/ros2_ws
colcon build --packages-select multi_lidar_calibrator
source install/setup.bash
```

## 使用方法

### 基本使用

1. **启动激光雷达驱动**，确保两个激光雷达的点云话题正在发布

2. **运行标定节点**：

```bash
ros2 launch multi_lidar_calibrator multi_lidar_calibrator.launch.py \
    points_parent_src:=/your_parent_lidar_topic \
    points_child_src:=/your_child_lidar_topic
```

3. **观察输出**，等待 NDT 算法收敛，标定结果将在终端输出

### 使用 RViz 可视化

打开另一个终端，启动 RViz 查看标定效果：

```bash
rviz2
```

在 RViz 中添加以下显示项：
- **PointCloud2**: 订阅父激光雷达话题
- **PointCloud2**: 订阅 `/points_calibrated`（标定后的子激光雷达点云）
- **TF**: 显示坐标变换

## 参数配置

### Launch 文件参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `points_parent_src` | string | `/livox/lidar_192_168_1_139` | 父激光雷达点云话题（作为参考坐标系） |
| `points_child_src` | string | `/livox/lidar_192_168_1_118` | 子激光雷达点云话题（需要标定的激光雷达） |
| `init_params_file_path` | string | `cfg/child_topic_list` | 初始参数文件路径 |
| `voxel_size` | double | `0.1` | 体素滤波器大小（米），用于下采样 |
| `ndt_epsilon` | double | `0.1` | NDT 变换收敛阈值 |
| `ndt_step_size` | double | `0.1` | NDT 优化步长 |
| `ndt_resolution` | double | `0.5` | NDT 网格分辨率 |
| `ndt_iterations` | int | `100` | NDT 最大迭代次数 |

### 参数调优建议

- **voxel_size**:
  - 值越小，点云越密集，精度越高，但计算速度越慢
  - 建议范围：0.05 - 0.2 米

- **ndt_resolution**:
  - 值越小，精度越高，但可能陷入局部最优
  - 建议范围：0.3 - 1.0 米

- **ndt_iterations**:
  - 如果初始位姿估计较好，可以设置较小值（50-100）
  - 如果初始位姿偏差较大，建议设置较大值（200-400）

## 初始参数文件格式

初始参数文件用于提供初始位姿估计，可以加速 NDT 收敛并避免陷入局部最优。

### 文件格式

```
<子话题数量>
<子话题名称1> <x> <y> <z> <yaw> <pitch> <roll>
<子话题名称2> <x> <y> <z> <yaw> <pitch> <roll>
...
```

### 参数说明

- **x, y, z**: 平移量（米）
- **yaw, pitch, roll**: 旋转角度（弧度）

### 示例文件 (`cfg/child_topic_list`)

```
1
/livox/lidar_192_168_1_118 -0.09 -0.25 -0.25 -0.6536 0.54 1.4444
```

此示例表示：
- 共有 1 个子激光雷达
- 话题名称为 `/livox/lidar_192_168_1_118`
- 初始位姿估计：x=-0.09m, y=-0.25m, z=-0.25m, yaw=-0.6536rad, pitch=0.54rad, roll=1.4444rad


## 输出结果

### 终端输出

程序运行后，终端会输出类似以下信息：

```
Normal Distributions Transform converged:1 score: 0.123456 prob:0.987654
transformation from child_frame to parent_frame
This transformation can be replicated using:
ros2 run tf2_ros static_transform_publisher --x 0.1234 --y -0.5678 --z 0.0912 --yaw 0.1234 --pitch 0.0567 --roll 1.5708 --frame-id parent_frame --child-frame-id child_frame
Corresponding transformation matrix:
    0.999   0.012  -0.034   0.123
   -0.011   0.999   0.056  -0.567
    0.035  -0.055   0.998   0.091
    0.000   0.000   0.000   1.000
```

### 输出说明

- **converged**: 是否收敛（1=是，0=否）
- **score**: 配准得分，越小越好（理想情况 < 0.5）
- **prob**: 变换概率，越大越好
- **static_transform_publisher 命令**: 可直接复制使用，用于发布静态 TF

### 发布的话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/points_calibrated` | `sensor_msgs/PointCloud2` | 标定后的子激光雷达点云（已变换到父坐标系） |

### 发布的 TF

程序会实时发布从 `child_frame` 到 `parent_frame` 的 TF 变换。

## 使用示例

### 示例 1: 两个 Livox 激光雷达标定

```bash
# 终端 1: 启动 Livox 驱动
ros2 launch livox_ros2_driver rviz_MID360_launch.py

# 终端 2: 启动标定程序
ros2 launch multi_lidar_calibrator multi_lidar_calibrator.launch.py \
    points_parent_src:=/livox/lidar_192_168_1_139 \
    points_child_src:=/livox/lidar_192_168_1_118

# 终端 3: 启动 RViz 可视化
rviz2
```

## 标定流程

1. **准备阶段**
   - 确保所有激光雷达正常工作
   - 测量或估计初始位姿，填写配置文件
   - 将车辆/机器人停放在特征丰富的环境中（如有墙壁、柱子等）

2. **标定阶段**
   - 启动所有激光雷达驱动
   - 启动标定节点
   - 观察终端输出，等待 `converged: 1` 且 `score` 值稳定

3. **验证阶段**
   - 在 RViz 中查看 `/points_calibrated` 是否与父激光雷达点云对齐
   - 如果对齐效果不好，调整参数重新标定

4. **应用阶段**
   - 记录最终的变换矩阵或 `static_transform_publisher` 命令
   - 将标定结果集成到机器人系统中

## 常见问题

### Q1: NDT 不收敛怎么办？

**可能原因及解决方案**：
- 初始位姿估计偏差过大：提供更准确的初始位姿
- 环境特征不足：移动到特征更丰富的环境
- 参数设置不当：增大 `ndt_iterations`，调整 `ndt_resolution`

### Q2: Score 值很大怎么办？

**可能原因**：
- 两个激光雷达的重叠区域太小
- 点云时间同步不佳
- 存在动态物体干扰

**解决方案**：
- 调整激光雷达安装位置，增大重叠区域
- 使用时间同步功能
- 在静态环境中进行标定

## 算法原理

本工具使用 NDT（Normal Distributions Transform）算法进行点云配准：

1. **点云预处理**: 使用体素滤波对子激光雷达点云进行下采样
2. **NDT 配准**:
   - 将父点云划分为网格单元
   - 每个单元用正态分布表示
   - 优化子点云的位姿，最大化与父点云的匹配概率
3. **迭代优化**: 使用梯度下降方法迭代求解最优变换
4. **结果输出**: 输出最终的 4x4 变换矩阵

## 许可证

Apache-2.0 License

## 参考资料

- [NDT 算法论文](https://www.researchgate.net/publication/4045903_The_Normal_Distributions_Transform_A_New_Approach_to_Laser_Scan_Matching)
- [PCL NDT 文档](https://pointclouds.org/documentation/classpcl_1_1_normal_distributions_transform.html)
