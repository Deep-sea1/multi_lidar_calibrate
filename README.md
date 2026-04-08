# Multi-LiDAR Calibrator - 多激光雷达外参自动标定工具
基于 ROS2 Humble 的多激光雷达外参自动标定程序，使用 NDT（Normal Distributions Transform）算法实现快速、高精度的激光雷达外参标定。
## 功能介绍
本工具用于计算多个激光雷达之间的外参变换矩阵（extrinsic calibration），主要特点：
- 基于 NDT 配准算法，自动计算子激光雷达相对于父激光雷达的 6DoF 变换（x, y, z, roll, pitch, yaw）
- 支持设置初始位姿估计，加速收敛
- 实时发布标定后的点云和 TF 变换
