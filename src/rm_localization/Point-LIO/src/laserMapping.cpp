#include <omp.h>
#include <mutex>
#include <cmath>
#include <thread>
#include <fstream>
#include <csignal>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "parameters.h"
#include "Estimator.h"


#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

// 地图边界更新的触发阈值系数
const float MOV_THRESHOLD = 1.5f;

// 多线程互斥锁与条件变量，用于协调雷达/IMU回调线程与主制图线程的数据同步
mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;

// 用于性能统计和日志记录的全局计数器
int feats_down_size = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;

int frame_ct = 0;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

// IMU预处理对象指针（负责点云去畸变与时间同步）
shared_ptr<ImuProcess> p_imu(new ImuProcess());
bool init_map = false, flg_first_scan = true;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());

// 耗时统计与调试日志数组
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

// 系统状态标志位
bool lidar_pushed = false, flg_reset = false, flg_exit = false;

// 存储需要从 ikD-Tree 局部地图中移除的边界框区域
vector<BoxPointType> cub_needrm;

// 数据缓冲区队列
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<double> time_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

// 当前帧经去畸变、降采样及转换后的各类点云指针
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());

// 降采样滤波器（体素网格滤波）
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

// 传感器数据测量组（对齐后的单帧LiDAR+期间对应的IMU数据）
MeasureGroup Measures;

sensor_msgs::msg::Imu imu_last, imu_next;
sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;
nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

// ROS2 日志记录器
auto logger = rclcpp::get_logger("laserMapping");

/**
 * @brief 系统中断信号（如 Ctrl+C）捕获处理函数
 */
void SigHandle(int sig) {
    flg_exit = true;
    RCLCPP_WARN(logger, "catch sig %d", sig);
    sig_buffer.notify_all();
}

/**
 * @brief 将当前的激光惯性里程计（LIO）状态打印写入日志文件
 */
inline void dump_lio_state_to_log(FILE *fp) {
    V3D rot_ang;
    // 根据系统使用的滤波器支线，提取当前的姿态旋转矩阵并转换为欧拉角
    if (!use_imu_as_input) {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    } else {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }

    // 格式化输出：时间戳、角度、位置、角速度、速度、加速度、IMU零偏以及重力向量
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    if (use_imu_as_input) {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2)); // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));    // Bias_g
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));    // Bias_a
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2)); // Bias_a
    } else {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2)); // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2)); // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));    // Bias_g
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));    // Bias_a
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
                kf_output.x_.gravity(2)); // Bias_a
    }
    fprintf(fp, "\r\n");
    fflush(fp);
}

/**
 * @brief 将雷达坐标系下的点转换到 IMU 坐标系下（利用外参）
 */
void pointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    // 如果开启了外参在线估计，使用状态向量中实时估计的旋转与平移；否则使用静态配置外参
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            p_body_imu = kf_output.x_.offset_R_L_I.normalized() * p_body_lidar + kf_output.x_.offset_T_L_I;
        } else {
            p_body_imu = kf_input.x_.offset_R_L_I.normalized() * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    } else {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

int points_cache_size = 0;

/**
 * @brief 收集并统计 ikD-Tree 内部被废弃或删除的点云缓存大小（主要用于调试）
 */
void points_cache_collect() // seems for debug
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

/**
 * @brief 动态滑窗动态管理：根据当前雷达所在视场位置，移动或切分局部地图立方体边界
 */
void lasermap_fov_segment() {
    cub_needrm.shrink_to_fit();

    V3D pos_LiD;
    // 计算当前雷达在世界坐标系下的绝对位置
    if (use_imu_as_input) {
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot.normalized() * Lidar_T_wrt_IMU;
    } else {
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot.normalized() * Lidar_T_wrt_IMU;
    }

    // 初始化局部地图包围盒（以当前雷达为中心，边长为 cube_len）
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    // 检查雷达是否距离当前地图立方体六个边界面过近
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        // 若与任一边缘距离小于触发阈值，则需要搬移地图滑窗
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move) return;

    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    // 计算滑窗沿轴向应当搬移的绝对物理距离
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                         double(DET_RANGE * (MOV_THRESHOLD - 1)));

    // 判断搬移方向，计算腾出来的待擦除区域盒子（tmp_boxpoints）并存入 cub_needrm
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    // 增量式从 ikD-Tree 树结构中彻底物理删除老旧视场网格对应的整个空间点云盒子
    if (cub_needrm.size() > 0) int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

/**
 * @brief 标准 PointCloud2 激光点云接收回调函数（针对常见机械旋转雷达等）
 */
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    // 时间戳异常倒流检测
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar) {
        RCLCPP_ERROR(logger, "lidar loop back, clear buffer");
        // lidar_buffer.shrink_to_fit();

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = msg->header.stamp.sec;

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    double time_div = get_time_sec(msg->header.stamp);

    // 调用预处理模块进行格式解析与畸变预筛
    p_pre->process(msg, ptr);

    // 如果配置了切帧（将大帧切细以提升时效性）
    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            // 按内部点的微秒曲率差值检测断帧间隔
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div >
                cut_frame_time_interval) {
                if (ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty()) {
            lidar_buffer.push_back(ptr_div);
            time_buffer.push_back(time_div);
        }
    } else if (con_frame) { // 如果配置了组帧（将多个包融合成大帧）
        if (frame_ct == 0) {
            time_con = last_timestamp_lidar;
        }
        if (frame_ct < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct++;
        } else {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    } else { // 正常直通模式：直接压入帧缓冲区
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

/**
 * @brief 览沃（Livox）定制格式点云数据接收回调函数（针对定制自定义结构体）
 */
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar) {
        RCLCPP_ERROR(logger, "lidar loop back, clear buffer");

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = get_time_sec(msg->header.stamp);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    double time_div = get_time_sec(msg->header.stamp);
    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div >
                cut_frame_time_interval) {
                if (ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty()) {
            lidar_buffer.push_back(ptr_div);
            time_buffer.push_back(time_div);
        }
    } else if (con_frame) {
        if (frame_ct == 0) {
            time_con = last_timestamp_lidar;
        }
        if (frame_ct < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct++;
        } else {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    } else {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

/**
 * @brief 高频 IMU 数据接收回调函数
 */
void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in) {
    publish_count++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    // 根据硬件延迟参数补偿修正 IMU 时间戳
    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_lag_imu_to_lidar);
    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {
        RCLCPP_ERROR(logger, "imu loop back, clear deque");
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    // 装入待同步 IMU 双端队列
    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

/**
 * @brief 时间包同步器：核心对齐函数，从数据队列中提取出对应当前点云帧时间周期内的全部高频 IMU 数据
 */
bool sync_packages(MeasureGroup &meas) {
    // 纯雷达模式（无IMU）逻辑
    if (!imu_en) {
        if (!lidar_buffer.empty()) {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            if (meas.lidar->points.size() < 1) {
                cout << "lose lidar" << std::endl;
                return false;
            }
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points) {
                if (pt.curvature > end_time) {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
            return true;
        }
        return false;
    }

    // 缺少必要传感器源时挂起
    if (lidar_buffer.empty() || imu_deque.empty()) {
        return false;
    }

    /*** 推送单帧激光点云数据，并计算当前帧末尾边界绝对时刻 ***/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        if (meas.lidar->points.size() < 1) {
            cout << "lose lidar" << endl;
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt: meas.lidar->points) {
            if (pt.curvature > end_time) {
                end_time = pt.curvature;
            }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);

        meas.lidar_last_time = lidar_end_time;
        lidar_pushed = true;
    }

    // 若当前缓存的 IMU 数据最远时刻还没覆盖到雷达帧的结束时刻，说明 IMU 数据不充足，需要继续等待
    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }

    /*** 抽取这一段被完全包围的 IMU 数据流，组成联合测量帧并从缓冲区剔除 ***/
    if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
            imu_time = get_time_sec(imu_deque.front()->header.stamp);
            if (imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    } else if (!init_map) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        meas.imu.emplace_back(imu_last_ptr);

        while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
            imu_time = get_time_sec(imu_deque.front()->header.stamp);
            if (imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true; // 时间同步大功告成
}

int process_increments = 0;

/**
 * @brief 增量式地图更新：将更新完位姿后的当前帧雷达点以最小空间网格进行空间降采样并入 ikD-Tree 树结构
 */
void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++) {
        // 利用 ikD-Tree 的内部多近邻查找算法寻找当前的最近点
        if (!Nearest_Points[i].empty()) {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            PointType downsample_result, mid_point;
            // 计算当前点所处的体素三维网格中心点坐标 (以 filter_size_map_min 为步长)
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            /* 若最近邻点距离当前中心网格极其遥远，无需降采样验证，直接归类为待安全添加点 */
            if (fabs(points_near[0].x - mid_point.x) > 1.732 * filter_size_map_min ||
                fabs(points_near[0].y - mid_point.y) > 1.732 * filter_size_map_min ||
                fabs(points_near[0].z - mid_point.z) > 1.732 * filter_size_map_min) {
                PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
                continue;
            }
            /* 检测是否已经有任何其他历史点已经落在此降采样空间包围网格内 */
            float dist = calc_dist<float>(feats_down_world->points[i], mid_point);
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min &&
                    fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min &&
                    fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) {
                    need_add = false; // 已有落入该网格的历史点，放弃重复插入
                    break;
                }
            }
            if (need_add) PointToAdd.emplace_back(feats_down_world->points[i]);
        } else {
            PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
        }
    }
    // 调用 ikD-Tree 原生的高效多线程增量添加接口，更新局部地图树模型
    int add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
}

/**
 * @brief 在第一帧初始化 ikD-Tree 构建时，展平树形结构并发布初始局部地图点云
 */
void publish_init_kdtree(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes) {
    int size_init_ikdtree = ikdtree.size();
    PointCloudXYZI::Ptr laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    PointVector().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);

    laserCloudInit->points = ikdtree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

/**
 * @brief 发布当前帧变换到世界坐标系（`odom`）下的全量注册点云，并控制 PCD 文件本地固化写盘
 */
void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes) {
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** 自动分段保存全局 PCD 地图文件 ****************/
    if (pcd_save_en) {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        // 达到预设间隔帧数阈值，自动写盘固化并清理内存
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

/**
 * @brief 发布当前去畸变后并且对齐到 IMU 刚体中心（`body`）坐标系下的实时点云
 */
void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull_body) {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "base_footprint";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

/**
 * @brief 泛型提取滤波器状态向量中的位置与旋转四元数信息，快速填充姿态消息结构体
 */
template<typename T>
void set_posestamp(T &out) {
    if (!use_imu_as_input) {
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        out.orientation.x = kf_output.x_.rot.coeffs()[0];
        out.orientation.y = kf_output.x_.rot.coeffs()[1];
        out.orientation.z = kf_output.x_.rot.coeffs()[2];
        out.orientation.w = kf_output.x_.rot.coeffs()[3];
    } else {
        out.position.x = kf_input.x_.pos(0);
        out.position.y = kf_input.x_.pos(1);
        out.position.z = kf_input.x_.pos(2);
        out.orientation.x = kf_input.x_.rot.coeffs()[0];
        out.orientation.y = kf_input.x_.rot.coeffs()[1];
        out.orientation.z = kf_input.x_.rot.coeffs()[2];
        out.orientation.w = kf_input.x_.rot.coeffs()[3];
    }
}

/**
 * @brief 实时打包并向外发布最终映射的 Odometry 里程计消息，并发送 TF2 坐标变换树
 */
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped,
                      std::shared_ptr<tf2_ros::TransformBroadcaster> &tf_br) {
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = "base_footprint";
    if (publish_odometry_without_downsample) {
        odomAftMapped.header.stamp = get_ros_time(time_current);
    } else {
        odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);

    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transform.header.stamp = odomAftMapped.header.stamp;
    tf_br->sendTransform(transform); // 广播世界坐标系到地图匹配坐标系的位姿转换
}

/**
 * @brief 在 RViz2 中发布并绘制完整的历史移动轨迹路径（Path）
 */
void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath) {
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = "odom";
    static int jjj = 0;
    jjj++;
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath->publish(path);
    }
}

/**
 * @brief 系统的核心 Main 主函数：控制整个制图节点生命周期内的参数加载、订阅绑定、IEKF滤波器迭代更新调度
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("laserMapping");
    readParameters(nh); // 从 ROS2 YAML 配置文件解析各类外参、模式及噪声参数
    cout << "lidar_type: " << lidar_type << endl;

    path.header.stamp = get_ros_time(lidar_end_time);
    path.header.frame_id = "odom";

    /*** 各种计数与全局平均耗时统计标量初始化 ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;
    std::time_t startTime, endTime;

    /*** 几何视场弧度计算与各类过滤器降采样叶子参数配置 ***/
    double FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    double HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    // 固化初值外参矩阵到 Eigen 变量
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        } else {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    // 分别初始化两套不同状态分支对应的非线性卡尔曼迭代状态转移系统方程（绑定回调函数指针）
    kf_input.init_dyn_share_modified(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_2h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);

    // 初始化卡尔曼滤波协方差矩阵 P（给位置、旋转、速度、Bias 分配初始不确定度度量值）
    Eigen::Matrix<double, 24, 24> P_init = MD(24, 24)::Identity() * 0.01;
    P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
    P_init.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    kf_input.change_P(P_init);
    Eigen::Matrix<double, 30, 30> P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init_output.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
    kf_input.change_P(P_init);
    kf_output.change_P(P_init_output);

    // 初始化过程噪声协方差矩阵 Q
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();

    /*** 打开本地调试日志记录文件流 ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_out, fout_imu_pbp;
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);
    if (fout_out && fout_imu_pbp)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

    /*** ROS2 通信节点订阅者与发布者接口注册激活 ***/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    // 依雷达硬件底层格式类型，条件触发绑定不同的特定回调函数
    if (p_pre->lidar_type == AVIA) {
        sub_pcl_livox_ = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
    } else {
        sub_pcl_pc_ = nh->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(),
                                                                             standard_pcl_cbk);
    }
    auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000, imu_cbk);
    auto pubLaserCloudFullRes = nh->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud_registered", 100000);
    auto pubLaserCloudFullRes_body = nh->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud_registered_body", 100000);
    auto pubLaserCloudEffect = nh->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud_effected", 100000);
    auto pubLaserCloudMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/Laser_map", 100000);
    auto pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>
            ("/aft_mapped_to_init", 100000);
    auto pubPath = nh->create_publisher<nav_msgs::msg::Path>
            ("/path", 100000);
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle); // 绑定中断退出句柄
    rclcpp::Rate rate(5000);  // 超高频自循环控制主线程速率
    while (rclcpp::ok()) {
        if (flg_exit) break;

        // 调度单线程执行器处理当前各个回调事件（雷达/IMU数据进队列）
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(nh);
        executor.spin_some();

        // 尝试进行时间戳对齐，提取当前联合同步帧测量数据
        if (sync_packages(Measures)) {
            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                cout << "first lidar time" << first_lidar_time << endl;
            }

            if (flg_reset) {
                RCLCPP_WARN(logger, "reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }
            double t0, t1, t2, t3, t4, t5, match_start, solve_start;
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();

            // 调用 IMU 预处理核心算法：融合高频 IMU 数据对当前点云进行运动畸变去除 (feats_undistort)
            p_imu->Process(Measures, feats_undistort);

            if (feats_undistort->empty() || feats_undistort == nullptr) {
                continue;
            }

            // 地图初始重力对齐初始化：如果系统刚启动，提取首段时间内的加速度平均向量估计地平线方向
            if (imu_en) {
                if (!p_imu->gravity_align_) {
                    while (Measures.lidar_beg_time > get_time_sec(imu_next.header.stamp)) {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                    }
                    if (non_station_start) {
                        state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                        state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                        state_out.acc << VEC_FROM_ARRAY(gravity_init);
                        state_out.acc *= -1;
                    } else {
                        state_in.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
                        state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
                        state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;
                    }
                    if (gravity_align) {
                        Eigen::Matrix3d rot_init;
                        p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
                        p_imu->Set_init(state_in.gravity, rot_init);
                        state_in.gravity = state_out.gravity = p_imu->gravity_;
                        state_in.rot = state_out.rot = rot_init;
                        state_in.rot.normalize();
                        state_out.rot.normalize();
                        state_out.acc = -rot_init.transpose() * state_out.gravity;
                    }
                    kf_input.change_x(state_in);
                    kf_output.change_x(state_out);
                }
            } else {
                if (!p_imu->gravity_align_) {
                    state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc *= -1;
                }
            }

            /*** 动态切分局部地图立方体视野 ***/
            lasermap_fov_segment();

            /*** 对当前帧去畸变点云执行体素降采样滤波以减少特征计算量 ***/
            t1 = omp_get_wtime();
            if (space_down_sample) {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            } else {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            }
            // 对点云按时间序列进行压缩切片组织，为接下来的点到平面/逐点计算做结构准备
            time_seq = time_compressing<int>(feats_down_body);
            feats_down_size = feats_down_body->points.size();

            /*** 历史首帧点云：用来构建初始地图 ikD-Tree 根节点 ***/
            if (!init_map) {
                if (ikdtree.Root_Node == nullptr)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                }

                feats_down_world->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++) {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if (init_feats_world->size() < init_map_size) continue;
                // 全量调用 Build 接口生成 K-D 树基础拓扑
                ikdtree.Build(init_feats_world->points);
                init_map = true;
                publish_init_kdtree(pubLaserCloudMap);
                continue;
            }

            /*** 迭代卡尔曼滤波前向状态递推与迭代更新计算 ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);

            t2 = omp_get_wtime();

            crossmat_list.reserve(feats_down_size);
            pbody_list.reserve(feats_down_size);

            // 预先缓存点云每个点在体坐标系下的三维坐标及其反对称矩阵（Skew-symmetric Matrix）
            for (size_t i = 0; i < feats_down_body->size(); i++) {
                V3D point_this(feats_down_body->points[i].x,
                               feats_down_body->points[i].y,
                               feats_down_body->points[i].z);
                pbody_list[i] = point_this;
                if (extrinsic_est_en) {
                    if (!use_imu_as_input) {
                        point_this = kf_output.x_.offset_R_L_I.normalized() * point_this + kf_output.x_.offset_T_L_I;
                    } else {
                        point_this = kf_input.x_.offset_R_L_I.normalized() * point_this + kf_input.x_.offset_T_L_I;
                    }
                } else {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                }
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);
                crossmat_list[i] = point_crossmat;
            }

            // ==================== 分支一：IMU 作为系统观测（Output 模式分支） ====================
            if (!use_imu_as_input) {
                bool imu_upda_cov = false;
                effct_feat_num = 0;
                /**** 点对点状态估计更新 ****/

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++) {
                    PointType &point_body = feats_down_body->points[idx + time_seq[k]];

                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                    if (is_first_frame) {
                        if (imu_en) {
                            while (time_current > get_time_sec(imu_next.header.stamp)) {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                imu_deque.pop_front();
                            }

                            angvel_avr
                                    << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr
                                    << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                        }
                        is_first_frame = false;
                        imu_upda_cov = true;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }
                    if (imu_en) {
                        bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                        while (imu_comes) {
                            imu_upda_cov = true;
                            angvel_avr
                                    << imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr
                                    << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                            /*** 基于 IMU 观测值的滤波器状态前向传播推算预测 ***/
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            double dt = get_time_sec(imu_last.header.stamp) - time_predict_last_const;
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            time_predict_last_const = get_time_sec(imu_last.header.stamp);
                            imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                            {
                                double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;

                                if (dt_cov > 0.0) {
                                    time_update_last = get_time_sec(imu_last.header.stamp);
                                    double propag_imu_start = omp_get_wtime();

                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                                    propag_time += omp_get_wtime() - propag_imu_start;
                                    double solve_imu_start = omp_get_wtime();
                                    // 结合 IMU 本身的误差观测方程更新滤波器内误差协方差
                                    kf_output.update_iterated_dyn_share_IMU();
                                    solve_time += omp_get_wtime() - solve_imu_start;
                                }
                            }
                        }
                    }

                    double dt = time_current - time_predict_last_const;
                    double propag_state_start = omp_get_wtime();
                    if (!prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;
                        }
                    }
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    propag_time += omp_get_wtime() - propag_state_start;
                    time_predict_last_const = time_current;

                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1) {
                        RCLCPP_WARN(logger, "No point, skip this scan!\n");
                        idx += time_seq[k];
                        continue;
                    }
                    // 核心数学步骤：执行迭代卡尔曼滤波观测模型更新（计算点云残差并实现流形状态位姿更新）
                    if (!kf_output.update_iterated_dyn_share_modified()) {
                        idx = idx + time_seq[k];
                        continue;
                    }

                    if (prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (!imu_en && (dt_cov >= imu_time_inte))
                        {
                            double propag_cov_start = omp_get_wtime();
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            imu_upda_cov = false;
                            time_update_last = time_current;
                            propag_time += omp_get_wtime() - propag_cov_start;
                        }
                    }

                    solve_start = omp_get_wtime();

                    // 高频发布：如果不等待降采样帧完成就输出位姿，进行亚帧级高频里程计发布
                    if (publish_odometry_without_downsample) {
                        publish_odometry(pubOdomAftMapped, tf_broadcaster);
                        if (runtime_pos_log) {
                            state_out = kf_output.x_;
                            euler_cur = SO3ToEuler(state_out.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                     << euler_cur.transpose() << " " << state_out.pos.transpose() << " "
                                     << state_out.vel.transpose() << " " << state_out.omg.transpose() << " "
                                     << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " "
                                     << state_out.bg.transpose() << " " << state_out.ba.transpose() << " "
                                     << feats_undistort->points.size() << endl;
                        }
                    }

                    // 位姿收敛后，将当前小切片内的体坐标系雷达点转换投影到世界坐标系下
                    for (int j = 0; j < time_seq[k]; j++) {
                        PointType &point_body_j = feats_down_body->points[idx + j + 1];
                        PointType &point_world_j = feats_down_world->points[idx + j + 1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }

                    solve_time += omp_get_wtime() - solve_start;

                    update_time += omp_get_wtime() - t_update_start;
                    idx += time_seq[k];
                }
            }
            // ==================== 分支二：IMU 作为系统状态转移输入（Input 模式分支，经典FAST-LIO） ====================
            else {
                bool imu_prop_cov = false;
                effct_feat_num = 0;

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++) {
                    PointType &point_body = feats_down_body->points[idx + time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (is_first_frame) {
                        while (time_current > get_time_sec(imu_next.header.stamp)) {
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                        }
                        imu_prop_cov = true;

                        is_first_frame = false;
                        t_last = time_current;
                        time_update_last = time_current;
                        {
                            input_in.gyro << imu_last.angular_velocity.x,
                                    imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;

                            input_in.acc << imu_last.linear_acceleration.x,
                                    imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        }
                    }

                    // 步进提取两个点云微切片之间的全部 IMU 脉冲，进行前向状态推算
                    while (time_current > get_time_sec(imu_next.header.stamp))
                    {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        input_in.gyro
                                << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        input_in.acc
                                << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;

                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        double dt = get_time_sec(imu_last.header.stamp) - t_last;

                        double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = get_time_sec(imu_last.header.stamp);
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false);
                        t_last = get_time_sec(imu_last.header.stamp);
                        imu_prop_cov = true;
                    }

                    double dt = time_current - t_last;
                    t_last = time_current;
                    double propag_start = omp_get_wtime();

                    if (!prop_at_freq_of_imu) {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0) {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = time_current;
                        }
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false);

                    propag_time += omp_get_wtime() - propag_start;

                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1) {
                        RCLCPP_WARN(logger, "No point, skip this scan!\n");

                        idx += time_seq[k];
                        continue;
                    }
                    // 执行卡尔曼观测项修正更新
                    if (!kf_input.update_iterated_dyn_share_modified()) {
                        idx = idx + time_seq[k];
                        continue;
                    }

                    solve_start = omp_get_wtime();

                    if (publish_odometry_without_downsample) {
                        publish_odometry(pubOdomAftMapped, tf_broadcaster);
                        if (runtime_pos_log) {
                            state_in = kf_input.x_;
                            euler_cur = SO3ToEuler(state_in.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                     << euler_cur.transpose() << " " << state_in.pos.transpose() << " "
                                     << state_in.vel.transpose() << " " << state_in.bg.transpose() << " "
                                     << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " "
                                     << feats_undistort->points.size() << endl;
                        }
                    }

                    // 转换雷达切帧点云到世界坐标系
                    for (int j = 0; j < time_seq[k]; j++) {
                        PointType &point_body_j = feats_down_body->points[idx + j + 1];
                        PointType &point_world_j = feats_down_world->points[idx + j + 1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }
                    solve_time += omp_get_wtime() - solve_start;

                    update_time += omp_get_wtime() - t_update_start;
                    idx = idx + time_seq[k];
                }
            }

            /******* 正常发布：在完成当前大帧全部状态估计后的统一发布 *******/
            if (!publish_odometry_without_downsample) {
                publish_odometry(pubOdomAftMapped, tf_broadcaster);
            }

            /*** 最终增量式融合新特征点进入局部地图 ikD-Tree 树结构 ***/
            t3 = omp_get_wtime();

            if (feats_down_size > 4) {
                map_incremental();
            }

            t5 = omp_get_wtime();
            /******* ROS 广播发布更新后的各类可视化主题数据（全量点云、局部体点云及轨迹） *******/
            if (path_en) publish_path(pubPath);
            if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);

            /*** 终端信息输出及系统性能日志文件打存 ***/
            if (runtime_pos_log) {
                frame_num++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                { aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num; }
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
                aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
                       t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
                       aver_time_propag);
                if (!publish_odometry_without_downsample) {
                    if (!use_imu_as_input) {
                        state_out = kf_output.x_;
                        euler_cur = SO3ToEuler(state_out.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << state_out.pos.transpose() << " "
                                 << state_out.vel.transpose() << " " << state_out.omg.transpose() << " "
                                 << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " "
                                 << state_out.bg.transpose() << " " << state_out.ba.transpose() << " "
                                 << feats_undistort->points.size() << endl;
                    } else {
                        state_in = kf_input.x_;
                        euler_cur = SO3ToEuler(state_in.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << state_in.pos.transpose() << " "
                                 << state_in.vel.transpose() << " " << state_in.bg.transpose() << " "
                                 << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " "
                                 << feats_undistort->points.size() << endl;
                    }
                }
                dump_lio_state_to_log(fp);
            }
        }
        rate.sleep();
    }
    //-------------------------- 节点退出前收尾：执行最终地图数据的保存 -----------------------------------
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // 写入二进制 PCD
    }
    fout_out.close();
    fout_imu_pbp.close();

    return 0;
}
