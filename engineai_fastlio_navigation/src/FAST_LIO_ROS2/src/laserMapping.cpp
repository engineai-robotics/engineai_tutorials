// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <atomic>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
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
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "lio_geometry.hpp"
#include "flat_ground.hpp"
#include "velocity_aiding.hpp"
#include "sensor_io.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
bool lidar_is_snapshot = false;
double geometry_threshold = 0.001;
LioGeometry geometry_health;
bool flat_ground_model_active = false;
double flat_ground_z = -.81;
std::vector<Eigen::Vector3d> flat_ground_points;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = fast_lio_runtime::root().string();
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0;
std::atomic<int> publish_count{0};
builtin_interfaces::msg::Time lidar_end_stamp;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool    is_first_lidar = true;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
struct BufferedLidarFrame {
    PointCloudXYZI::Ptr cloud;
    double stamp;
    builtin_interfaces::msg::Time source_stamp;
    double received_steady;
    double input_age;
    double preprocess_sec;
};
// Keep cloud, source time and receive time together across queue resets.
deque<BufferedLidarFrame> lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
rclcpp::Clock::SharedPtr input_clock;
double lidar_input_age = 0, imu_input_age = 0;
double frame_input_age = 0, frame_preprocess_sec = 0, frame_queue_wait_sec = 0;
uint64_t lidar_callbacks = 0, imu_callbacks = 0;

double steady_seconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    if (!fp) return;
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    const double received = steady_seconds();
    const double preprocess_start_time = omp_get_wtime();
    const double input_age = input_clock->now().seconds() - get_time_sec(msg->header.stamp);
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    if (lidar_is_snapshot) {
        for (auto & point : ptr->points) point.curvature = 0.0f;
    }
    const double preprocess_sec = steady_seconds()-received;
    std::lock_guard<std::mutex> lock(mtx_buffer);
    scan_count ++;
    double cur_time = get_time_sec(msg->header.stamp);
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
        lidar_pushed = false;
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    lidar_buffer.push_back({ptr, cur_time, msg->header.stamp, received, input_age, preprocess_sec});
    lidar_input_age = input_age;
    ++lidar_callbacks;
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    const double received = steady_seconds();
    const double preprocess_start_time = omp_get_wtime();
    const double input_age = input_clock->now().seconds() - get_time_sec(msg->header.stamp);
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    if (lidar_is_snapshot) {
        for (auto & point : ptr->points) point.curvature = 0.0f;
    }
    const double preprocess_sec = steady_seconds()-received;
    std::lock_guard<std::mutex> lock(mtx_buffer);
    double cur_time = get_time_sec(msg->header.stamp);
    scan_count ++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
        lidar_pushed = false;
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    lidar_buffer.push_back({ptr, last_timestamp_lidar, msg->header.stamp, received, input_age, preprocess_sec});
    lidar_input_age = input_age;
    ++lidar_callbacks;
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    

    // Preserve the integer input stamp, including when no correction is needed.
    msg->header.stamp = fast_lio_time::add_seconds(msg_in->header.stamp, -time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = fast_lio_time::add_seconds(msg_in->header.stamp, timediff_lidar_wrt_imu);
    }

    double timestamp = get_time_sec(msg->header.stamp);

    std::lock_guard<std::mutex> lock(mtx_buffer);
    imu_input_age = input_clock->now().seconds() - get_time_sec(msg_in->header.stamp);
    ++imu_callbacks;

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front().cloud;
        meas.lidar_beg_time = lidar_buffer.front().stamp;
        double scan_duration = 0.0;
        if (lidar_is_snapshot)
        {
            // Gazebo gpu_ray points share the header acquisition time. The
            // CustomMsg bridge's synthetic offsets must not create a rolling scan.
            // Keep the exact integer acquisition stamp for snapshot output.
        }
        else if (meas.lidar->points.size() <= 1) // time too little
        {
            scan_duration = lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            scan_duration = lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            scan_duration = meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        lidar_end_time = meas.lidar_beg_time + scan_duration;
        meas.lidar_end_time = lidar_end_time;
        lidar_end_stamp = fast_lio_time::add_seconds(lidar_buffer.front().source_stamp, scan_duration);

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    const auto & frame = lidar_buffer.front();
    frame_input_age = frame.input_age;
    frame_preprocess_sec = frame.preprocess_sec;
    frame_queue_wait_sec = steady_seconds()-frame.received_steady-frame.preprocess_sec;
    lidar_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = lidar_end_stamp;
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    // Keep every registered scan. Ctrl-C writes them to PCD/scans.pcd.
    if (pcd_save_en && feats_undistort && !feats_undistort->empty())
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
        {
            pcd_index++;
            string all_points_dir = fast_lio_runtime::file("PCD", "scans_" + to_string(pcd_index) + ".pcd");
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to " << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = lidar_end_stamp;
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = lidar_end_stamp;
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = lidar_end_stamp;
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = lidar_end_stamp;
    // laserCloudMap.header.frame_id = "camera_init";
    // pubLaserCloudMap->publish(laserCloudMap);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = lidar_end_stamp;
    set_posestamp(odomAftMapped.pose);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }
    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = lidar_end_stamp;
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    if (flat_ground_model_active) {
        ekfom_data.valid = !flat_ground_points.empty();
        flat_ground_model(flat_ground_points,s.rot.toRotationMatrix(),s.pos,flat_ground_z,
            ekfom_data.h_x,ekfom_data.h);
        return;
    }
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 
    geometry_health = LioGeometry();

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    geometry_health = lio_geometry(ekfom_data.h_x, geometry_threshold);
    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        // Gazebo's CustomMsg bridge is SensorDataQoS (best effort). The real
        // Livox driver is reliable, so this stays false outside simulation.
        this->declare_parameter<bool>("common.lidar_qos_best_effort", false);
        this->declare_parameter<bool>("common.lidar_is_snapshot", false);
        this->declare_parameter<bool>("common.concurrent_sensor_io", false);
        this->declare_parameter<int>("common.imu_queue_size", 10);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        bool lidar_qos_best_effort = false;
        this->get_parameter_or<bool>("common.lidar_qos_best_effort", lidar_qos_best_effort, false);
        this->get_parameter_or<bool>("common.lidar_is_snapshot", lidar_is_snapshot, false);
        input_clock = this->get_clock();
        degeneracy_fallback_ = this->declare_parameter<bool>("mapping.degeneracy_fallback", false);
        geometry_threshold = this->declare_parameter<double>("mapping.degeneracy_threshold", 0.001);
        if (!std::isfinite(geometry_threshold) || geometry_threshold <= 0 || geometry_threshold >= 1)
            throw std::invalid_argument("mapping.degeneracy_threshold must lie between zero and one");
        max_lidar_translation_ = this->declare_parameter<double>("mapping.max_lidar_correction_m", 0.0);
        max_lidar_rotation_ = this->declare_parameter<double>("mapping.max_lidar_correction_rad", 0.0);
        if (!std::isfinite(max_lidar_translation_) || !std::isfinite(max_lidar_rotation_) ||
            max_lidar_translation_ < 0 || max_lidar_rotation_ < 0)
            throw std::invalid_argument("Lidar correction limits must be finite and nonnegative");
        flat_ground_aiding_ = this->declare_parameter<bool>("mapping.flat_ground_aiding", false);
        flat_ground_z = this->declare_parameter<double>("mapping.flat_ground_z", -.81);
        validate_recovery_ = this->declare_parameter<bool>("mapping.validate_lidar_recovery", false);
        recovery_stable_frames_ = this->declare_parameter<int>("mapping.recovery_stable_frames", 3);
        recovery_max_residual_ = this->declare_parameter<double>("mapping.recovery_max_residual", .06);
        if (!std::isfinite(flat_ground_z) || !std::isfinite(recovery_max_residual_) ||
            recovery_max_residual_ <= 0 || recovery_stable_frames_ < 1)
            throw std::invalid_argument("Invalid flat-ground or recovery parameters");
        velocity_aiding_ = this->declare_parameter<bool>("velocity_aiding.enabled", false);
        height_aiding_ = this->declare_parameter<bool>("velocity_aiding.use_height", false);
        velocity_frame_ = this->declare_parameter<std::string>("velocity_aiding.frame", "imu_link");
        velocity_max_age_ = this->declare_parameter<double>("velocity_aiding.max_age", 0.08);
        if (!std::isfinite(velocity_max_age_) || velocity_max_age_ <= 0 || velocity_max_age_ > 0.5)
            throw std::invalid_argument("velocity_aiding.max_age must lie in (0, 0.5]");
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="camera_init";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        // Optional initial IMU pose in the mapping frame. Identity preserves
        // the upstream IMU-frame origin; simulation uses the nominal upright
        // base-to-IMU mounting pose so map XY is a horizontal plane.
        const auto initial_xyz = this->declare_parameter<std::vector<double>>(
            "mapping.initial_xyz", {0.0, 0.0, 0.0});
        const auto initial_rpy = this->declare_parameter<std::vector<double>>(
            "mapping.initial_rpy", {0.0, 0.0, 0.0});
        if (initial_xyz.size() != 3 || initial_rpy.size() != 3)
            throw std::invalid_argument("mapping.initial_xyz/initial_rpy require three values");
        auto initial_state = kf.get_x();
        initial_state.pos = V3D(initial_xyz[0], initial_xyz[1], initial_xyz[2]);
        initial_state.rot = (Eigen::AngleAxisd(initial_rpy[2], V3D::UnitZ()) *
            Eigen::AngleAxisd(initial_rpy[1], V3D::UnitY()) *
            Eigen::AngleAxisd(initial_rpy[0], V3D::UnitX())).toRotationMatrix();
        kf.change_x(initial_state);

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        fast_lio_runtime::configure(this->declare_parameter<std::string>(
            "output_root", fast_lio_runtime::root().string()));
        root_dir = fast_lio_runtime::root().string();
        RCLCPP_INFO(get_logger(), "Runtime output directory: %s", root_dir.c_str());

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<root_dir<<" file opened" << endl;
        else
            cout << "~~~~"<<root_dir<<" doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        const bool concurrent_sensor_io = this->get_parameter("common.concurrent_sensor_io").as_bool();
        const auto imu_queue_size = this->get_parameter("common.imu_queue_size").as_int();
        auto sensor_io = make_sensor_io_options(*this, concurrent_sensor_io, imu_queue_size);
        sensor_callback_group_ = sensor_io.group;
        const auto & sensor_options = sensor_io.subscription;
        auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(20));
        if (lidar_qos_best_effort) {
            lidar_qos.best_effort();
        }
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, lidar_qos, livox_pcl_cbk, sensor_options);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, lidar_qos_best_effort ? lidar_qos : rclcpp::SensorDataQoS(), standard_pcl_cbk, sensor_options);
        }
        // Gazebo IMU stays reliable. Do not switch this subscription to best effort.
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, sensor_io.imu_qos, imu_cbk, sensor_options);
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        pubQuality_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/lio/status", 10);
        if (velocity_aiding_) {
            const auto topic = this->declare_parameter<std::string>("velocity_aiding.topic", "/lio/leg_velocity");
            subVelocity_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
                topic, rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
                    if (msg->header.frame_id != velocity_frame_) return;
                    const auto & v = msg->twist.twist.linear;
                    Eigen::Vector3d velocity(v.x, v.y, v.z);
                    Eigen::Matrix3d covariance;
                    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
                        covariance(i, j) = msg->twist.covariance[i*6+j];
                    if (!velocity.allFinite() || velocity.norm() > 3 || !covariance.allFinite() ||
                        !covariance.isApprox(covariance.transpose(), 1e-8)) return;
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
                    if (solver.info() != Eigen::Success ||
                        solver.eigenvalues()[0] < 1e-6 || solver.eigenvalues()[2] > 4) return;
                    velocity_buffer_.push_back(msg);
                    while (velocity_buffer_.size() > 100) velocity_buffer_.pop_front();
                });
            if (height_aiding_) {
                subHeight_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "/lio/leg_height", rclcpp::SensorDataQoS(),
                    [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
                        if (msg->header.frame_id != "camera_init" || !std::isfinite(msg->pose.pose.position.z) ||
                            !std::isfinite(msg->pose.covariance[14]) || msg->pose.covariance[14] < 1e-6 ||
                            msg->pose.covariance[14] > 1) return;
                        height_buffer_.push_back(msg);
                        while (height_buffer_.size() > 100) height_buffer_.pop_front();
                    });
            }
        }
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        if (fp) fclose(fp);
    }

private:
    void timer_callback()
    {
        const double cycle_started = steady_seconds();
        if(sync_packages(Measures))
        {
            processing_started_age_ = this->get_clock()->now().seconds()-Measures.lidar_end_time;
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            lidar_update_applied_ = true;
            lidar_correction_rejected_ = false;
            lidar_translation_ = lidar_rotation_ = 0;
            ground_update_applied_ = false;
            ground_observation_ = FlatGroundObservation();
            recovery_rejection_ = false;
            if (degeneracy_fallback_) {
                auto predicted_state = kf.get_x();
                esekfom::dyn_share_datastruct<double> observation;
                observation.valid = true; observation.converge = true;
                h_share_model(predicted_state, observation);
                lidar_update_applied_ = observation.valid && geometry_health.rank == 6;
            }
            if (lidar_update_applied_) {
                auto predicted_state = kf.get_x();
                auto predicted_covariance = kf.get_P();
                auto update = [&]() {
                    kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
                };
                if (max_lidar_translation_ > 0 || max_lidar_rotation_ > 0) {
                    lidar_update_applied_ = guarded_lidar_update(kf, update,
                        max_lidar_translation_, max_lidar_rotation_, lidar_translation_, lidar_rotation_);
                    lidar_correction_rejected_ = !lidar_update_applied_;
                    if (lidar_correction_rejected_) ++lidar_corrections_rejected_;
                } else {
                    update();
                }
                if (lidar_update_applied_ && validate_recovery_) {
                    // Force a fresh association at the final filter state. The
                    // callback inside IKFoM precedes its last boxplus correction.
                    auto posterior = kf.get_x();
                    esekfom::dyn_share_datastruct<double> observation;
                    observation.valid = true; observation.converge = true;
                    h_share_model(posterior,observation);
                    if (!observation.valid || geometry_health.rank != 6 ||
                        !std::isfinite(res_mean_last) || res_mean_last > recovery_max_residual_) {
                        kf.change_x(predicted_state); kf.change_P(predicted_covariance);
                        lidar_update_applied_ = false; recovery_rejection_ = true;
                        ++recovery_rejections_;
                    }
                }
            }
            full_update_streak_ = lidar_update_applied_ ? full_update_streak_+1 : 0;
            if (flat_ground_aiding_ && !lidar_update_applied_) {
                const auto predicted_state = kf.get_x();
                std::vector<Eigen::Vector3d> points;
                points.reserve(feats_down_body->size());
                for (const auto & point : feats_down_body->points)
                    points.push_back(predicted_state.offset_R_L_I*Eigen::Vector3d(point.x,point.y,point.z)+
                        predicted_state.offset_T_L_I);
                ground_observation_ = select_flat_ground(points,predicted_state.rot.toRotationMatrix(),
                    predicted_state.pos,flat_ground_z);
                flat_ground_points = ground_observation_.points_imu;
                if (!flat_ground_points.empty()) {
                    flat_ground_model_active = true;
                    double translation=0,rotation=0;
                    ground_update_applied_ = guarded_lidar_update(kf,[&]() {
                        kf.update_iterated_dyn_share_modified(LASER_POINT_COV,solve_H_time);
                    },.15,.08,translation,rotation);
                    flat_ground_model_active = false;
                }
            }
            map_update_applied_ = lidar_update_applied_ &&
                (!validate_recovery_ || full_update_streak_ >= recovery_stable_frames_);
            apply_velocity_aiding();
            apply_height_aiding();
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            current_compute_sec_ = steady_seconds()-cycle_started;
            const double odom_started = steady_seconds();
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
            current_odom_publish_sec_ = steady_seconds()-odom_started;
            publish_quality();

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            if (map_update_applied_) map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            const double cloud_started = steady_seconds();
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            previous_cloud_publish_sec_ = steady_seconds()-cloud_started;
            previous_cycle_sec_ = steady_seconds()-cycle_started;
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

    void map_publish_callback()
    {
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            save_to_pcd();
            res->success = true;
            res->message = "Map saved.";
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

private:
    void apply_height_aiding()
    {
        height_status_ = height_aiding_ ? "stale_or_missing" : "disabled";
        if (!height_aiding_ || !velocity_aiding_) return;
        geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr latest;
        while (!height_buffer_.empty() && get_time_sec(height_buffer_.front()->header.stamp) <= lidar_end_time) {
            latest = height_buffer_.front(); height_buffer_.pop_front();
        }
        if (!latest || lidar_end_time-get_time_sec(latest->header.stamp) > velocity_max_age_) return;
        height_status_ = fuse_ground_height(kf, latest->pose.pose.position.z, latest->pose.covariance[14]) ?
            "accepted" : "innovation_rejected";
    }

    void apply_velocity_aiding()
    {
        velocity_status_ = velocity_aiding_ ? "stale_or_missing" : "disabled";
        if (!velocity_aiding_) return;
        geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr latest;
        while (!velocity_buffer_.empty() && get_time_sec(velocity_buffer_.front()->header.stamp) <= lidar_end_time) {
            latest = velocity_buffer_.front(); velocity_buffer_.pop_front();
        }
        if (!latest) return;
        const double age = lidar_end_time-get_time_sec(latest->header.stamp);
        if (age > velocity_max_age_) return;
        const auto & v = latest->twist.twist.linear;
        Eigen::Matrix3d covariance;
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
            covariance(i, j) = latest->twist.covariance[i*6+j];
        const bool accepted = fuse_body_velocity(kf, Eigen::Vector3d(v.x, v.y, v.z), covariance, velocity_nis_);
        velocity_status_ = accepted ? "accepted" : "innovation_rejected";
        if (accepted) { ++velocity_updates_; last_velocity_accept_ = lidar_end_time; }
    }

    void publish_quality()
    {
        diagnostic_msgs::msg::DiagnosticArray message;
        message.header.stamp = lidar_end_stamp;
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "fast_lio/geometry";
        status.hardware_id = "lidar_imu";
        status.level = geometry_health.rank < 6 || lidar_correction_rejected_ || recovery_rejection_ ? 1 : 0;
        status.message = recovery_rejection_ ? "lidar_recovery_candidate_rejected" :
            lidar_correction_rejected_ ? "lidar_correction_rejected" :
            (geometry_health.rank < 6 ? "weak_geometry" : "geometry_supported");
        auto add = [&](const std::string & key, const std::string & value) {
            diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = value;
            status.values.push_back(item);
        };
        add("pose_rank", std::to_string(geometry_health.rank));
        add("weakest_information", std::to_string(geometry_health.weakest_information));
        add("strongest_information", std::to_string(geometry_health.strongest_information));
        add("effective_features", std::to_string(effct_feat_num));
        add("mean_plane_residual_m", std::to_string(res_mean_last));
        add("velocity_norm_mps", std::to_string(state_point.vel.norm()));
        add("accel_bias_norm_mps2", std::to_string(state_point.ba.norm()));
        add("gyro_bias_norm_radps", std::to_string(state_point.bg.norm()));
        add("velocity_aiding", velocity_status_);
        add("velocity_aiding_nis", std::to_string(velocity_nis_));
        add("velocity_aiding_updates", std::to_string(velocity_updates_));
        add("height_aiding", height_status_);
        {
            // Sensor callbacks run in a separate mutually-exclusive group;
            // copy their shared counters under the same queue mutex.
            std::lock_guard<std::mutex> lock(mtx_buffer);
            add("lidar_callbacks", std::to_string(lidar_callbacks));
            add("imu_callbacks", std::to_string(imu_callbacks));
            add("pending_lidar_scans", std::to_string(lidar_buffer.size()));
            add("pending_imu_samples", std::to_string(imu_buffer.size()));
            add("latest_lidar_input_age_sec", std::to_string(lidar_input_age));
            add("latest_imu_input_age_sec", std::to_string(imu_input_age));
            add("frame_input_age_sec", std::to_string(frame_input_age));
            add("frame_preprocess_sec", std::to_string(frame_preprocess_sec));
            add("frame_queue_wait_sec", std::to_string(frame_queue_wait_sec));
        }
        add("processing_started_age_sec", std::to_string(processing_started_age_));
        add("compute_sec", std::to_string(current_compute_sec_));
        add("odom_tf_publish_sec", std::to_string(current_odom_publish_sec_));
        add("previous_cloud_publish_sec", std::to_string(previous_cloud_publish_sec_));
        add("previous_cycle_sec", std::to_string(previous_cycle_sec_));
        const bool motion_supported = ((geometry_health.rank == 6 && lidar_update_applied_) ||
            (velocity_aiding_ && lidar_end_time-last_velocity_accept_ < 0.3)) && state_point.vel.norm() < 2;
        add("motion_supported", motion_supported ? "true" : "false");
        add("lidar_update_applied", lidar_update_applied_ ? "true" : "false");
        add("lidar_correction_rejected", lidar_correction_rejected_ ? "true" : "false");
        add("lidar_correction_translation_m", std::to_string(lidar_translation_));
        add("lidar_correction_rotation_rad", std::to_string(lidar_rotation_));
        add("lidar_corrections_rejected", std::to_string(lidar_corrections_rejected_));
        add("ground_update_applied", ground_update_applied_ ? "true" : "false");
        add("flat_ground_enabled", flat_ground_aiding_ ? "true" : "false");
        add("recovery_validation_enabled", validate_recovery_ ? "true" : "false");
        add("ground_inliers", std::to_string(ground_observation_.inliers));
        add("ground_rms_m", std::to_string(ground_observation_.rms));
        add("ground_predicted_tilt_rad", std::to_string(ground_observation_.predicted_tilt));
        add("map_update_applied", map_update_applied_ ? "true" : "false");
        add("full_update_streak", std::to_string(full_update_streak_));
        add("recovery_candidate_rejected", recovery_rejection_ ? "true" : "false");
        add("recovery_rejections", std::to_string(recovery_rejections_));
        add("recovery_state", lidar_update_applied_ ?
            (map_update_applied_ ? "tracking" : "confirming") :
            (ground_update_applied_ ? "ground_supported" : "waiting_geometry"));
        message.status.push_back(status);
        pubQuality_->publish(message);
    }

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pubQuality_;
    rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
    double processing_started_age_ = 0, current_compute_sec_ = 0, current_odom_publish_sec_ = 0;
    double previous_cloud_publish_sec_ = 0, previous_cycle_sec_ = 0;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr subVelocity_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subHeight_;
    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> height_buffer_;
    std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr> velocity_buffer_;
    bool velocity_aiding_ = false;
    bool height_aiding_ = false;
    std::string height_status_ = "disabled";
    bool degeneracy_fallback_ = false, lidar_update_applied_ = true;
    bool lidar_correction_rejected_ = false;
    double max_lidar_translation_ = 0, max_lidar_rotation_ = 0;
    double lidar_translation_ = 0, lidar_rotation_ = 0;
    uint64_t lidar_corrections_rejected_ = 0;
    bool flat_ground_aiding_ = false, validate_recovery_ = false;
    bool ground_update_applied_ = false, map_update_applied_ = false, recovery_rejection_ = false;
    int recovery_stable_frames_ = 3, full_update_streak_ = 0;
    uint64_t recovery_rejections_ = 0;
    double recovery_max_residual_ = .06;
    FlatGroundObservation ground_observation_;
    std::string velocity_frame_, velocity_status_ = "disabled";
    double velocity_max_age_ = 0.08, velocity_nis_ = 0;
    double last_velocity_accept_ = -1e9;
    uint64_t velocity_updates_ = 0;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    auto node = std::make_shared<LaserMappingNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcd_save_en && pcl_wait_save->size() > 0)
    {
        string all_points_dir = fast_lio_runtime::file("PCD", "scans.pcd");
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to " << all_points_dir
             << " (" << pcl_wait_save->size() << " points)" << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    else if (pcd_save_en)
    {
        cout << "No scan was accumulated, map was not saved" << endl;
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        if (!fp2) { std::cerr << "Cannot write " << log_dir << std::endl; return 1; }
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
