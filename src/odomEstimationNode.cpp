// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"

using namespace std;

string lidarFrame;
string baseFrame;
string globalFrame;

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry, pubLaserAfterMappedPath;
nav_msgs::Path laserAfterMappedPath;
tf::TransformListener* tf_listener_ptr;
tf::StampedTransform tf_base2lidar;

void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    if (pointCloudSurfBuf.size() > 1) {
      pointCloudSurfBuf.pop();
    }
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    if (pointCloudEdgeBuf.size() > 1) {
      pointCloudEdgeBuf.pop();
    }
    mutex_lock.unlock();
}
void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    if (pointCloudBuf.size() > 1) {
      pointCloudBuf.pop();
    }
    mutex_lock.unlock();
}

bool is_odom_inited = false;
double total_time =0;
int total_frame=0;
[[noreturn]] void odom_estimation(){
    while(true){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()&& !pointCloudBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudBuf.empty() && (pointCloudBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                ROS_WARN("time stamp unaligned error and odom discarded, pls check your data --> odom correction"); 
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }
            //if time aligned 

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            pointCloudBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                ROS_INFO("average odom estimation time %f ms", total_time/total_frame);
            }



            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            //q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            tf::Transform tf_odom2lidar;
            tf_odom2lidar.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            tf_odom2lidar.setRotation(q);
            tf::Transform tf_odom2base;
            tf_odom2base.mult(tf_odom2lidar, tf_base2lidar.inverse());

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = globalFrame;
            laserOdometry.child_frame_id = baseFrame;
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = tf_odom2base.getRotation().x();
            laserOdometry.pose.pose.orientation.y = tf_odom2base.getRotation().y();
            laserOdometry.pose.pose.orientation.z = tf_odom2base.getRotation().z();
            laserOdometry.pose.pose.orientation.w = tf_odom2base.getRotation().w();
            laserOdometry.pose.pose.position.x = tf_odom2base.getOrigin().x();
            laserOdometry.pose.pose.position.y = tf_odom2base.getOrigin().y();
            laserOdometry.pose.pose.position.z = tf_odom2base.getOrigin().z();
            pubLaserOdometry.publish(laserOdometry);

            // 发布odometry_path
            geometry_msgs::PoseStamped laserAfterMappedPose;
            laserAfterMappedPose.header = laserOdometry.header;
            laserAfterMappedPose.pose = laserOdometry.pose.pose;
            laserAfterMappedPath.header.stamp = laserOdometry.header.stamp;
            laserAfterMappedPath.header.frame_id = globalFrame;
            laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
            pubLaserAfterMappedPath.publish(laserAfterMappedPath);

            // publish tf
            static tf::TransformBroadcaster br;
            // tf_odom2base.setOrigin(tf::Vector3(
            //     tf_odom2base.getOrigin().x(), tf_odom2base.getOrigin().y(), 0.0));
            br.sendTransform(
                tf::StampedTransform(tf_odom2base, pointcloud_time, globalFrame, baseFrame));

            ROS_INFO("[FLOAM] odometry elapsed time: %f ms",
                     (ros::Time::now().toSec() - pointcloud_time.toSec()) * 1000);
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(1);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);

    nh.param<string>("lidarFrame", lidarFrame, "/velodyne");
    nh.param<string>("baseFrame", baseFrame, "/base_link");
    nh.param<string>("globalFrame", globalFrame, "/odom");

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, map_resolution);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 1, velodyneHandler);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 1, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 1, velodyneSurfHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom", 10);
    pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/lidar_odom_path", 10);

    tf_listener_ptr = new tf::TransformListener(nh);
    // 先获得base_link到velodyne的tf转换关系，以便发布map到base_link的tf
    bool getTf = false;
    while (!getTf) {
      try {
        tf_listener_ptr->waitForTransform(baseFrame, lidarFrame, ros::Time(0), ros::Duration(1.0));
        tf_listener_ptr->lookupTransform(baseFrame, lidarFrame, ros::Time(0), tf_base2lidar);
        ROS_INFO("got TF between base_link and lidar");
        getTf = true;
      }
      catch (tf::TransformException ex) {
        ROS_WARN("Could not get tf between base_link and lidar: %s", ex.what());
      }
    }

    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}
