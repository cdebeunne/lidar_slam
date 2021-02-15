#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <math.h>
#include <ctime>

#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

const double RAD2DEG = 180.0 / M_PI;

inline Eigen::Matrix3d eul2rotm(double theta, double phi, double psi)
{
    Eigen::Matrix3d rotx;
    rotx << 1,0,0,
            0,cos(theta),-sin(theta),
            0,sin(theta),cos(theta);
    Eigen::Matrix3d roty;
    roty << cos(phi),0,sin(phi),
            0,1,0,
            -sin(phi),0,cos(phi);
    Eigen::Matrix3d rotz;
    rotz << cos(psi), -sin(psi),0,
            sin(psi),cos(psi),0,
            0,0,1;
    return rotx*roty*rotz;
}

#endif // UTILITY_H
