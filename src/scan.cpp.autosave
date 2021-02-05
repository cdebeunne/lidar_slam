#include <iostream>

#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>

#include "scan.h"
#include "feature.h"

Scan::Scan(pcl::PointCloud<pcl::PointXYZ>::Ptr i_cloud){
    _cloud=i_cloud;
    _pose = Eigen::VectorXd::Zero(6);
}


ScanVLP16::ScanVLP16(pcl::PointCloud<pcl::PointXYZ>::Ptr i_cloud): Scan(i_cloud)
{
    LENGTH = 1800;
    WIDTH = 16;
}

ScanHDL64::ScanHDL64(pcl::PointCloud<pcl::PointXYZ>::Ptr i_cloud): Scan(i_cloud)
{
    LENGTH = 4500;
    WIDTH = 64;
}
