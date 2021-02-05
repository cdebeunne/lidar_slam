#include "featuredetector.h"

#include <pcl/common/distances.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

void CESARFeatureDetector::detect(const std::shared_ptr<Scan> &scan, std::vector<Edge> &edgeVect, std::vector<Plane> &planeVect)
{
    WIDTH = scan->getWidth();
    LENGTH = scan->getLength();

    // Reinit Memory
    _orgCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(WIDTH,LENGTH));
    _planeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(WIDTH,LENGTH));
    _edgeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(WIDTH,LENGTH));
    _edgeVect.clear();
    _planeVect.clear();
    edgeVect.clear();
    planeVect.clear();

    // Perform detection
    _cloud = scan->getCloud();
    cloudOrganizer();
    smoothnessCalculator();
    featureCloudComputation();
    edgeSegmentation();
    planeSegmentation();

    edgeVect = _edgeVect;
    planeVect = _planeVect;
}

int CESARFeatureDetector::modulo(int idx, int length){
    if (idx < 0){
        idx = idx+length;
    }
    if (idx > length-1){
        idx = idx-length;
    }
    return idx;
}

void CESARFeatureDetector::smoothnessCalculator()
{
    _smoothnessCloud = Eigen::MatrixXf::Zero(WIDTH,LENGTH);
    _labelCloud = Eigen::MatrixXf::Zero(WIDTH,LENGTH);

    // creation of the origin point
    pcl::PointXYZ origin;
    origin.x=0;
    origin.y=0;
    origin.z=0;
    // parsing the whole cloud
    for(int i=0; i < WIDTH; i++)
    {
        for(int j=0; j < LENGTH; j++)
        {
            // check if the point is valid
            pcl::PointXYZ scorePoint = _orgCloud->at(i,j);
            if (scorePoint.x==0 || std::isnan(scorePoint.x)){
                continue;
            }

            // Establishing the neighbours indices...
            int goodPointsIdx[SLICE];
            int counter = 0;
            int idx = modulo(j-1, LENGTH);

            // ...backward...
            while (counter < SLICE/2){
                pcl::PointXYZ pt = _orgCloud->at(i,idx);
                if (pt.x != 0 && !std::isnan(pt.x)){
                    goodPointsIdx[counter] = idx;
                    counter++;
                    idx = modulo(idx-1, LENGTH);
                } else{
                    idx = modulo(idx-1, LENGTH);
                }
            }
            counter = 0;
            idx = modulo(j+1, LENGTH);

            // ... and forward
            while (counter < SLICE/2){
                pcl::PointXYZ pt = _orgCloud->at(i,idx);
                if (pt.x != 0 && !std::isnan(pt.x)){
                    goodPointsIdx[counter+SLICE/2] = idx;
                    counter++;
                    idx = modulo(idx+1, LENGTH);
                } else{
                    idx = modulo(idx+1, LENGTH);
                }
            }

            // score calculation
            Eigen::VectorXf distVect(SLICE);
            for (int k=0; k<SLICE; k++){
                pcl::PointXYZ slicePoint = _orgCloud->at(i, goodPointsIdx[k]);
                distVect(k) = pcl::euclideanDistance(scorePoint, slicePoint);
            }
            _smoothnessCloud(i,j) = distVect.sum()/(SLICE*pcl::euclideanDistance(scorePoint,origin));

            // labelCloud & feature clouds edition
            if (_smoothnessCloud(i,j)<C_PLANE){
                _labelCloud(i,j) = 1;
            }
            if (_smoothnessCloud(i,j)>C_EDGE){
                _labelCloud(i,j) = 2;
            }
        }
    }
}

void CESARFeatureDetector::featureCloudComputation(){
    for(int i=0; i < WIDTH; ++i)
    {
        for(int j=0; j < LENGTH; j++)
        {
            pcl::PointXYZ scorePoint = _orgCloud->at(i,j);
            int up = modulo(i+1, WIDTH);
            int down = modulo(i-1, WIDTH);
            int left = modulo(j-1, LENGTH);
            int right = modulo(j+1, LENGTH);
            if (_labelCloud(i,j)==1){
                bool isPlane = ((_labelCloud(up,j)==1)||(_labelCloud(down,j)==1))&&((_labelCloud(i,left)==1)||(_labelCloud(i,right)==1));
                if (isPlane){
                    _planeCloud->at(i,j) = scorePoint;
                }
            }
            if (_labelCloud(i,j)==2){
                bool isEdge = true;
                if (isEdge){
                    _edgeCloud->at(i,j) = scorePoint;
                }
            }
        }
    }
}

void CESARFeatureDetector::edgeSegmentation(){
    // Create the filtering object: downsample the dataset using a leaf size

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (_edgeCloud);
    vg.setLeafSize (LEAFSIZE, LEAFSIZE, LEAFSIZE);
    vg.filter (*cloud_filtered);

    // Creating the KdTree object for the search method of the extraction

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (DISTTHRESHOLD_EDGE);
    ec.setMinClusterSize (MINCLUSTER_EDGE);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_filtered)[*pit]);
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      Edge cloud_edge(cloud_cluster);

      // check if it is a real edge
      if (cloud_edge.getEigenValues()(2) > 10*cloud_edge.getEigenValues()(1) && cloud_edge.getDirection()(2) > 0.1){
        _edgeVect.push_back(cloud_edge);
      }
    }
}

void CESARFeatureDetector::planeSegmentation(){
    // Create the filtering object: downsample the dataset using a leaf size

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (_planeCloud);
    vg.setLeafSize (LEAFSIZE, LEAFSIZE, LEAFSIZE);
    vg.filter (*cloud_filtered);

    // Creating the KdTree object for the search method of the extraction

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (DISTTHRESHOLD_PLANE);
    ec.setMinClusterSize (MINCLUSTER_PLANE);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_filtered)[*pit]);
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      Plane cloud_plane(cloud_cluster);

      // check if it is a real plane
      if (cloud_plane.getEigenValues()(1) > 20*cloud_plane.getEigenValues()(0) && std::abs(cloud_plane.getDirection()(2)) < 0.1){
        _planeVect.push_back(cloud_plane);
      }
    }
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr CESARFeatureDetector::colorComputation(){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud (new pcl::PointCloud<pcl::PointXYZRGB>(WIDTH,LENGTH));
    for(int i=0; i < WIDTH; ++i)
    {
        for(int j=0; j < LENGTH; j++)
        {
            pcl::PointXYZ ptIn = _orgCloud->at(i,j);
            pcl::PointXYZRGB ptOut;
            ptOut.x = ptIn.x;
            ptOut.y = ptIn.y;
            ptOut.z = ptIn.z;
            if (_labelCloud(i,j) == 2){
                uint8_t r = 0, g = 255, b = 0;
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                ptOut.rgb = *reinterpret_cast<float*>(&rgb);
            } else if (_labelCloud(i,j) == 1){
                uint8_t r = 0, g = 0, b = 255;
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                ptOut.rgb = *reinterpret_cast<float*>(&rgb);
            } else {
                uint8_t r = 255, g = 0, b = 0;
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                ptOut.rgb = *reinterpret_cast<float*>(&rgb);
            }
            colorCloud->at(i,j) = ptOut;
        }
    }
    return colorCloud;
}

VLP16CESARDetector::VLP16CESARDetector(){
    SLICE = 10;
    MINCLUSTER_EDGE = 5;
    MINCLUSTER_PLANE = 20;
    DISTTHRESHOLD_EDGE = 0.5;
    DISTTHRESHOLD_PLANE = 2;
    C_EDGE = 0.05;
    C_PLANE = 0.02;
    LEAFSIZE = 0.1;
}

void VLP16CESARDetector::cloudOrganizer()
{
    for(int i=0; i < _cloud->size(); ++i)
    {
        pcl::PointXYZ pt = _cloud->at(i);
        double theta = (std::atan2(pt.y, pt.x)+M_PI)*180/M_PI;
        double phi = std::atan2(pt.z, sqrt(pt.y*pt.y + pt.x*pt.x))*180/M_PI+15;
        int row = std::round(phi*WIDTH/30);
        int column = std::round(theta*LENGTH/360);
        row = std::max(row-1, 0);
        column = std::max(column-1,0);
        _orgCloud->at(row, column) = pt;
    }
}

HDL64CESARDetector::HDL64CESARDetector()
{
    SLICE = 10;
    MINCLUSTER_EDGE = 8;
    MINCLUSTER_PLANE = 50;
    DISTTHRESHOLD_EDGE = 0.25;
    DISTTHRESHOLD_PLANE = 0.1;
    C_EDGE = 0.12;
    C_PLANE = 0.05;
    LEAFSIZE = 0.05;
}

void HDL64CESARDetector::cloudOrganizer()
{
    for(int i=0; i < _cloud->size(); ++i)
    {
        pcl::PointXYZ pt = _cloud->at(i);
        double theta = (std::atan2(pt.y, pt.x)+M_PI)*180/M_PI;
        double phi = std::atan2(pt.z, sqrt(pt.y*pt.y + pt.x*pt.x))*180/M_PI+23.72;
        int row = std::round(phi*WIDTH/29.42);
        int column = std::round(theta*LENGTH/360);
        row = modulo(std::max(row, 0),WIDTH);
        column = modulo(std::max(column,0), LENGTH);
        _orgCloud->at(row, column) = pt;
    }
}
