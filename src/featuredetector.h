#ifndef FEATUREDETECTOR_H
#define FEATUREDETECTOR_H

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include "scan.h"

/*!
 * \file featuredetector.h
 * \brief a class that contains every calculation dedicated to
 * feature computation
 * \author César Debeunne
 */


class FeatureDetector
{
public:
    FeatureDetector(){}
    virtual ~FeatureDetector(){}

    /*!
    *  \brief virtual function that launches the detection process
    *  \param a pointer to the scan, a pointer to the edge vector
    * and a pointer to the plane vector
    */
    virtual void detect(const std::shared_ptr<Scan> &scan, std::vector<Edge> &edgeVect, std::vector<Plane> &planeVect)=0;
};

class CESARFeatureDetector : public FeatureDetector
{
public:
    CESARFeatureDetector(){}
    ~CESARFeatureDetector(){}
    virtual void detect(const std::shared_ptr<Scan> &scan, std::vector<Edge> &edgeVect, std::vector<Plane> &planeVect);

    /*!
    *  \brief function that constructs the color cloud for display purpose
    *  \return color Cloud
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorComputation();

protected :
    int WIDTH;
    int LENGTH;
    int SLICE; /*!< number of points to compute the smoothness score*/
    int MINCLUSTER_EDGE; /*!< the minimum size of an edge cluster*/
    int MINCLUSTER_PLANE; /*!< the minimum size of a plane cluster*/
    float DISTTHRESHOLD_EDGE; /*!< threshold for edge segmentation*/
    float DISTTHRESHOLD_PLANE; /*!< threshold for plane segmentation*/
    float C_PLANE; /*!< smoothness score for planes*/
    float C_EDGE; /*!< smoothness score for edges*/
    float LEAFSIZE;

    /*!
    *  \brief calculates the smoothness score of each point and stores it in a matrix
    */
    void smoothnessCalculator();

    /*!
    *  \brief builds the edge cloud and the plane cloud
    */
    void featureCloudComputation();

    /*!
    *  \brief segmentation of the edge cloud into features
    */
    void edgeSegmentation();

    /*!
    *  \brief segmentation of the plane cloud into features
    */
    void planeSegmentation();

    /*!
    *  \brief virtual function that stores the cloud in a polar matrix
    */
    virtual void cloudOrganizer()=0;

    int modulo(int idx, int length);

    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _orgCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _planeCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _edgeCloud;
    std::vector<Edge> _edgeVect;
    std::vector<Plane> _planeVect;
    Eigen::MatrixXf _smoothnessCloud; /*!< the matrix of the smoothness scores*/
    Eigen::MatrixXf _labelCloud; /*!< for display purpose*/
};

class VLP16CESARDetector: public CESARFeatureDetector
{
public:
    VLP16CESARDetector();

private:
    virtual void cloudOrganizer();
};

class HDL64CESARDetector: public CESARFeatureDetector
{
public:
    HDL64CESARDetector();

private:
    virtual void cloudOrganizer();
};

#endif // FEATUREDETECTOR_H
