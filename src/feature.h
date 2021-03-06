#ifndef FEATURE_H
#define FEATURE_H

#include "utility.h"

/*!
 * \file feature.h
 * \brief everything you need to know about planes and edges
 * \author César Debeunne
 */

class Feature
{
public:
    Feature(pcl::PointCloud<pcl::PointXYZ>::Ptr i_featureCloud);

    /*!
    *  \brief accessor to the pcl point cloud
    *  \return featureCloud
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFeatureCloud(){return _featureCloud;}

    /*!
    *  \brief accessor to the barycenter
    *  \return barycenter
    */
    Eigen::Vector3f getBarycenter() const{return _barycenter;}

    /*!
    *  \brief accessor to the direction
    *  \return direction
    */
    Eigen::Vector3f getDirection() const{return _direction;}

    /*!
    *  \brief accessor to the eigen values
    *  \return eigen_values
    */
    Eigen::Vector3f getEigenValues() const{return _eigenValues;}

    /*!
    *  \brief accessor to the eigen vectors
    *  \return eigen_vectors
    */
    Eigen::Matrix3f getEigenVectors() const{return _eigenVectors;}

    /*!
    *  \brief accessor to the size
    *  \return size
    */
    int getSize() const{return size;}
    ~Feature(){}

protected:
    pcl::PointCloud<pcl::PointXYZ>::Ptr _featureCloud; /*!< pcl point cloud of the feature*/
    Eigen::Vector3f _barycenter;
    Eigen::Matrix3f _covarianceMatrix;
    Eigen::Vector3f _eigenValues;
    Eigen::Matrix3f _eigenVectors;
    Eigen::Vector3f _direction;
    int size;
};

class Edge : public Feature
{
public:
    Edge(pcl::PointCloud<pcl::PointXYZ>::Ptr i_edgeCloud);
    ~Edge(){}
};

class Plane : public Feature
{
public:
    Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr i_planeCloud);
    ~Plane(){}
};

#endif // FEATURE_H
