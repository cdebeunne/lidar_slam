#include "feature.h"
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>

Feature::Feature(pcl::PointCloud<pcl::PointXYZ>::Ptr i_featureCloud)
{
    _featureCloud = i_featureCloud;
    Eigen::Vector4f baryTemp;
    compute3DCentroid (*_featureCloud, baryTemp);
    _barycenter << baryTemp(0), baryTemp(1), baryTemp(2);
    computeCovarianceMatrix(*_featureCloud, baryTemp, _covarianceMatrix);
    size = _featureCloud->size();

    // eigen computation
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(_covarianceMatrix);
    _eigenValues = eigensolver.eigenvalues();
    _eigenVectors = eigensolver.eigenvectors();
}

Edge::Edge(pcl::PointCloud<pcl::PointXYZ>::Ptr i_edgeCloud): Feature(i_edgeCloud)
{
    // For an edge, the direction is the eigen vectors of the biggest eigen value
    _direction << _eigenVectors(0,2), _eigenVectors(1,2), _eigenVectors(2,2);

    // Flip the direction toward the sky
    Eigen::Vector3f dir;
    dir << 0,0,1;
    if (dir.dot(_direction) <0)
    {
        _direction = _direction*(-1);
    }
}

Plane::Plane(pcl::PointCloud<pcl::PointXYZ>::Ptr i_planeCloud): Feature(i_planeCloud)
{
    // For a plane, the direction is the normal of the plane
    // iow the eigen vector of the smallest eigen value
    _direction << _eigenVectors(0,0), _eigenVectors(1,0), _eigenVectors(2,0);

    // Flip the normal in the direction of the sensor
    if (_barycenter.dot(_direction) > 0)
    {
        _direction = _direction*(-1);
    }
    _direction = _direction/_direction.norm();
}
