#ifndef DISPLAY_H
#define DISPLAY_H

#include "utility.h"

#include "feature.h"

/*!
 * \file display.h
 * \brief handles point cloud graphical vizualization
 * \author César Debeunne
 */

class Display
{
public:
    Display();
    /**
     * \brief Display a point cloud with pcl vizualizer
     * \param pointCloud
     */
    void dispPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

    /**
     * \brief Update the point cloud in the viewer
     * \param pointCloud
     */
    void updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

    /**
     * \brief Display a RGB point cloud with pcl vizualizer
     * \param colorCloud
     */
    void dispColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud);

    /**
     * \brief Display a feature and its direction with pcl vizualizer
     * \param feature
     */
    void dispFeature(Feature feature, std::string id);

    /**
     * \brief Display two features
     * \param feature1, feature2
     */
    void dispCorespondance(Feature feature1, Feature feature2);

    /**
     * \brief display a complete set of edges
     * \param edgeVect
     */
    void dispEdgeVect(std::vector<Edge> edgeVect);

    /**
     * \brief display a complete set of planes
     * \param planeVect
     */
    void dispPlaneVect(std::vector<Plane> planeVect);

    /**
     * \brief display matched edges on two viewports
     * \param corespEdge and the two corespondings edgeVect
     */
    void dispMatchedEdges(Eigen::MatrixXi corespEdge, std::vector<Edge> edgeVect, std::vector<Edge> edgeVect1);

    /**
     * \brief display matched planes on two viewports
     * \param corespPlane and the two corespondings planeVect
     */
    void dispMatchedPlanes(Eigen::MatrixXi corespPlane, std::vector<Plane> planeVect, std::vector<Plane> planeVect1);
    ~Display();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer; /*!< the pcl viewer */
    Eigen::MatrixXf colors;
};

#endif // DISPLAY_H
