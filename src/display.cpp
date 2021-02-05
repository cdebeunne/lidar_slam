#include "display.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/io/io.h>
#include <Eigen/Dense>
#include <string>

Eigen::MatrixXf colorGenerator(){
    Eigen::MatrixXf colors(64,3);
    int counter = 0;
    for (int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            for(int k=0; k<4; k++){
                colors(counter,0) = (float)i/3;
                colors(counter,1) = (float)j/3;
                colors(counter,2) = (float)k/3;
                counter++;
            }
        }
    }
    return colors;
}

Display::Display():viewer(new pcl::visualization::PCLVisualizer ("3D viewer"))
{
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    colors = colorGenerator();
}

void Display::dispPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
    viewer->addPointCloud(pointCloud, "sample cloud");
    viewer->spinOnce(10);
}

void Display::updatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
    viewer->updatePointCloud(pointCloud, "sample cloud");
    viewer->spinOnce(10);
}

void Display::dispColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colorCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud, rgb, "sample cloud");
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
}

void Display::dispFeature(Feature feature, std::string id)
{
    viewer->addPointCloud<pcl::PointXYZ> (feature.getFeatureCloud(), id);

    // add the direction vector to the viewer
    pcl::PointXYZ p1;
    Eigen::Vector3d barycenter = feature.getBarycenter();
    p1.x = barycenter(0);
    p1.y = barycenter(1);
    p1.z = barycenter(2);

    Eigen::Vector3d dirPt;
    Eigen::Vector3d direction = feature.getDirection();
    dirPt =barycenter+direction;
    pcl::PointXYZ p2;
    p2.x = dirPt(0);
    p2.y = dirPt(1);
    p2.z = dirPt(2);
    viewer->addArrow(p2, p1, 1,0,0, false, "arrow "+id);

}

void Display::dispCorespondance(Feature feature1, Feature feature2)
{
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addPointCloud<pcl::PointXYZ> (feature1.getFeatureCloud(), "sample cloud1", v1);
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    viewer->addPointCloud<pcl::PointXYZ> (feature2.getFeatureCloud(), "sample cloud2", v2);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
}

void Display::dispEdgeVect(std::vector<Edge> edgeVect)
{
    viewer->setBackgroundColor (0.2, 0.2, 0.2);
    for (int i=0; i<edgeVect.size(); i++){
        viewer->addPointCloud<pcl::PointXYZ> (edgeVect[i].getFeatureCloud(),"cloud"+std::to_string(i));
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  colors(i,0), colors(i,1),colors(i,2), "cloud"+std::to_string(i));
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  3, "cloud"+std::to_string(i));
    }
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
}

void Display::dispPlaneVect(std::vector<Plane> planeVect)
{
    viewer->setBackgroundColor (0.2, 0.2, 0.2);
    for (int i=0; i<planeVect.size(); i++){
        dispFeature(planeVect[i],"cloud"+std::to_string(i));
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  colors(i,0), colors(i,1),colors(i,2), "cloud"+std::to_string(i));
    }
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
}

void Display::dispMatchedEdges(Eigen::MatrixXi corespEdge, std::vector<Edge> edgeVect, std::vector<Edge> edgeVect1)
{
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0.6, 0.6, 0.6, v1);
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    for (int i=0; i<corespEdge.rows(); i++){
        viewer->addPointCloud<pcl::PointXYZ> (edgeVect[corespEdge(i,0)].getFeatureCloud(),
                "edge1_"+std::to_string(i), v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  colors(i,0), colors(i,1),colors(i,2), "edge1_"+std::to_string(i), v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  3, "edge1_"+std::to_string(i), v1);
        viewer->addPointCloud<pcl::PointXYZ> (edgeVect1[corespEdge(i,1)].getFeatureCloud(),
                "edge2_"+std::to_string(i), v2);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  colors(i,0), colors(i,1),colors(i,2), "edge2_"+std::to_string(i), v2);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  3,"edge2_"+std::to_string(i), v2);
    }
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
}

void Display::dispMatchedPlanes(Eigen::MatrixXi corespPlane, std::vector<Plane> planeVect, std::vector<Plane> planeVect1)
{
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0.6, 0.6, 0.6, v1);
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    for (int i=0; i<corespPlane.rows(); i++){
        viewer->addPointCloud<pcl::PointXYZ> (planeVect[corespPlane(i,0)].getFeatureCloud(),
                "plane1_"+std::to_string(i), v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  colors(i,0), colors(i,1),colors(i,2), "plane1_"+std::to_string(i), v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  3,"plane1_"+std::to_string(i), v2);
        viewer->addPointCloud<pcl::PointXYZ> (planeVect1[corespPlane(i,1)].getFeatureCloud(),
                "plane2_"+std::to_string(i), v2);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  colors(i,0), colors(i,1),colors(i,2), "plane2_"+std::to_string(i), v1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  3,"plane2_"+std::to_string(i), v2);
    }
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
}


Display::~Display()
{}
