#ifndef SCAN_H
#define SCAN_H

#include "utility.h"
#include "feature.h"

/*!
 * \file scan.h
 * \brief a global class to assembly every scan properties
 * \author César Debeunne
 */


class Scan
{
public:
    Scan(pcl::PointCloud<pcl::PointXYZ>::Ptr i_cloud);
    ~Scan(){}

    /*!
    *  \brief accessor to the original point cloud
    *  \return point Cloud
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() const{return _cloud;}

    /*!
    *  \brief accessor to the pose vector
    *  \return pose
    */
    Eigen::VectorXd getPose() const{return _pose;}

    /*!
    *  \brief set the pose of the scan
    *  \param i_pose
    */
    void setPose(Eigen::VectorXd i_pose){_pose=i_pose;}

    /*!
    *  \brief set the edge vector of the frame
    *  \param edge vector
    */
    void setEdgeVect(const std::vector<Edge> e) {_edgeVect = e;}
    
    /*!
    *  \brief accessor to the edge vector
    *  \return edge vector
    */
    std::vector<Edge> getEdgeVect() const {return _edgeVect;}

    /*!
    *  \brief set the plane vector of the frame
    *  \param plane vector
    */
    void setPlaneVect(const std::vector<Plane> p) {_planeVect = p;}
    
    /*!
    *  \brief accessor to the plane vector
    *  \return plane vector
    */
    std::vector<Plane> getPlaneVect() const {return _planeVect;}

    int getWidth(){return WIDTH;}
    int getLength(){return LENGTH;}

protected:
    int LENGTH;
    int WIDTH;
    std::vector<Edge> _edgeVect;
    std::vector<Plane> _planeVect;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    Eigen::VectorXd _pose;
};

class ScanVLP16: public Scan
{
public:
    ScanVLP16(pcl::PointCloud<pcl::PointXYZ>::Ptr i_cloud);
    ~ScanVLP16(){}
};

class ScanHDL64: public Scan
{
public:
    ScanHDL64(pcl::PointCloud<pcl::PointXYZ>::Ptr i_cloud);
    ~ScanHDL64(){}
};


#endif // SCAN_H
