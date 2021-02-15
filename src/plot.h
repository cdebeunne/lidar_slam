#ifndef PLOT_H
#define PLOT_H

#include "utility.h"

/*!
 * \file plot.h
 * \brief 3D vizualization of the robot trajectory
 * \author César Debeunne
 */

class Plot
{
public:
    Plot(ros::NodeHandle nh);
    ~Plot(){}

    /*!
    *  \brief add a new point in the view
    *  \param pt
    */
    void addPoint(Eigen::Vector3d pt){_traj.push_back(pt);}

    /*!
    *  \brief plot the groundtruth with a set of velocity vectors
    *  \param velVect
    */
    void publishGroundtruth(std::vector<Eigen::VectorXd> velVect);
    void publishOdometry(Eigen::VectorXd pose);

private:
    std::vector<Eigen::Vector3d> _traj;

    ros::NodeHandle _nh;
    ros::Publisher _pubPath;
    ros::Publisher _pubOdom;
};

#endif // PLOT_H
