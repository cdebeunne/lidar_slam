#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <math.h>
#include <ctime>
#include <ros/ros.h>

#include <Eigen/Dense>

#include "scan.h"
#include "feature.h"
#include "scanprovider.h"
#include "display.h"
#include "matching.h"
#include "transformationsolver.h"
#include "plot.h"
#include "ransac.h"
#include "featuredetector.h"
#include <thread>
#include <algorithm>

#include <pcl/io/pcd_io.h>


int main (int argc, char** argv)
{
    ros::init(argc, argv, "lidar_slam");
    ros::NodeHandle nh;

    KittiScanProvider prov("/media/debeunne/DATA/Stage DEOS/pcdFiles/KITTI_3/message",3, 268);
    std::vector<Eigen::VectorXd> gt = prov.getGrountruth("/media/debeunne/DATA/Stage DEOS/pcdFiles/KITTI_3_GT/data");

    // Initialization of the tools
    HDL64CESARDetector detector;
    Matching matcher;
    TransformationSolver solver;

    // Scan providing
    std::shared_ptr<ScanHDL64> theScan = prov.next();
    std::shared_ptr<ScanHDL64> theScan1 = prov.next();

    // Initialization of the variable
    Eigen::MatrixXi corespEdge;
    Eigen::MatrixXi corespPlane;
    Eigen::VectorXd transVec;
    std::vector<Edge> edgeVect, edgeVect1;
    std::vector<Plane> planeVect, planeVect1;

    // Detection
    detector.detect(theScan, edgeVect, planeVect);
    theScan->setEdgeVect(edgeVect);
    theScan->setPlaneVect(planeVect);

    Plot plot(nh);
    plot.publishGroundtruth(gt);

//    Display disp;
//    disp.dispColorCloud(detector.colorComputation());
//    disp.dispPlaneVect(theScan->getPlaneVect());
//    disp.dispMatchedPlanes(corespPlane, theScan->getPlaneVect(), theScan1->getPlaneVect());
//    disp.dispEdgeVect(theScan1->getEdgeVect());
//    disp.dispMatchedEdges(corespEdge, theScan->getEdgeVect(), theScan1->getEdgeVect());
//    disp.dispPointCloud(theScan->getCloud());

    while (theScan1 != nullptr){
        std::cout << "--------- " << prov.getIdx() << " ---------" << std::endl;

        // feature detection
        detector.detect(theScan1, edgeVect1, planeVect1);
        theScan1->setEdgeVect(edgeVect1);
        theScan1->setPlaneVect(planeVect1);

        // collect and match feature data
        corespEdge = matcher.edgeMatching(edgeVect, edgeVect1);
        RANSAC ransacFiltere(std::round(0.7*corespEdge.rows()), 4, 0.3, 0.08);
        corespEdge = ransacFiltere.ransacFilterEdge(theScan, theScan1, corespEdge, &transVec);

        // set the new scan
        solver.applyTransformation(transVec, theScan, theScan1);
        theScan = theScan1;
        edgeVect = edgeVect1;
        planeVect = planeVect1;
        theScan1 = prov.next();

        // plot the line
        plot.publishOdometry(theScan->getPose());

        std::cout << theScan->getPose() << std::endl;

    }

    return 0;
}
