#ifndef SCANPROVIDER_H
#define SCANPROVIDER_H

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <math.h>
#include "scan.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

/*!
 * \file scanprovider.h
 * \brief A scan provider for point cloud stored in .pcd files
 * \author César Debeunne
 */

class ScanProvider
{
public:
    ScanProvider(std::string path, int startIdx=0, int endIdx=-1);

    /*!
    *  \brief accessor to the current index
    *  \return featureCloud
    */
    int getIdx() const;

    ~ScanProvider();

protected:
    /*!
    *  \brief store the next scan path
    *  \return false if this is the last scan, true otherwise
    */
    bool fetchNextScan();

    std::string _folder;
    std::string _scanPath;
    int _idx; /*!< current index */
    int _endIdx;
};

class KittiScanProvider: public ScanProvider
{
public:
    KittiScanProvider(std::string path, int startIdx=0, int endIdx=-1);
    ~KittiScanProvider();

    /*!
    *  \brief gives a pointer to the next scan
    *  \return scan
    */
    std::shared_ptr<ScanHDL64> next();

    /*!
    *  \brief compute the velocity vectors of each scans thanks to
    * the groundtruth text files
    *  \param the path of grountruth folder
    *  \return an array of the velocity vectors
    */
    std::vector<Eigen::VectorXd> getGrountruth(std::string path);
};

class VLP16ScanProvider: public ScanProvider
{
public:
    VLP16ScanProvider(std::string path, int startIdx=0, int endIdx=-1);
    ~VLP16ScanProvider();

    /*!
    *  \brief gives a pointer to the next scan
    *  \return scan
    */
    std::shared_ptr<ScanVLP16> next();
};

#endif // SCANPROVIDER_H
