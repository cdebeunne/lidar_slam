#include "scanprovider.h"

ScanProvider::ScanProvider(std::string path, int startIdx, int endIdx):_folder(path),_idx(startIdx),_endIdx(endIdx)
{}

bool ScanProvider::fetchNextScan()
{
    if (_idx == _endIdx){
        return false;
    }
    _scanPath = _folder+std::to_string(_idx)+".pcd";
    _idx++;
    return true;
}

int ScanProvider::getIdx() const{
    return _idx;
}

ScanProvider::~ScanProvider()
{}

KittiScanProvider::KittiScanProvider(std::string path, int startIdx, int endIdx):
    ScanProvider(path, startIdx, endIdx)
{}

KittiScanProvider::~KittiScanProvider()
{}

std::shared_ptr<ScanHDL64> KittiScanProvider::next()
{
    if(!fetchNextScan()){
        return nullptr;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (_scanPath, *cloud_i);
    std::shared_ptr<ScanHDL64> scan = std::shared_ptr<ScanHDL64>(new ScanHDL64(cloud_i));
    return scan;
}

std::vector<Eigen::VectorXd> KittiScanProvider::getGrountruth(std::string gtPath)
{
    bool in = true;
    int i = 0;
    std::vector<Eigen::VectorXd> velVect;
    while (in){
        std::string txtPath;
        i++;
        int no = log10(i);
        int numberOfZeros = 10-no;
        for (int j = 0; j<numberOfZeros-1; j++){
            txtPath = txtPath + "0";
        }
        txtPath = gtPath + "/" + txtPath + std::to_string(i) + ".txt";
        std::ifstream flow(txtPath);
        if (flow){
            Eigen::VectorXd dx(6);
            double nombre;
            for (int i=0; i<8; i++){
                flow >> nombre;
            }
            for (int i=0; i<3;i++){
                flow >> dx(i);
            }
            for (int i=0;i<9;i++){
                flow >> nombre;
            }
            for (int i=3;i<6;i++){
                flow >>dx(i);
            }
            velVect.push_back(dx);
        } else {
            in = false;
        }
    }
    return velVect;
}

VLP16ScanProvider::VLP16ScanProvider(std::string path, int startIdx, int endIdx):
    ScanProvider(path, startIdx, endIdx)
{}

VLP16ScanProvider::~VLP16ScanProvider()
{}

std::shared_ptr<ScanVLP16> VLP16ScanProvider::next()
{
    if(!fetchNextScan()){
        return nullptr;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (_scanPath, *cloud_i);
    std::shared_ptr<ScanVLP16> scan = std::shared_ptr<ScanVLP16>(new ScanVLP16(cloud_i));
    return scan;
}
