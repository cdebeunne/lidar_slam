#include "scan.h"
#include "feature.h"
#include "scanprovider.h"
#include "display.h"
#include "matching.h"
#include "transformationsolver.h"
#include "plot.h"
#include "ransac.h"
#include "featuredetector.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

class rosbagReader
{
public:
  rosbagReader(std::string filename){
    _bag.open(filename, rosbag::bagmode::Read);
    _lidarTopic = "/kitti/velo/pointcloud";
    rosbag::View _view(_bag, rosbag::TopicQuery(_lidarTopic));
    _it = _view.begin();
  }

  std::shared_ptr<ScanHDL64> next()
  {
    std::cout << "in next" << std::endl;
    if(_it == _view.end()){
      _bag.close();
      return nullptr;
    }
    const sensor_msgs::PointCloud2ConstPtr &bagCloud = _it->instantiate<sensor_msgs::PointCloud2>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*bagCloud, *cloud_i);
    std::shared_ptr<ScanHDL64> scan = std::shared_ptr<ScanHDL64>(new ScanHDL64(cloud_i));
    std::cout << "before incr" << std::endl;
    _it++;
    std::cout << "after incr" << std::endl;
    return scan;
  }
private:
  rosbag::Bag _bag;
  std::string _lidarTopic;
  rosbag::View _view;
  rosbag::View::iterator _it;
};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "lidar_slam");
  ros::NodeHandle nh;

  KittiScanProvider prov("/media/debeunne/DATA/Stage DEOS/pcdFiles/KITTI_3/message",1, 268);
  std::vector<Eigen::VectorXd> gt = prov.getGrountruth("/media/debeunne/DATA/Stage DEOS/pcdFiles/KITTI_3_GT/data");

  Plot plot(nh);
  plot.publishGroundtruth(gt);

  //  rosbag::Bag bag;
  std::string filename = "/media/debeunne/DATA/Stage DEOS/KITTI Data/KITTI-converter/kitti_2011_09_26_drive_0046_synced.bag";
  //  std::string lidarTopic = "/kitti/velo/pointcloud";
  //  bag.open(filename, rosbag::bagmode::Read);
  //  rosbag::View view(bag, rosbag::TopicQuery(lidarTopic));
  //  rosbag::View::iterator m = view.begin();
  //  ros::Publisher pub;
  //  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  //  while(m!=view.end()){
  //    m++;
  //    const sensor_msgs::PointCloud2ConstPtr &cloud = m->instantiate<sensor_msgs::PointCloud2>();
  //    pub.publish(cloud);
  //    usleep(1e6);
  //  }
  //  bag.close();
  rosbagReader reader(filename);
  std::shared_ptr<ScanHDL64> scan = reader.next();
  std::cout << scan->getWidth() << std::endl;

  return 0;
}
