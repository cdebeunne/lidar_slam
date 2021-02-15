#ifndef LOAMFEATUREDETECTOR_H
#define LOAMFEATUREDETECTOR_H

#include "featuredetector.h"
#include "utility.h"


class LOAMFeatureDetector: public FeatureDetector
{
public:
  LOAMFeatureDetector(){}
  ~LOAMFeatureDetector(){}
  virtual void detect(const std::shared_ptr<Scan> &scan, std::vector<Edge> &edgeVect, std::vector<Plane> &planeVect)=0;

protected:
  void projectPointCloud();

  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudIn;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _orgCloud;

  Eigen::MatrixXf _range_mat;   // range matrix for range image
  Eigen::MatrixXi _label_mat;   // label matrix for segmentaiton marking
};



#endif // LOAMFEATUREDETECTOR_H
