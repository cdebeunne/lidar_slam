#include "loamfeaturedetector.h"

void LOAMFeatureDetector::projectPointCloud(){
  const size_t cloudSize = _cloudIn->points.size();
  for (size_t i = 0; i < cloudSize; ++i) {
    pcl::PointXYZ pt = _cloudIn->points[i];

    float range = sqrt(pt.x * pt.x +
                       pt.y * pt.y +
                       pt.z * pt.z);

    // find the row and column index in the image for this point
    float verticalAngle = std::asin(pt.z / range);
    std::cout << verticalAngle << std::endl;
  }
}
