#include "plot.h"

Eigen::Matrix3d eul2rotm(double theta, double phi, double psi)
{
    Eigen::Matrix3d rotx;
    rotx << 1,0,0,
            0,cos(theta),-sin(theta),
            0,sin(theta),cos(theta);
    Eigen::Matrix3d roty;
    roty << cos(phi),0,sin(phi),
            0,1,0,
            -sin(phi),0,cos(phi);
    Eigen::Matrix3d rotz;
    rotz << cos(psi), -sin(psi),0,
            sin(psi),cos(psi),0,
            0,0,1;
    return rotx*roty*rotz;
}

Plot::Plot(ros::NodeHandle nh): _nh(nh)
{
  // initializing the publisher
  _pubPath = _nh.advertise<nav_msgs::Path>     ("lidar_slam/path", 1);
  _pubOdom = _nh.advertise<nav_msgs::Odometry> ("lidar_slam/odometry", 1);
}


void Plot::publishGroundtruth(std::vector<Eigen::VectorXd> velVect){
    Eigen::Vector3d pose(0,0,0);
    Eigen::Vector3d att(0,0,0);
    usleep(2e6);

    for (std::vector<Eigen::VectorXd>::iterator it = velVect.begin() ; it != velVect.end(); ++it){
      static nav_msgs::Path pathMsg;
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = pose(0);
      pose_stamped.pose.position.y = -pose(1);
      pose_stamped.pose.position.z = pose(2);
      pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(att(0),att(1),att(2));
      pathMsg.poses.push_back(pose_stamped);
      pathMsg.header.stamp = ros::Time::now();
      pathMsg.header.frame_id = "map";
      _pubPath.publish(pathMsg);
      att = att + it->tail(3)*0.1;
      pose = pose + eul2rotm(att(0), att(1), att(2))*it->head(3)*0.1;
    }
}

void Plot::publishOdometry(Eigen::VectorXd pose){
  double x, y, z, roll, pitch, yaw;
  x = pose(0);
  y = pose(1);
  z = pose(2);
  roll = pose(3);
  pitch = pose(4);
  yaw = pose(5);

  // tf to update the robot frame
  static tf::TransformBroadcaster tfMap2Base;
  tf::Transform map_to_robot = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
  tfMap2Base.sendTransform(tf::StampedTransform(map_to_robot, ros::Time::now(), "map", "robot"));

  // publish the odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "map";
  odom.child_frame_id = "robot";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  _pubOdom.publish(odom);
  usleep(1e5);
}
