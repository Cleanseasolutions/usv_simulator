#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <usv_msgs/SpeedCourse.msg>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/LU>
#include <cmath>

class LOSGuidance
{
public:
  LOSGuidance();

private:
  void pathCallback(const nav_msgs::Path& path);
  void poseCallback(const nav_msgs::Odometry& pose);
  void followPath(double x, double y, double psi);

  ros::Subscriber m_pathSub;
  ros::Subscriber m_poseSub;

  ros::Publisher m_speedCoursePub;

  nav_msgs::Path m_path;

  // Desired values
  double u_d = 0.0;
  double psi_d = 0.0;

  // Sensor data
  double u = 0.0;
  double psi = 0.0;
  double r = 0.0;

// lookahead distance
  double DELTA = 6;

  // time-varying lookahead distance
  double delta_max = 4.0;
  double delta_min = 1.0;
  double delta_k = 1.0;

  // circle of acceptance
  double R = 1.0;

  double m_maxSpeed = 1; // m/s
  double m_maxSpeedTurn = 1; //wtf
  double m_minSpeed = 0;
};