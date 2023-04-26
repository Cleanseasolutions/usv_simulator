#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <usv_msgs/SpeedCourse.h>
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

    // Speed controller
  double Kp_u = 2.0;
  double Ki_u = 1.0;
  double mass_u = 29 - 5.0; // m - xDotU: 29 - 5
  double damp_u = 20.0; // xU

  // Heading controller
  double Kp_psi = 4;
  double Ki_psi = 1.0;
  double Kd_psi = 4;
  double mass_psi = 10.0 - 1.0; // Iz - nDotR
  double damp_psi = 20.0; // nR

// lookahead distance
  double DELTA = 0.5;

  // time-varying lookahead distance
  double delta_max = 4.0;
  double delta_min = 1.0;
  double delta_k = 1.0;

  // circle of acceptance
  double R = 1.0;

  double m_maxSpeed;
  double m_maxSpeedTurn;
  double m_minSpeed;
};