#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <usv_msgs/SpeedCourse.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/LU>
#include <cmath>

class Controller
{
public:
  Controller();

private:
  void speedCourseCallback(const usv_msgs::SpeedCourse& speed_course);
  Eigen::Vector2d thrustAllocation(Eigen::Vector3d tau_d);
  double calculateSurgeForce(double deltaTime, double u_d);
  double calculateYawMoment(double deltaTime, double psi_d);

  void odometryCallback(const nav_msgs::Odometry& odometry);


  ros::Subscriber m_speedCourseSub;
  ros::Subscriber m_odometrySub;

  ros::Publisher m_leftPub;
  ros::Publisher m_rightPub;

  // Thruster configuration matrix
  Eigen::MatrixXd T;

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
  double Kp_psi = 0.5;
  double Ki_psi = 1.0;
  double Kd_psi = 4;
  double mass_psi = 10.0 - 1.0; // Iz - nDotR
  double damp_psi = 20.0; // nR
};

#endif // CONTROLLER_H
