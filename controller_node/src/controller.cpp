#include "controller.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Controller::Controller() : T(3, 2)
{
  ros::NodeHandle nh;

  m_speedCourseSub = nh.subscribe("speed_course", 1000, &Controller::speedCourseCallback, this);
  m_odometrySub = nh.subscribe("p3d", 1000, &Controller::odometryCallback, this);


  m_leftPub = nh.advertise<std_msgs::Float32>("left_thrust_cmd", 10);
  m_rightPub = nh.advertise<std_msgs::Float32>("right_thrust_cmd", 10);

  // Initialize thruster configuration matrix
  T << 50, 50, 0, 0, -0.39 * 50, 0.39 * 50;
}

double Controller::calculateSurgeForce(double deltaTime, double u_d)
{
  // Implement the surge force calculation similar to the provided code
  static double integralTerm = 0.0;

  double u_d_dot = 0.0;
  double u_tilde = u_d - u;

  // integralTerm += u_tilde * deltaTime;

  return mass_u * (u_d_dot - Kp_u * u_tilde - Ki_u* integralTerm);
}

double Controller::calculateYawMoment(double deltaTime, double psi_d)
{
  // Implement the yaw moment calculation similar to the provided code
  static double integralTerm = 0.0;

  double psi_d_dot = 0.0;
  double r_tilde = psi_d - psi;

  // integralTerm += r_tilde * deltaTime;

  return mass_psi * (psi_d_dot - Kp_psi * r_tilde - Ki_psi * integralTerm);
}

Eigen::Vector2d Controller::thrustAllocation(Eigen::Vector3d tau_d)
{
  // Initialize thruster configuration matrix pseudoinverse
  static bool initialized = false;
  static Eigen::MatrixXd pinv(3, 2);
  if (!initialized) {
    initialized = true;
    pinv = T.completeOrthogonalDecomposition().pseudoInverse();
  }

  // Calculate thruster output
  Eigen::Vector2d u = pinv * tau_d;

  // Ensure in interval [-1, 1]
  u[0] = std::min(std::max(u[0], -1.0), 1.0);
  u[1] = std::min(std::max(u[1], -1.0), 1.0);

  return u;
}

void Controller::odometryCallback(const nav_msgs::Odometry& odometry)
{
  u = odometry.twist.twist.linear.x;
  
  tf2::Quaternion q;
  tf2::fromMsg(odometry.pose.pose.orientation, q);
  double roll, pitch;
  tf2::Matrix3x3(q).getRPY(roll, pitch, psi);

  r = odometry.twist.twist.angular.z;
}



void Controller::speedCourseCallback(const usv_msgs::SpeedCourse& speed_course)
{
  // Implement the controller logic
  // TODO: Customize the logic for the specific speed_course message

  // Calculate the surge force and yaw moment
  double deltaTime = 0.1; // TODO: Use actual delta time
  double fx = calculateSurgeForce(deltaTime, speed_course.speed);
  double mz = calculateYawMoment(deltaTime, speed_course.course); // TODO: Use actual r value

  // Calculate thrust allocation
  Eigen::Vector3d tau_d(fx, 0, mz);
  Eigen::Vector2d u = thrustAllocation(tau_d);

  // Publish left and right thrust commands
  std_msgs::Float32 left_msg;
  std_msgs::Float32 right_msg;
  left_msg.data = u[0];
  right_msg.data = u[1];
  m_leftPub.publish(left_msg);
  m_rightPub.publish(right_msg);
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "controller_node");
  Controller controller;

  ros::spin();

  return 0;
}
