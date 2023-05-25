#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <usv_msgs/SpeedCourse.msg>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>


class USVController {
public:
  USVController(ros::NodeHandle& nh);
  void speedCourseCallback(const usv_msgs::SpeedCourse& msg);
  void poseTwistCallback(const nav_msgs::Odometry& msg);
  void controlLoop();

private:
  ros::Subscriber m_speedCourseSub;
  ros::Subscriber m_poseTwistSub;
  ros::Publisher m_leftPub;
  ros::Publisher m_rightPub;

  double m_desiredSpeed;
  double m_desiredCourse;

  double m_currentSpeed;
  double m_currentYaw;

  Eigen::MatrixXd m_thrusterConfigMatrix;

  // Integral error accumulators
  double m_speedIntegralError;
  double m_courseIntegralError;

  // Integral gains
  double m_Ki_speed;
  double m_Ki_course;
  
  // Derivative
  double m_prevCourseError;
  double Kd_course;

  double Kp_speed;  // Proportional gain for speed
  double Kp_course; // Proportional gain for course

  double dt; // Time step for the control loop (1 / loop_rate in the main function)

};

#endif // CONTROLLER_H
