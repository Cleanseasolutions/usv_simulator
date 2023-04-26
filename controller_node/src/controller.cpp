#include "controller.h"

USVController::USVController(ros::NodeHandle& nh) {
  m_speedCourseSub = nh.subscribe("speed_course", 10, &USVController::speedCourseCallback, this);
  m_poseTwistSub = nh.subscribe("p3d", 10, &USVController::poseTwistCallback, this);
  m_leftPub = nh.advertise<std_msgs::Float32>("left_thrust_cmd", 10);
  m_rightPub = nh.advertise<std_msgs::Float32>("right_thrust_cmd", 10);

  m_desiredSpeed = 0.0;
  m_desiredCourse = 0.0;
  m_currentSpeed = 0.0;
  m_currentYaw = 0.0;

  m_thrusterConfigMatrix = Eigen::MatrixXd::Zero(3, 2);
  m_thrusterConfigMatrix << 50, 50, 0, 0, -0.39 * 50, 0.39 * 50;

  // Initialize integral error accumulators
  m_speedIntegralError = 0.0;
  m_courseIntegralError = 0.0;

  // Initialize integral gains
  m_Ki_speed = 0.1;
  m_Ki_course = 0.1;
}

void USVController::speedCourseCallback(const usv_msgs::SpeedCourse& msg) {
  m_desiredSpeed = msg.speed;
  m_desiredCourse = msg.course;
}

void USVController::poseTwistCallback(const nav_msgs::Odometry& msg) {
  m_currentSpeed = msg.twist.twist.linear.x;
  m_currentYaw = tf::getYaw(msg.pose.pose.orientation);
}

void USVController::controlLoop() {
  double Kp_speed = 5.0;  // Proportional gain for speed
  double Kp_course = 4.0; // Proportional gain for course

  double dt = 0.1; // Time step for the control loop (1 / loop_rate in the main function)


  double speed_error = m_desiredSpeed - m_currentSpeed;
  double course_error = m_desiredCourse - m_currentYaw;

  // Wrap course_error to [-pi, pi]
  while (course_error > M_PI) course_error -= 2 * M_PI;
  while (course_error < -M_PI) course_error += 2 * M_PI;

  // Print speed and heading error
  ROS_INFO("Speed error(m/s): %f, Heading error(deg): %f", speed_error, course_error * 180.0 / M_PI);

  // Update integral error accumulators
  m_speedIntegralError += speed_error * dt;
  m_courseIntegralError += course_error * dt;

  // Calculate control inputs using proportional and integral terms
  double u_speed = Kp_speed * speed_error + m_Ki_speed * m_speedIntegralError;
  double u_course = Kp_course * course_error + m_Ki_course * m_courseIntegralError;

  Eigen::VectorXd control_input(3);
  control_input << u_speed, 0, u_course;

  static bool initialized = false;
  static Eigen::MatrixXd pinv(3, 2);
  if (!initialized) {
  initialized = true;
  pinv = m_thrusterConfigMatrix.completeOrthogonalDecomposition().pseudoInverse();

  // Print thruster configuration matrix initialization
  ROS_INFO("Thruster configuration matrix initialized");
  }

  Eigen::VectorXd thruster_output = pinv * control_input;

  std_msgs::Float32 left_thrust_msg;
  std_msgs::Float32 right_thrust_msg;

  left_thrust_msg.data = thruster_output(0);
  right_thrust_msg.data = thruster_output(1);

  m_leftPub.publish(left_thrust_msg);
  m_rightPub.publish(right_thrust_msg);
  }

int main(int argc, char** argv) {
  ros::init(argc, argv, "usv_controller");
  ros::NodeHandle nh;

  USVController controller(nh);

  ros::Rate loop_rate(10); // Control loop rate (Hz)

  while (ros::ok()) {
    controller.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
