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
  m_Ki_speed = 5; //5;
  m_Ki_course = 0.0; //0.1;

  Kp_speed = 70.0;  // Proportional gain for speed
  Kp_course = 20.0; // Proportional gain for course

  Kd_course = 1;

  dt = 0.1; // Time step for the control loop (1 / loop_rate in the main function)
}

void USVController::speedCourseCallback(const usv_msgs::SpeedCourse& msg) {
  m_desiredSpeed = msg.speed;
  m_desiredCourse = msg.course;
}

void USVController::poseTwistCallback(const nav_msgs::Odometry& msg) {
  double speed_x_world = msg.twist.twist.linear.x;
  double speed_y_world = msg.twist.twist.linear.y;
  m_currentYaw = tf::getYaw(msg.pose.pose.orientation);

  // Project the speed onto the boat's heading
  m_currentSpeed = speed_x_world * cos(m_currentYaw) + speed_y_world * sin(m_currentYaw);

  ROS_INFO("Speed (m/s): %f", m_currentSpeed);
}


void USVController::controlLoop() {
  double speed_x_world = m_currentSpeed * cos(m_currentYaw);
  double speed_y_world = m_currentSpeed * sin(m_currentYaw);

  // This is speed in the desired heading
  double speed_desired_heading = speed_x_world * cos(m_desiredCourse) + speed_y_world * sin(m_desiredCourse);
  
  // Calculate error
  double speed_error = m_desiredSpeed - m_currentSpeed;
  double course_error = m_desiredCourse - m_currentYaw;

  // Wrap course_error to [-pi, pi]
  while (course_error > M_PI) course_error -= 2 * M_PI;
  while (course_error < -M_PI) course_error += 2 * M_PI;

  // Calculate derivative
  double course_derivative = (course_error - m_prevCourseError) / 1; //measurement at 1Hz
  m_prevCourseError = course_error;
  // Print speed and heading error
  ROS_INFO("Speed error(m/s): %f, Heading error(deg): %f", speed_error, course_error * 180.0 / M_PI);

  // Update integral error accumulators
  m_speedIntegralError += speed_error * dt;
  m_courseIntegralError += course_error * dt;

  // Calculate control inputs using proportional and integral terms
  double u_speed = Kp_speed * speed_error + m_Ki_speed * m_speedIntegralError;
  double u_course = Kp_course * course_error + Kd_course * course_derivative + m_Ki_course * m_courseIntegralError; // PID

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
