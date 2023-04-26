#include "los_guidance.h"


LOSGuidance::LOSGuidance()
{
  ros::NodeHandle nh;

  m_pathSub = nh.subscribe("path", 1000, &LOSGuidance::pathCallback, this);
  m_poseSub = nh.subscribe("p3d", 1000, &LOSGuidance::poseCallback, this);

  m_speedCoursePub = nh.advertise<usv_msgs::SpeedCourse>("speed_course", 10);
}

void LOSGuidance::pathCallback(const nav_msgs::Path& path)
{
  m_path = path;
}

void LOSGuidance::poseCallback(const nav_msgs::Odometry& pose)
{
  double x = pose.pose.pose.position.x;
  double y = pose.pose.pose.position.y;
  double psi = tf2::getYaw(pose.pose.pose.orientation);

  u = pose.twist.twist.linear.x;
  r = pose.twist.twist.angular.z;

  followPath(x, y, psi);
}

void LOSGuidance::followPath(double x, double y, double psi)
{
  // Implement the LOS guidance logic similar to the provided code
  // TODO: Customize the logic for the specific /path and /p3d messages in this implementation

  // Finished?
  if (m_path.poses.size() <= 1)
  {
    usv_msgs::SpeedCourse msg;
    msg.speed = 0.0;
    msg.course = psi;
    m_speedCoursePub.publish(msg);
    return;
  }

  // Identify closest point on path
  std::vector<geometry_msgs::PoseStamped>::iterator closest;
  double minDist = std::numeric_limits<double>::max();
  for (auto it = m_path.poses.begin(); it != m_path.poses.end(); it++)
  {
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
                            std::pow(y - it->pose.position.y, 2));
    if (dist < minDist)
    {
      minDist = dist;
      closest = it;
    }
  }

  // Store closest
  geometry_msgs::PoseStamped pose_d = *closest;

  // Erase previous elements
  m_path.poses.erase(m_path.poses.begin(), closest);

  // Path tangential angle
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);

  // Cross-track error
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) +
               (y - pose_d.pose.position.y) * std::cos(gamma_p);

  // Time-varying lookahead distance
  double delta_y_e =
      (delta_max - delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) +
      delta_min;
  // if turning => small lookahead distance
  bool isTurning = false;
  if ((closest + 1) != m_path.poses.end())
  {
    double nextAngle = tf2::getYaw((*(closest + 1)).pose.orientation);
    if (std::fabs(gamma_p - nextAngle) > std::numeric_limits<double>::epsilon())
    {
      delta_y_e = delta_min;
      isTurning = true;
    }
  }

  // velocity-path relative angle
  double chi_r = std::atan(-y_e / delta_y_e);

  // desired course angle
  double chi_d = gamma_p + chi_r;

  // calculate error in heading
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
  }

  // calculate desired speed
  double u = m_maxSpeed * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
  u = std::max(u, m_minSpeed);
  if (isTurning)
    u = m_maxSpeedTurn;

  // Publish speed and course to controller
  usv_msgs::SpeedCourse msg;
  msg.speed = u;
  msg.course = chi_d;
  m_speedCoursePub.publish(msg);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "losguidance_node");
  LOSGuidance losGuidance;

  ros::spin();

  return 0;
}