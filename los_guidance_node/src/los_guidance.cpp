#include "los_guidance.h"
#include <usv_msgs/SpeedCourse.h>
// #include <math>


LOSGuidance::LOSGuidance()
{
  ros::NodeHandle nh;

  m_pathSub = nh.subscribe("path", 1000, &LOSGuidance::pathCallback, this);
  m_poseSub = nh.subscribe("p3d", 1000, &LOSGuidance::poseCallback, this);

  m_speedCoursePub = nh.advertise<usv_msgs::SpeedCourse>("speed_course", 10);
}

void LOSGuidance::pathCallback(const nav_msgs::Path& msg)
{
  m_path = msg;
  ROS_INFO("Received path with %zu poses", m_path.poses.size());

  // Debugging: Print received poses
  for (size_t i = 0; i < m_path.poses.size(); ++i)
  {
    ROS_INFO("Pose %zu: x: %f, y: %f", i, m_path.poses[i].pose.position.x, m_path.poses[i].pose.position.y);
  }
}


void LOSGuidance::poseCallback(const nav_msgs::Odometry& pose)
{
  double x = pose.pose.pose.position.x;
  double y = pose.pose.pose.position.y;
  double psi = tf2::getYaw(pose.pose.pose.orientation);

  u = std::sqrt(std::pow(pose.twist.twist.linear.x,2) + std::pow(pose.twist.twist.linear.y,2));
  r = pose.twist.twist.angular.z;

  followPath(x, y, psi);
}

void LOSGuidance::followPath(double x, double y, double psi)
{
  // Check if the path is empty or only has one point
  if (m_path.poses.size() <= 1)
  {
    usv_msgs::SpeedCourse msg;
    msg.speed = 0.0;
    msg.course = psi;
    m_speedCoursePub.publish(msg);
    ROS_INFO("Path empty");
    return;
  }

  // Find the lookahead point
  geometry_msgs::PoseStamped lookahead_pose;
  bool lookahead_found = false;
  for (size_t i = 0; i < m_path.poses.size() - 1; ++i)
  {
    geometry_msgs::PoseStamped pose_a = m_path.poses[i];
    geometry_msgs::PoseStamped pose_b = m_path.poses[i + 1];

    double dx = pose_b.pose.position.x - pose_a.pose.position.x;
    double dy = pose_b.pose.position.y - pose_a.pose.position.y;
    double seg_length = std::sqrt(dx * dx + dy * dy);

    double proj_len = ((x - pose_a.pose.position.x) * dx + (y - pose_a.pose.position.y) * dy) / seg_length;

    if (proj_len >= 0 && proj_len <= seg_length)
    {
      lookahead_pose.pose.position.x = pose_a.pose.position.x + proj_len * dx / seg_length;
      lookahead_pose.pose.position.y = pose_a.pose.position.y + proj_len * dy / seg_length;

      double lookahead_dist = std::sqrt(std::pow(x - lookahead_pose.pose.position.x, 2) +
                                        std::pow(y - lookahead_pose.pose.position.y, 2));

      if (lookahead_dist <= DELTA)
      {
        lookahead_found = true;
        break;
      }
    }
  }

  // If a lookahead point is not found, use the last point in the path
  if (!lookahead_found)
  {
    lookahead_pose = m_path.poses.back();
  }

  // Calculate desired heading
  double dx_lookahead = lookahead_pose.pose.position.x - x;
  double dy_lookahead = lookahead_pose.pose.position.y - y;
  double chi_d = std::atan2(dy_lookahead, dx_lookahead);

  ROS_INFO("Lookahead point: x: %f, y: %f", lookahead_pose.pose.position.x, lookahead_pose.pose.position.y);

  // Calculate heading error
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
  }

  // Calculate desired speed
  double u = m_maxSpeed * (1 - std::abs(chi_err) / M_PI_2);
  u = std::max(u, m_minSpeed);

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