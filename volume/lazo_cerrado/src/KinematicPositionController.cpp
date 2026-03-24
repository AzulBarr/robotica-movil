#include <angles/angles.h>
#include "KinematicPositionController.h"
#include <cmath>

KinematicPositionController::KinematicPositionController() : TrajectoryFollower(), tfBuffer_(this->get_clock()), transform_listener_(tfBuffer_)
{
  rclcpp::QoS qos_profile(rclcpp::KeepLast(50));
  qos_profile.reliable();
  qos_profile.durability_volatile();

  expected_position_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(10));

  current_pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10), std::bind(&KinematicPositionController::getCurrentPoseFromOdometry, this, std::placeholders::_1));

  std::string goal_selection = this->declare_parameter("goal_selection", "TIME_BASED");
  fixed_goal_x_ = this->declare_parameter("fixed_goal_x", 3.0);
  fixed_goal_y_ = this->declare_parameter("fixed_goal_y", 0.0);
  fixed_goal_a_ = this->declare_parameter("fixed_goal_a", -M_PI_2);

  if (goal_selection == "TIME_BASED")
    goal_selection_ = TIME_BASED;
  else if (goal_selection == "PURSUIT_BASED")
    goal_selection_ = PURSUIT_BASED;
  else if (goal_selection == "FIXED_GOAL")
    goal_selection_ = FIXED_GOAL;
  else
    goal_selection_ = TIME_BASED; // default
}

double lineal_interp(const rclcpp::Time &t0, const rclcpp::Time &t1, double y0, double y1, const rclcpp::Time &t)
{
  return y0 + (t - t0).seconds() * (y1 - y0) / (t1 - t0).seconds();
}

void KinematicPositionController::getCurrentPoseFromOdometry(const nav_msgs::msg::Odometry &odometry_msg)
{
  x = odometry_msg.pose.pose.position.x;
  y = odometry_msg.pose.pose.position.y;
  tf2::Quaternion q(odometry_msg.pose.pose.orientation.x,
                    odometry_msg.pose.pose.orientation.y,
                    odometry_msg.pose.pose.orientation.z,
                    odometry_msg.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  a = yaw;
}

// ORIGINALES
// #define K_RHO 1
// #define K_ALPHA 1.5
// #define K_BETA -0.5

#define KX 1
#define KY 1
#define KTH 1

bool KinematicPositionController::control(const rclcpp::Time &t, double &vx, double& vy, double &w)
{
  // Se obtiene la pose actual publicada por la odometria
  double current_x, current_y, current_a;
  current_x = this->x;
  current_y = this->y;
  current_a = this->a;

  //RCLCPP_INFO(this->get_logger(), "cx: %.3f cy: %.3f ca: %.3f", current_x, current_y, current_a);

  // Se obtiene la pose objetivo actual a seguir
  double goal_x, goal_y, goal_a;
  if (not getCurrentGoal(t, goal_x, goal_y, goal_a))
    return false;
  publishCurrentGoal(t, goal_x, goal_y, goal_a); // publicación de la pose objetivo para visualizar en RViz

  double dx = goal_x - current_x;
  double dy = goal_y - current_y;
  double theta = goal_a - current_a;  

  while(theta > M_PI) theta -= 2*M_PI;
  while(theta < -M_PI) theta += 2*M_PI;

  //RCLCPP_INFO(this->get_logger(), "atan2: %.2f, theta siegwart: %.2f, expected_atheta: %.2f, rho: %.2f, alpha: %.2f, beta: %.2f, v: %.2f, w: %.2f",
  //            atan2(dy, dx), theta, current_a, dx, dy, theta, dx, w);

  //RCLCPP_INFO(this->get_logger(), "goal_x: %.2f, goal_y: %.2f, goal_a: %.2f, current_x: %.2f, current_y: %.2f, current_a: %.2f",
  //            goal_x, goal_y, goal_a, current_x, current_y, current_a);

  double vx_map = KX * dx;
  double vy_map = KY * dy;
  w = KTH * theta;

  vx = vx_map * cos(current_a) + vy_map * sin(current_a);
  vy = -vx_map * sin(current_a) + vy_map * cos(current_a);

  //vx = 0;
  //vy = 0.5;
  //w = 0;

  //RCLCPP_WARN(this->get_logger(), "vx: %.3f vy: %.3f w: %.3f", vx, vy, w);

  return true;
}

/* Funcion auxiliar para calcular la distancia euclidea */
double dist2(double x0, double y0, double x1, double y1)
{
  return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

bool KinematicPositionController::getPursuitBasedGoal(const rclcpp::Time &t, double &x, double &y, double &a)
{
  // Los obtienen los valores de la posicion y orientacion actual.
  double current_x, current_y, current_a;
  current_x = this->x; current_y = this->y; current_a = this->a;
    
  // Se obtiene la trayectoria requerida.
  const robmovil_msgs::msg::Trajectory& trajectory = getTrajectory();

  double lookahead = 0.05;

  unsigned int wp_it = 0;

  for(unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    // Recorren cada waypoint definido
    const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[i];
    
    // Y de esta manera puede acceder a la informacion de la posicion y orientacion requerida en el waypoint
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;
    double wpoint_a = tf2::getYaw(wpoint.transform.rotation);
    
    if(dist2(current_x, current_y, wpoint_x, wpoint_y) < lookahead) {
      wp_it = i;
      break;
    }
    
  }

  for(unsigned int i = wp_it; i < trajectory.points.size(); i++) {
    const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[i];
    if(dist2(current_x, current_y, wpoint.transform.translation.x, wpoint.transform.translation.y) > lookahead) {
      break;
    }
    wp_it = i;
  }

  const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[wp_it];
  x = wpoint.transform.translation.x; 
  y = wpoint.transform.translation.y; 
  a = tf2::getYaw(wpoint.transform.rotation);

  const robmovil_msgs::msg::TrajectoryPoint& last_wpoint = trajectory.points.back(); 

  RCLCPP_INFO(this->get_logger(), "cx: %.3f cy: %.3f gx: %.3f gy: %.3f cercania: %.3f", current_x, current_y, x, y,
dist2(current_x, current_y, last_wpoint.transform.translation.x, last_wpoint.transform.translation.y));
  
  
  /* retorna true si es posible definir un goal, false si se termino la trayectoria y no quedan goals. */
  if(wp_it == trajectory.points.size() && dist2(current_x, current_y, last_wpoint.transform.translation.x, last_wpoint.transform.translation.y) < lookahead) {
    return false;
  }
  return true;
}

bool KinematicPositionController::getTimeBasedGoal(const rclcpp::Time &t, double &x, double &y, double &a)
{
  size_t next_point_idx;

  if (not nextPointIndex(t, next_point_idx))
    return false;

  RCLCPP_INFO(this->get_logger(), "processing index: %zu", next_point_idx);

  const robmovil_msgs::msg::TrajectoryPoint &prev_point = getTrajectory().points[next_point_idx - 1];
  const robmovil_msgs::msg::TrajectoryPoint &next_point = getTrajectory().points[next_point_idx];

  const rclcpp::Time &t0 = getInitialTime() + prev_point.time_from_start;
  const rclcpp::Time &t1 = getInitialTime() + next_point.time_from_start;

  assert(t0 <= t);
  assert(t < t1);

  double x0 = prev_point.transform.translation.x;
  double x1 = next_point.transform.translation.x;

  double y0 = prev_point.transform.translation.y;
  double y1 = next_point.transform.translation.y;

  double a0 = tf2::getYaw(prev_point.transform.rotation);
  double a1 = tf2::getYaw(next_point.transform.rotation);

  x = lineal_interp(t0, t1, x0, x1, t);
  y = lineal_interp(t0, t1, y0, y1, t);
  a = lineal_interp(t0, t1, a0, a1, t);

  return true;
}

