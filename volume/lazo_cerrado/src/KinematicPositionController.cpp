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

  std::string goal_selection = this->declare_parameter("goal_selection", "PURSUIT_BASED");
  fixed_goal_x_ = this->declare_parameter("fixed_goal_x", 3.0);
  fixed_goal_y_ = this->declare_parameter("fixed_goal_y", 0.0);
  fixed_goal_a_ = this->declare_parameter("fixed_goal_a", -M_PI_2);

  if (goal_selection == "PURSUIT_BASED")
    goal_selection_ = PURSUIT_BASED;
  else if (goal_selection == "FIXED_GOAL")
    goal_selection_ = FIXED_GOAL;
  else
    goal_selection_ = PURSUIT_BASED; // default
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

/**
 * NOTA: Para un sistema estable mantener:
 * - 0 < K_RHO
 * - K_RHO < K_ALPHA
 * - K_BETA < 0
 */

#define K_X 0.3
#define K_Y 1.5
#define K_THETA -0.3

bool KinematicPositionController::control(
    const rclcpp::Time &t,
    double &vx,
    double &vy,
    double &w)
{
  // Pose actual (odometría)
  double current_x = this->x;
  double current_y = this->y;
  double current_a = this->a;

  // Pose objetivo
  double goal_x, goal_y, goal_a;
  if (!getCurrentGoal(t, goal_x, goal_y, goal_a))
    return false;

  publishCurrentGoal(t, goal_x, goal_y, goal_a);

  // Error en el marco inercial
  double dx = goal_x - current_x;
  double dy = goal_y - current_y;

  // Error angular
  double dtheta = angles::normalize_angle(goal_a - current_a);

  // Controlador P holonómico (desacoplado)
  vx = K_X * dx;
  vy = K_Y * dy;
  w  = K_THETA * dtheta;

  RCLCPP_INFO(this->get_logger(), "vx: %.2f, vy: %.2f, w: %.2f", vx, vy, w);
  
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
  current_x = this->x;
  current_y = this->y;
  current_a = this->a;

  // Se obtiene la trayectoria requerida.
  const robmovil_msgs::msg::Trajectory &trajectory = getTrajectory();

  /** 
   * Se recomienda encontrar el waypoint de la trayectoria más cercano al robot en términos de x,y
   * y luego buscar el primer waypoint que se encuentre a una distancia predefinida de lookahead en x,y */

  /* NOTA: De esta manera les es posible recorrer la trayectoria requerida */

  double lookahead = 0.5;           
  double dist_min = pow(2.0, 63.0); 
  int indice_punto_mas_cercano = 0;

  // buscar el punto mas cercano al robot
  for (unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    // Recorren cada waypoint definido
    const robmovil_msgs::msg::TrajectoryPoint &wpoint = trajectory.points[i];

    // Y de esta manera puede acceder a la informacion de la posicion y orientacion requerida en el waypoint
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;
    double wpoint_a = tf2::getYaw(wpoint.transform.rotation);

    double distancia = dist2(current_x, current_y, wpoint_x, wpoint_y);

    if (distancia < dist_min)
    {
      dist_min = distancia;
      indice_punto_mas_cercano = i;
    }
  }

  int indice_lookahead = 0;
  // paso 3 encontrar un punto goal que este <lookahead> delante del robot
  // miramos desde el indice mas cercano al robot para adelante
  for (unsigned int i = indice_punto_mas_cercano; i < trajectory.points.size(); i++)
  {
    // Recorren cada waypoint definido
    const robmovil_msgs::msg::TrajectoryPoint &wpoint = trajectory.points[i];

    // Y de esta manera puede acceder a la informacion de la posicion y orientacion requerida en el waypoint
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;
    double wpoint_a = tf2::getYaw(wpoint.transform.rotation);

    double distancia = dist2(current_x, current_y, wpoint_x, wpoint_y);

    if (distancia > lookahead)
    {
      indice_lookahead = i;
      break;
    }
    // si no hay ninguno le doy el ultimo?
    indice_lookahead = i;
  }

  x = trajectory.points[indice_lookahead].transform.translation.x;
  y = trajectory.points[indice_lookahead].transform.translation.y;
  a = tf2::getYaw(trajectory.points[indice_lookahead].transform.rotation);

  const robmovil_msgs::msg::TrajectoryPoint &last_wpoint = trajectory.points.back();
  int tolerancia = 0.5;
  if (dist2(current_x, current_y, last_wpoint.transform.translation.x, last_wpoint.transform.translation.y) < 0.5)
  {
    return false;
  }

  /* retorna true si es posible definir un goal, false si se termino la trayectoria y no quedan goals. */
  return true;
}