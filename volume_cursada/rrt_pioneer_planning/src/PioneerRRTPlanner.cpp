#include "PioneerRRTPlanner.h"

#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>
#include <random>
#include <queue>

typedef robmovil_planning::PioneerRRTPlanner::SpaceConfiguration SpaceConfiguration;

robmovil_planning::PioneerRRTPlanner::PioneerRRTPlanner()
: RRTPlanner(0, 0)
{ 
  goal_bias_ = this->declare_parameter<double>("goal_bias", 0.6);
  int it_tmp = this->declare_parameter<int>("max_iterations", 20000);
  max_iterations_ = it_tmp >= 0 ? it_tmp : 20000;
  Vx_step_ = this->declare_parameter<double>("linear_velocity_stepping", 0.05);
  Wz_step_ = this->declare_parameter<double>("angular_velocity_stepping", 0.025);
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineStartConfig()
{
  /* Se toma la variable global de la pose incial y se la traduce a SpaceConfiguration */
  //return SpaceConfiguration( { starting_pose_.getOrigin().getX(), starting_pose_.getOrigin().getY(), tf::getYaw(starting_pose_.getRotation()) } );
  return SpaceConfiguration( { starting_pose_.pose.position.x, starting_pose_.pose.position.y, tf2::getYaw(starting_pose_.pose.orientation) } );
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineGoalConfig()
{
  /* Se toma la variable global de la pose del goal y se la traduce a SpaceConfiguration */
  //return SpaceConfiguration( { goal_pose_.getOrigin().getX(), goal_pose_.getOrigin().getY(), tf::getYaw(goal_pose_.getRotation()) } );
  return SpaceConfiguration( { goal_pose_.pose.position.x, goal_pose_.pose.position.y, tf2::getYaw(goal_pose_.pose.orientation) } );
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::generateRandomConfig()
{  
  
    /* COMPLETAR: Deben retornar una configuracion aleatoria dentro del espacio de busqueda.
     * 
     * ATENCION: - Tener encuenta el valor de la variable global goal_bias_ 
     *           - Pueden utilizar la funcion randBetween(a,b) para la generacion de numeros aleatorios 
     *           - Utilizar las funciones getOriginOfCell() y la informacion de la grilla para establecer el espacio de busqueda:
     *                grid_->info.width, grid_->info.height, grid_->info.resolution */  

/*random('Normal', 0, bt);*/
     
    double r = randBetween(0,1);
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double x_min = 0.0;
    double y_min = 0.0;
    double x_max = 0.0;
    double y_max = 0.0;
    double theta_min = -M_PI;
    double theta_max = M_PI;
    getOriginOfCell(0, 0, x_min, y_min); // esquina inferior izquierda
    getOriginOfCell(grid_->info.width - 1, grid_->info.height - 1, x_max, y_max); // esquina superior derecha

    double x_goal = goal_config_.get(0);
    double y_goal = goal_config_.get(1);
    double theta_goal = goal_config_.get(2);

    x = randBetween(x_min, x_max);
    y = randBetween(y_min, y_max);
    theta = randBetween(theta_min, theta_max);


    if (r < goal_bias_){
      float radio = 0.5; // definir un radio alrededor del goal
      x = randBetween(x_goal - radio, x_goal + radio);
      y = randBetween(y_goal - radio, y_goal + radio);
      //theta = randBetween(theta_goal - radio, theta_goal + radio);
    }
    
    SpaceConfiguration rand( {x, y, theta} );
    return rand;
}

double robmovil_planning::PioneerRRTPlanner::distancesBetween(const SpaceConfiguration& c1, const SpaceConfiguration& c2)
{
  /* COMPLETAR: Funcion auxiliar recomendada para evaluar la distancia entre configuraciones
   * 
   * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
   *           para medir la distancia entre las orientaciones */
  
  double dist_ori = abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) );
  double dist_pos = sqrt( pow(c1.get(0) - c2.get(0), 2) + pow(c1.get(1) - c2.get(1), 2) );

  
  return dist_pos + dist_ori;
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::nearest()
{
  /* COMPLETAR: Retornar configuracion mas cercana a la aleatoria (rand_config_). DEBE TENER HIJOS LIBRES
   * 
   * ATENCION: - Deberan recorrer la variable global graph_ la cual contiene los nodos del arbol
   *             generado hasta el momento como CLAVES DEL DICCIONARIO
   *           - Se recomienda establecer una relacion de distancia entre configuraciones en distancesBetween() 
   *             y utilizar esa funcion como auxiliar */
  
  SpaceConfiguration nearest;
  double min_distance = std::numeric_limits<double>::max(); 
  double max_hijos = 4; 
  
  for(const auto& config : graph_ )
  {
    double d = distancesBetween(config.first, rand_config_);
    if (d < min_distance && graph_[config.first].size() < max_hijos) // Asegura que tenga espacio para nuevos hijos
    {
      min_distance = d;
      nearest = config.first;
    }
    
  }

  return nearest;
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::steer()
{  
  /* COMPLETAR: Retornar una nueva configuracion a partir de la mas cercana near_config_.
   *            La nueva configuracion debe ser ademas la mas cercana a rand_config_ de entre las posibles.
   * 
   * ATENCION: - Utilizar las variables globales Vx_step_ , Wz_step_ para la aplicaciones de las velocidades
   *           - Pensar en la conversion de coordenadas polares a cartesianas al establecer la nueva configuracion
   *           - Utilizar angles::normalize_angle() */
  
  /* Ejemplo de como construir una posible configuracion: */
  std::vector<SpaceConfiguration> free_steerings;
  const std::list<SpaceConfiguration> occupied_steerings = graph_[near_config_];   // /* Conjunto de steers ya ocupados en la configuracion near_config_ */

  //CONFIGURACION POSIBLE 1
  double x_posible = near_config_.get(0) + Vx_step_ * cos(near_config_.get(2));
  double y_posible = near_config_.get(0) + Vx_step_ * sin(near_config_.get(2));
  double theta_posible = angles::normalize_angle(near_config_.get(2) + Wz_step_);
  SpaceConfiguration s_posible1({ x_posible, y_posible, theta_posible });

  bool es_igual = false;
  for (const auto& config : occupied_steerings) {
      if (s_posible1 == config) {
        es_igual = true;
        break;
      }
  }
  if (!es_igual) free_steerings.push_back(s_posible1);


  //CONFIGURACION POSIBLE 2
  x_posible = near_config_.get(0) - Vx_step_ * cos(near_config_.get(2));
  y_posible = near_config_.get(0) - Vx_step_ * sin(near_config_.get(2));
  theta_posible = angles::normalize_angle(near_config_.get(2) - Wz_step_);
  SpaceConfiguration s_posible2({ x_posible, y_posible, theta_posible });

  es_igual = false;
  for (const auto& config : occupied_steerings) {
      if (s_posible2 == config) {
        es_igual = true;
        break;
      }
  }
  if (!es_igual) free_steerings.push_back(s_posible2);

  //CONFIGURACION POSIBLE 3
  x_posible = near_config_.get(0) + Vx_step_ * cos(near_config_.get(2));
  y_posible = near_config_.get(0) - Vx_step_ * sin(near_config_.get(2));
  theta_posible = angles::normalize_angle(near_config_.get(2));
  SpaceConfiguration s_posible3({ x_posible, y_posible, theta_posible });

  es_igual = false;
  for (const auto& config : occupied_steerings) {
      if (s_posible3 == config) {
        es_igual = true;
        break;
      }
  }
  if (!es_igual) free_steerings.push_back(s_posible3);

  //CONFIGURACION POSIBLE 4
  x_posible = near_config_.get(0) - Vx_step_ * cos(near_config_.get(2));
  y_posible = near_config_.get(0) + Vx_step_ * sin(near_config_.get(2));
  theta_posible = angles::normalize_angle(near_config_.get(2));
  SpaceConfiguration s_posible4({ x_posible, y_posible, theta_posible });

  es_igual = false;
  for (const auto& config : occupied_steerings) {
      if (s_posible4 == config) {
        es_igual = true;
        break;
      }
  }
  if (!es_igual) free_steerings.push_back(s_posible4);

  SpaceConfiguration steer;
  
  
  /* RECOMENDACION: Establecer configuraciones posibles en free_steerings y calcular la mas cercana a rand_config_ */
  double min_distance = std::numeric_limits<double>::max();
  for (const auto& config : free_steerings)
  {
    double d = distancesBetween(config, rand_config_);
    if (d < min_distance)
    {
      min_distance = d;
      steer = config;
    }
  }
  
  return steer;
}

bool robmovil_planning::PioneerRRTPlanner::isFree()
{
  /* COMPLETAR: Utilizar la variable global new_config_ para establecer si existe un area segura alrededor de esta */
  bool esta_libre = true;

  const double x = new_config_.get(0);
  const double y = new_config_.get(1);

  const double tamaño_robot = 0.5 * grid_->info.resolution;

  std::vector<std::pair<double, double>> posiciones_que_ocupa = { 
    { x - tamaño_robot, y - tamaño_robot},
    { x - tamaño_robot, y + tamaño_robot},
    { x + tamaño_robot, y - tamaño_robot},
    { x + tamaño_robot, y + tamaño_robot},
    {x, y}
  };

  for (auto& pos : posiciones_que_ocupa) {
      if (isPositionOccupy(pos.first, pos.second)) {
          esta_libre = false;
          break;
      }
  }
  
  return esta_libre;
}

bool robmovil_planning::PioneerRRTPlanner::isGoalAchieve()
{
  
  /* COMPLETAR: Comprobar si new_config_ se encuentra lo suficientemente cerca del goal.
   * 
   * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
   *           para medir la distancia entre las orientaciones */

  const double gx = goal_config_.get(0);
  const double gy = goal_config_.get(1);
  const double gtheta = goal_config_.get(2);

  const double nx = new_config_.get(0);
  const double ny = new_config_.get(1);
  const double ntheta = new_config_.get(2);

  const double distancia_posicion = sqrt( pow(gx - nx, 2) + pow(gy - ny, 2) );
  const double distancia_orientacion = std::abs(angles::shortest_angular_distance(ntheta, gtheta));

  if (distancia_posicion <= 0.1 && distancia_orientacion <= M_PI/2.0) {
      return true;
  }
  else  return false;
}



/* DESDE AQUI YA NO HACE FALTA COMPLETAR */



bool robmovil_planning::PioneerRRTPlanner::isValid()
{ return true; }

void robmovil_planning::PioneerRRTPlanner::notifyTrajectory(robmovil_msgs::msg::Trajectory& result_trajectory,
                                                            const SpaceConfiguration& start, const SpaceConfiguration& goal, 
                                                            std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const
{
  std::vector<SpaceConfiguration> path;
  SpaceConfiguration current = goal;
  
  path.push_back(current);
  
  while(current != start)
  {
    current = came_from[current];
    path.push_back(current);
  }
  
  result_trajectory.header.stamp = this->now();
  result_trajectory.header.frame_id = "map";
  
  rclcpp::Duration t_from_start = rclcpp::Duration::from_seconds(0.0);
  rclcpp::Duration delta_t = rclcpp::Duration::from_seconds(1.0);

  /* Se recorre de atras para adelante */
  for (auto it = path.rbegin(); it != path.rend(); ++it) {    
    double config_x = it->get(0);
    double config_y = it->get(1);
    double config_theta = it->get(2);
    
    tf2::Transform wp_odom_ref;
    wp_odom_ref.setOrigin(tf2::Vector3(config_x, config_y, 0.0));
    tf2::Quaternion delta_q;
    delta_q.setRPY(0,0,config_theta);
    wp_odom_ref.setRotation(delta_q);
    
    //wp_odom_ref = map_to_odom_.inverse() * wp_odom_ref;
    tf2::Transform tf_map_to_odom;
    tf2::fromMsg(map_to_odom_.transform, tf_map_to_odom);

    wp_odom_ref = tf_map_to_odom.inverse() * wp_odom_ref;
    
    // Se crean los waypoints de la trayectoria
    robmovil_msgs::msg::TrajectoryPoint point_msg;
    
    //transformTFToMsg(wp_odom_ref, point_msg.transform);
    point_msg.transform = tf2::toMsg(wp_odom_ref);
    
    if(it != path.rend()-1) {
      double config_dx = (it+1)->get(0) - config_x;
      double config_dy = (it+1)->get(1) - config_y;
      double config_dtheta = angles::shortest_angular_distance(config_theta, (it+1)->get(2));
      point_msg.velocity.linear.x = sqrt(pow(config_dx,2) + pow(config_dy,2));
      point_msg.velocity.angular.z = config_dtheta;
    }else{
      point_msg.velocity.linear.x = 0;
      point_msg.velocity.angular.z = 0;
    }
    
    point_msg.time_from_start = t_from_start;
    
    result_trajectory.points.push_back( point_msg );
    
    t_from_start = t_from_start + delta_t;
  }
}
