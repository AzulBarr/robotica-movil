#include "AStarPlanner.h"
#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#define COST_BETWEEN_CELLS 1

robmovil_planning::AStarPlanner::AStarPlanner()
: GridPlanner()
{ }

std::vector<robmovil_planning::AStarPlanner::Cell> robmovil_planning::AStarPlanner::neighbors(const Cell& c)
{
  /* COMPLETAR: Calcular un vector de vecinos (distintos de c).
   * IMPORTANTE: Tener en cuenta los limites de la grilla (utilizar grid_->info.width y grid_->info.heigh)
   *             y aquellas celdas ocupadas */
  
   std::vector<Cell> neighbors;

   int i = c.i ; // columna
   int j = c.j ; // fila

  
   for (int fila = -1; fila < 2; fila++){
      for (int columna = -1; columna < 2; columna++){
        if (grid_->info.width > i + columna && i + columna >= 0 and grid_->info.height > j + fila and j + fila >= 0) {
          if (isANeighborCellOccupy(i + columna, j + fila) == false && !(fila == 0 && columna == 0)) {
            
            //RCLCPP_INFO(this->get_logger(), "Vecino agregado (%d, %d)", i + columna, j + fila);
            neighbors.push_back(Cell(i + columna, j + fila));
          
          }
        }
      }
   }

  return neighbors;
}

double robmovil_planning::AStarPlanner::heuristic_cost(const Cell& start, const Cell& goal, const Cell& current)
{  
  /* COMPLETAR: Funcion de heuristica de costo */

  double heuristic = 0.0;

  double x_c = 0;
  double y_c = 0;
  getCenterOfCell(current.i, current.j,x_c,y_c);

  double x_g = 0;
  double y_g = 0;
  getCenterOfCell(goal.i, goal.j,x_g,y_g);

  heuristic = abs(x_c - x_g) + abs(y_c - y_g); //esto
  //heuristic = abs(current.i - goal.i) + abs(current.j - goal.j); //Manhattan
  return heuristic;
}

bool robmovil_planning::AStarPlanner::do_planning(robmovil_msgs::msg::Trajectory& result_trajectory)
{
  uint start_i, start_j;
  uint goal_i, goal_j;
  
  getCellOfPosition(starting_pose_.pose.position.x, starting_pose_.pose.position.y, start_i, start_j);
  getCellOfPosition(goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_i, goal_j);
  //goal_pose_.pose.position.z
  
  /* Celdas de inicio y destino */
  Cell start = Cell(start_i, start_j);
  Cell goal = Cell(goal_i, goal_j);
  
  /* Contenedores auxiliares recomendados para completar el algoritmo */
  std::priority_queue<CellWithPriority, std::vector<CellWithPriority>, PriorityCompare> frontier; //OPEN
  std::map<Cell, Cell> came_from; // prev[]
  std::map<Cell, double> cost_so_far; //g[v]
  
  bool path_found = false;
  
  /* Inicializacion de los contenedores (start comienza con costo 0) */
  frontier.push(CellWithPriority(start, 0)); //add start to OPEN
  cost_so_far[start] = 0; //g[start] ← 0
  
  /* COMPLETAR: Utilizar los contenedores auxiliares para la implementacion del algoritmo A*
   * NOTA: Pueden utilizar las funciones neighbors(const Cell& c) y heuristic_cost(const Cell& start, const Cell& goal, const Cell& current)
   *       para la resolucion */
  
  std::map<Cell, double> f_distancia; //f[v] 
  f_distancia[start] = heuristic_cost(start, goal, goal); //f[start] ← h(goal)

  std::set<Cell> closed;

  while (!frontier.empty()) { //while OPEN is not empty
    CellWithPriority current = frontier.top(); //current ← vertex in OPEN with min f[] 

    if (current == goal){  //if current = goal then
      notifyTrajectory(result_trajectory, start, goal, came_from); //return reconstruct path(prev , current)
      return true;
    }
    frontier.pop(); //remove current from OPEN

    closed.insert(current); //add current to CLOSED

    std::vector<robmovil_planning::AStarPlanner::Cell> vecinos = neighbors(current); 
    for (int i = 0; i <  size(vecinos); i++) { //for each neighbor of current do
      Cell vecino = vecinos[i];


      if (closed.count(vecino)) { //if neighbor in CLOSED then
        continue; //continue
      }

      int costo = cost_so_far[current] + COST_BETWEEN_CELLS; //cost = g[current] + movement cost(current, neighbor)


      if (cost_so_far.find(vecino) == cost_so_far.end()) { //if neighbor not in OPEN then
        CellWithPriority vecino_prioridad(vecino, cost_so_far[vecino] + heuristic_cost(start, goal, vecino));
        frontier.push(vecino_prioridad);
      } else if (costo >= cost_so_far[vecino]){ //else if cost ≥ g[neighbor] then{
        continue; //continue {esto no es un mejor camino }
      }
      came_from[vecino] = current; //prev [neighbor ] ← current
      cost_so_far[vecino] = costo; //g[neighbor ] ← cost
      f_distancia[vecino] = cost_so_far[vecino] + heuristic_cost(start, goal, vecino); //f[neighbor ] ← g[neighbor ] + h(vecino)
      
    }
    
  }

  
  if(not path_found)
    return false;
  
  /* Construccion y notificacion de la trajectoria.
   * NOTA: Se espera que came_from sea un diccionario representando un grafo de forma que:
   *       goal -> intermedio2 -> intermedio1 -> start */
  notifyTrajectory(result_trajectory, start, goal, came_from);

  return true;
}



void robmovil_planning::AStarPlanner::notifyTrajectory(robmovil_msgs::msg::Trajectory& result_trajectory, const Cell& start, const Cell& goal, 
                                                       std::map<Cell, Cell>& came_from)
{
  std::vector<Cell> path;
  Cell current = goal;
  
  path.push_back(current);
  
  while(current != start)
  {
    current = came_from[current];
    path.push_back(current);
  }

  /* Se recorre de atras para adelante */
  for (auto it = path.rbegin(); it != path.rend(); ++it) {
    //ROS_INFO_STREAM("Path " << it->i << ", " << it->j);
    RCLCPP_INFO(this->get_logger(), "Path: %u, %u", it->i, it->j);
    
    double cell_x, cell_y;
    
    getCenterOfCell(it->i, it->j, cell_x, cell_y);
    
    // Se crean los waypoints de la trajectoria
    robmovil_msgs::msg::TrajectoryPoint point_msg;

    point_msg.transform.translation.x = cell_x;
    point_msg.transform.translation.y = cell_y;
    point_msg.transform.translation.z = 0;
    
    if(it != path.rend()-1){
      double delta_x, delta_y;
      getCenterOfCell((it+1)->i, (it+1)->j, delta_x, delta_y);
      delta_x = delta_x - cell_x;
      delta_y = delta_y - cell_y;
      tf2::Quaternion delta_q;
      delta_q.setRPY(0, 0, angles::normalize_angle(atan2(delta_y, delta_x)));
      point_msg.transform.rotation = tf2::toMsg(delta_q);
      //point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( angles::normalize_angle(atan2(delta_y, delta_x)) );
    } else {
      tf2::Quaternion delta_q;
      delta_q.setRPY(0, 0, angles::normalize_angle(atan2(cell_y, cell_x)));
      point_msg.transform.rotation = tf2::toMsg(delta_q);
      //point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( angles::normalize_angle(atan2(cell_y, cell_x)) );
    }
    
    result_trajectory.points.push_back( point_msg );
  }
}