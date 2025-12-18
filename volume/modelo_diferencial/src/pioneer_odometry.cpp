#include "pioneer_odometry.h"
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <robmovil_msgs/msg/multi_encoder_ticks.hpp>

using namespace robmovil;

#define WHEEL_BASELINE 0.331 //falta actualizar
#define WHEEL_RADIUS 0.0975
#define ENCODER_TICKS 500.0

PioneerOdometry::PioneerOdometry() : Node("nodeOdometry"), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10), std::bind(&PioneerOdometry::on_velocity_cmd, this, std::placeholders::_1));

  vel_pub_front_left_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_front_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_right_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rear_left_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rear_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_right_wheel/cmd_vel", rclcpp::QoS(10));

  encoder_sub_ =  this->create_subscription<robmovil_msgs::msg::MultiEncoderTicks>("/robot/encoders", rclcpp::QoS(10), std::bind(&PioneerOdometry::on_encoder_ticks, this, std::placeholders::_1));
  
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  x_ = 0;
  y_ = 0;
}

void PioneerOdometry::on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  /** Completar los mensajes de velocidad vLeft y vRight*/
     
  double wFrontLeft = (twist->linear.x - twist->linear.y - WHEEL_BASELINE * twist->angular.z) / WHEEL_RADIUS;
  double wFrontRight = (twist->linear.x + twist->linear.y + WHEEL_BASELINE * twist->angular.z) / WHEEL_RADIUS;
  double wRearLeft = (twist->linear.x + twist->linear.y - WHEEL_BASELINE * twist->angular.z) / WHEEL_RADIUS;
  double wRearRight = (twist->linear.x - twist->linear.y + WHEEL_BASELINE * twist->angular.z) / WHEEL_RADIUS;

  //double vFrontLeft = (wFrontLeft * WHEEL_RADIUS) / cos(45);
  //double vFrontRight = (wFrontRight * WHEEL_RADIUS) / cos(45);
  //double vRearLeft = (wRearLeft * WHEEL_RADIUS) / cos(45);
  //double vRearRight = (wRearRight * WHEEL_RADIUS) / cos(45);

  // publish front left velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = wFrontLeft;

    vel_pub_front_left_->publish(msg);
  }

  // publish front right velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = wFrontRight;

    vel_pub_front_right_->publish(msg);
  }

  // publish rear_left velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = wRearLeft;

    vel_pub_rear_left_->publish(msg);
  }


  // publish rear_right velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = wRearRight;

    vel_pub_rear_right_->publish(msg);
  }
}

void PioneerOdometry::on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (!ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_front_left_ = encoder->ticks[0];
    last_ticks_front_right_ = encoder->ticks[1];
    last_ticks_rear_left_ = encoder->ticks[2];
    last_ticks_rear_right_ = encoder->ticks[3];
    last_ticks_time = encoder->header.stamp;
    return;
  }

  int32_t delta_ticks_front_left = encoder->ticks[0] - last_ticks_front_left_;
  int32_t delta_ticks_front_right = encoder->ticks[1] - last_ticks_front_right_;
  int32_t delta_ticks_rear_left = encoder->ticks[2] - last_ticks_rear_left_;
  int32_t delta_ticks_rear_right = encoder->ticks[3] - last_ticks_rear_right_;

  // calcular el desplazamiento relativo: delta_x, delta_y, delta_theta

  /* Utilizar este delta de tiempo entre momentos */
  rclcpp::Time current_time(encoder->header.stamp);
  double delta_t = (current_time - last_ticks_time).seconds();

  double w_front_left = M_PI * 2 * delta_ticks_front_left / ENCODER_TICKS * delta_t;
  double w_front_right = M_PI * 2 * delta_ticks_front_right / ENCODER_TICKS * delta_t;
  double w_rear_left = M_PI * 2 * delta_ticks_rear_left / ENCODER_TICKS * delta_t;
  double w_rear_right = M_PI * 2 * delta_ticks_rear_right / ENCODER_TICKS * delta_t;

  double longitudinal_velocity = (w_front_left + w_front_right + w_rear_left + w_rear_right) * (WHEEL_RADIUS / 4);
  double transversal_velocity = (-w_front_left + w_front_right + w_rear_left - w_rear_right) * (WHEEL_RADIUS / 4);
  double angular_velocity = (-w_front_left + w_front_right - w_rear_left + w_rear_right) * (WHEEL_RADIUS / (4 * WHEEL_BASELINE));

  double delta_x = longitudinal_velocity * delta_t;
  double delta_y = transversal_velocity * delta_t;
  double delta_theta = angular_velocity * delta_t;

  /** Utilizar variables globales x_, y_, theta_ definidas en el .h */
  x_ += delta_x;
  y_ += delta_y;  
  theta_ += delta_theta;

  // Construir el mensaje odometry utilizando el esqueleto siguiente:
  nav_msgs::msg::Odometry msg;

  msg.header.stamp = encoder->header.stamp;
  msg.header.frame_id = "odom"; //cambiado para el taller 12
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);  // roll, pitch, yaw
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x = longitudinal_velocity;
  msg.twist.twist.linear.y = transversal_velocity;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = angular_velocity;

  pub_odometry_->publish( msg );

  // Actualizo las variables de estado

  last_ticks_front_left_ = encoder->ticks[0];
  last_ticks_front_right_ = encoder->ticks[1];
  last_ticks_rear_left_ = encoder->ticks[2];
  last_ticks_rear_right_ = encoder->ticks[3];
  last_ticks_time = current_time;

  /* Mando tambien un transform usando TF */

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "odom"; //cambiado para el taller 12
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg.pose.pose.position.x;
  t.transform.translation.y = msg.pose.pose.position.y;
  t.transform.translation.z = msg.pose.pose.position.z;
  t.transform.rotation = msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);


}
