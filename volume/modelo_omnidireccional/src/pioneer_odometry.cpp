#include "pioneer_odometry.h"
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace robmovil;

#define WHEEL_BASELINE 0.35
#define WHEEL_RADIUS 0.05
#define ENCODER_TICKS 500.0

PioneerOdometry::PioneerOdometry() : Node("nodeOdometry"), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/../cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10), std::bind(&PioneerOdometry::on_velocity_cmd, this, std::placeholders::_1));

  vel_pub_front_left_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_front_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_right_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rear_left_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rear_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_right_wheel/cmd_vel", rclcpp::QoS(10));
  
  encoder_sub_ =  this->create_subscription<robmovil_msgs::msg::MultiEncoderTicks>("/robot/encoders", rclcpp::QoS(10), std::bind(&PioneerOdometry::on_encoder_ticks, this, std::placeholders::_1));
  
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void PioneerOdometry::on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  // Completar los mensajes de velocidad vFrontLeft, vFrontRight, vRearLeft y vRearRight 
     
  double vFrontLeft = (twist->linear.x - twist->linear.y - (WHEEL_BASELINE * twist->angular.z)) / WHEEL_RADIUS;
  double vFrontRight = (twist->linear.x + twist->linear.y + (WHEEL_BASELINE * twist->angular.z)) / WHEEL_RADIUS;
  double vRearLeft = (twist->linear.x + twist->linear.y - (WHEEL_BASELINE * twist->angular.z)) / WHEEL_RADIUS;
  double vRearRight = (twist->linear.x - twist->linear.y + (WHEEL_BASELINE * twist->angular.z)) / WHEEL_RADIUS;


  // publish front left velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vFrontLeft;

    vel_pub_front_left_->publish(msg);
  }

  // publish front right velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vFrontRight;

    vel_pub_front_right_->publish(msg);
  }

  // publish rear left velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vRearLeft;

    vel_pub_rear_left_->publish(msg);
  }

  // publish rear right velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vRearRight;

    vel_pub_rear_right_->publish(msg);
  }
}

void PioneerOdometry::on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder)
{
  // La primera vez que llega un mensaje de encoders inicializo las variables de estado.
  // El orden desde Coppelia vemos que es ticks={ FrontLeft, FrontRight, RearLeft, RearRight}
  if (!ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_front_left_ = encoder->ticks[0];
    last_ticks_front_right_ = encoder->ticks[1];
    last_ticks_rear_left_ = encoder->ticks[2]; 
    last_ticks_rear_right_ = encoder->ticks[3];
    last_ticks_time = encoder->header.stamp;
    return;
  }

  int32_t delta_ticks_front_left_ = encoder->ticks[0] - last_ticks_front_left_;
  int32_t delta_ticks_front_right_ = encoder->ticks[1] - last_ticks_front_right_;
  int32_t delta_ticks_rear_left_ = encoder->ticks[2] - last_ticks_rear_left_;
  int32_t delta_ticks_rear_right_ = encoder->ticks[3] - last_ticks_rear_right_;

  // calcular el desplazamiento relativo: delta_x, delta_y, delta_theta

  rclcpp::Time current_time(encoder->header.stamp);
  double delta_t = (current_time - last_ticks_time).seconds();

  double W_front_left = 2 * M_PI * delta_ticks_front_left_ / ENCODER_TICKS / delta_t; //nos dijo pablo de revoluciones por unidad de tiempo a rasdianes por ?segundo
  double W_front_right = 2 * M_PI * delta_ticks_front_right_ / ENCODER_TICKS / delta_t;
  double W_rear_left = 2 * M_PI * delta_ticks_rear_left_ / ENCODER_TICKS / delta_t;
  double W_rear_right = 2 * M_PI * delta_ticks_rear_right_ / ENCODER_TICKS / delta_t;

  double longitudinal_Velocity = (W_front_left + W_front_right + W_rear_left + W_rear_right) * (WHEEL_RADIUS / 4); //eq 20-23
  double transversal_Velocity = (-W_front_left + W_front_right + W_rear_left - W_rear_right) * (WHEEL_RADIUS / 4);
  double angular_Velocity = (-W_front_left + W_front_right - W_rear_left + W_rear_right) * (WHEEL_RADIUS / (4 * WHEEL_BASELINE));

  double delta_x_robot = longitudinal_Velocity * delta_t;
  double delta_y_robot = transversal_Velocity * delta_t;
  double delta_theta = angular_Velocity * delta_t;

  double dx =  cos(theta_) * delta_x_robot - sin(theta_) * delta_y_robot;
  double dy =  sin(theta_) * delta_x_robot + cos(theta_) * delta_y_robot;

  x_ += dx;
  y_ += dy;
  theta_ += delta_theta;

  // Construir el mensaje odometry utilizando el esqueleto siguiente:
  nav_msgs::msg::Odometry msg;

  msg.header.stamp = encoder->header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);  // roll, pitch, yaw
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x = longitudinal_Velocity;
  msg.twist.twist.linear.y = transversal_Velocity;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = angular_Velocity;

  //std::cout << x_ << " " << y_ << std::endl;

  pub_odometry_->publish(msg);

  // Actualizo las variables de estado

  last_ticks_front_left_ = encoder->ticks[0];
  last_ticks_front_right_ = encoder->ticks[1];
  last_ticks_rear_left_ = encoder->ticks[2]; 
  last_ticks_rear_right_ = encoder->ticks[3];
  last_ticks_time = current_time;

  /* Mando tambien un transform usando TF */

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg.pose.pose.position.x;
  t.transform.translation.y = msg.pose.pose.position.y;
  t.transform.translation.z = msg.pose.pose.position.z;
  t.transform.rotation = msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);


}
