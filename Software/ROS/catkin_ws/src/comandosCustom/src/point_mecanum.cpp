#define _USE_MATH_DEFINES
#include <cmath>
#include "ros/ros.h"
#include "laboratorio/Point.h"
#include "laboratorio/CustomVel.h"
#include "gazebo_msgs/ModelStates.h" // Mensajes que se leen de /gazebo/model_states // OJO LA 's' DEL FINAL
#include "gazebo_msgs/ModelState.h" // Mensajes que publicamos en /gazebo/set_model_state
#include <iostream>

// Definicion de structs
struct Quaternion {
  double w, x, y, z;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

// *** Variables globales ***
// Position (xyz)
float curr_x = 0;
float curr_y = 0;
float curr_z = 0;
// Orientation
Quaternion curr_q;
EulerAngles curr_rpy;

// Publisher y el mensaje que publica
ros::Publisher pub;
ros::Publisher pub_custom;
gazebo_msgs::ModelState mensaje;
laboratorio::CustomVel custom_msg;

// Goal path and orientations

float goal_x = 0;
float goal_y = 0;
float goal_yaw = 0;

bool reached_x = false;
bool reached_y = false;
bool reached_yaw = false;

// Parametros del regulador proporcional
float Kp = 1.2;
float tolerance = 0.2;

// Funciones de conversion entre cuaterniones y rol-pitch-yaw
Quaternion ToQuaternion(double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
{
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

EulerAngles ToEulerAngles(Quaternion q) {
  EulerAngles angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
      angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      angles.pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

// Callback que se ejecuta cada vez que un cliente llama al servicio go_to
bool setGoal (laboratorio::Point::Request &req, laboratorio::Point::Response &res)
{
  // Si las variables existen, el servicio devuelve true
  ROS_INFO("Requested position: x=%ld, y=%ld", req.x, req.y);
  ROS_INFO("Requested orientation: %ld rad", req.yaw);
  res.success = true;
  
  // ROS_INFO("Parameters received incorrectly");
  // res.success = false;

  // Guardamos los objetivos recibidos
  goal_x = req.x;
  goal_y = req.y;
  goal_yaw = req.yaw;

  return res.success;
}

// Callback que se ejecuta cada vez que el subscriber recibe la pose del robot
void savePose(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
// Guardamos posicion actual
curr_x = msg->pose[1].position.x;
curr_y = msg->pose[1].position.y;
curr_z = msg->pose[1].position.z;

// Guardamos orientacion actual en el struct Quaternion
curr_q.x = msg->pose[1].orientation.x;
curr_q.y = msg->pose[1].orientation.y;
curr_q.z = msg->pose[1].orientation.z;
curr_q.w = msg->pose[1].orientation.w;

// Convertimos orientacion actual a RPY
curr_rpy = ToEulerAngles(curr_q);

}

// Funciones para calcular error en X y en Y
float deltaX()
{
return goal_x - curr_x;
}

float deltaY()
{
return goal_y - curr_y;
}

float deltaYaw()
{
return goal_yaw - curr_rpy.yaw;
}


// Funcion de control que pone velocidades y publica el mensaje
void control ()
{
  mensaje.pose.position.x = curr_x;
  mensaje.pose.position.y = curr_y;

  mensaje.pose.orientation.x = curr_q.x;
  mensaje.pose.orientation.y = curr_q.y;
  mensaje.pose.orientation.z = curr_q.z;
  mensaje.pose.orientation.w = curr_q.w;

  // Giramos en yaw para encarar hacia el punto objetivo
  if(deltaYaw() > tolerance)
  {
    mensaje.twist.angular.z = 2;
    custom_msg.vel_yaw = 2; // para el robot fisico
  }
  else if(deltaYaw() < -tolerance)
  {
    mensaje.twist.angular.z = -2;
    custom_msg.vel_yaw = 2; // para el robot fisico
  }
  else
  {
    mensaje.twist.angular.z = 0;
    custom_msg.vel_yaw = 0; // para el robot fisico
    ROS_INFO("Reached orientation in YAW");
    reached_yaw = true;
  }

  if(deltaX() > tolerance){
  //mensaje.twist.linear.x = Kp*deltaX();
    mensaje.twist.linear.x = 1;
    custom_msg.vel_x = 1; // para el robot fisico

  }
  else if(deltaX() < -tolerance){
    //mensaje.twist.linear.x = -Kp*-deltaX();
    mensaje.twist.linear.x = -1;
    custom_msg.vel_x = -1; // para el robot fisico

  }
  else{
    mensaje.twist.linear.x = 0;
    custom_msg.vel_x = 0; // para el robot fisico
    ROS_INFO("Reached distance in X");
    reached_x = true;
  }

  if(deltaY() > tolerance){
    //mensaje.twist.linear.y = Kp*deltaY();
    mensaje.twist.linear.y = 1;
    custom_msg.vel_y = 1; // para el robot fisico
  }
  else if(deltaY() < -tolerance){
    //mensaje.twist.linear.y = -Kp*-deltaY();
    mensaje.twist.linear.y = -1;
    custom_msg.vel_y = -1; // para el robot fisico
  }
  else{
    mensaje.twist.linear.y = 0;
    custom_msg.vel_y = 0; // para el robot fisico
    ROS_INFO("Reached distance in Y");
    reached_y = true;
  }

  // Pasamos al siguiente punto del camino
  if (reached_x && reached_y && reached_yaw)
  {       
    // std::cout << "introduce posicion destino: " << std::endl;
    // std::cout << "X: " << std::endl;
    // std::cin >> goal_x;
    // std::cout << "Y: " << std::endl;
    // std::cin >> goal_y;
    // std::cout << "Yaw: " << std::endl;
    // std::cin >> goal_yaw;
    reached_x = false;
    reached_y = false;
    reached_yaw = false;
  }

  // ROS_INFO("VELOC X: %f",  mensaje.twist.linear.x);
  // ROS_INFO("VELOC Y: %f",  mensaje.twist.linear.y);
  // ROS_INFO("VELOC YAW: %f",  mensaje.twist.angular.z);

  //custom_msg.vel_x ++;
  //custom_msg.vel_y ++;
  pub_custom.publish(custom_msg);

  pub.publish(mensaje);
}

int main(int argc, char **argv)
{
// Iniciamos el nodo y el nodeHandler
ros::init(argc, argv, "point_mecanum");
ros::NodeHandle n;

// Creamos un subscriber del topic /gazebo/model_states 
ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, savePose);

// Publisher (declarado global) que publica en el topic /gazebo/set_model_state
pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);
pub_custom = n.advertise<laboratorio::CustomVel>("/custom_vel", 1000); // Publicamos en un topic creado por nosotros llamado /custom_vel, y de ahi leera el arduino las velocidades

custom_msg.vel_x = 2;
custom_msg.vel_y = 4;
custom_msg.vel_yaw = 0;
custom_msg.time_mov = 10;

ros::ServiceServer service = n.advertiseService("point", setGoal);
ROS_INFO("Server ready to get a goal");

// Rellenamos nombre y mundo del mensaje
mensaje.model_name = "nexus_4wd_mecanum";
mensaje.reference_frame = "world";

// Frecuencia de refresco de mensajes (10Hz en este caso)
ros::Rate loop_rate(10);

// Bucle principal
while(ros::ok()){
  ros::spinOnce();
  control();

  loop_rate.sleep();
}

return 0;
}
