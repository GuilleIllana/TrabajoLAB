#define _USE_MATH_DEFINES
#include <cmath>
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h" // Mensajes que se leen de /gazebo/model_states // OJO LA 's' DEL FINAL
#include "gazebo_msgs/ModelState.h" // Mensajes que publicamos en /gazebo/set_model_state


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
gazebo_msgs::ModelState mensaje;

// Goal path and orientations
std::vector<float> goal_x {1, 3, -2};
std::vector<float> goal_y {4, -1, 0};
std::vector<float> goal_yaw{1.5707, 3.1415, -1.5707}; // 90, 180 y 270 grados
int step = 0; // The current point in the path
bool reached_x = false;
bool reached_y = false;

// Parametros del regulador proporcional
float Kp = 1.2;
float tolerance = 0.1;

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

    ROS_INFO("Actual-> x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", 
            msg->pose[1].position.x,
            msg->pose[1].position.y,
            msg->pose[1].position.z,
            curr_rpy.yaw
            );
}

// Funciones para calcular error en X y en Y
float deltaX()
{
  return goal_x[step] - curr_x;
}

float deltaY()
{
  return goal_y[step] - curr_y;
}

float deltaYaw()
{
  return goal_yaw[step] - curr_rpy.yaw;
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
  }
  else if(deltaYaw() < -tolerance)
  {
    mensaje.twist.angular.z = -2;
  }
  else
  {
    mensaje.twist.angular.z = 0;
    ROS_INFO("Reached orientation in YAW");
  }

  if(deltaX() > tolerance){
  mensaje.twist.linear.x = Kp*deltaX();
  }
  else if(deltaX() < -tolerance){
    mensaje.twist.linear.x = -Kp*-deltaX();
  }
  else{
    mensaje.twist.linear.x = 0;
    ROS_INFO("Reached distance in X");
    reached_x = true;
  }

  if(deltaY() > tolerance){
    mensaje.twist.linear.y = Kp*deltaY();
  }
  else if(deltaY() < -tolerance){
    mensaje.twist.linear.y = -Kp*-deltaY();
  }
  else{
    mensaje.twist.linear.y = 0;
    ROS_INFO("Reached distance in Y");
    reached_y = true;
  }

  // Pasamos al siguiente punto del camino
  if (reached_x && reached_y && step < 3)
  {
    step++;
    reached_x = false;
    reached_y = false;
  }

  ROS_INFO("VELOC X: %f",  mensaje.twist.linear.x);
  ROS_INFO("VELOC Y: %f",  mensaje.twist.linear.y);

  pub.publish(mensaje);
}

int main(int argc, char **argv)
{
  // Iniciamos el nodo y el nodeHandler
  ros::init(argc, argv, "mecanum");
  ros::NodeHandle n;
  
  // Creamos un subscriber del topic /gazebo/model_states 
  ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, savePose);

  // Publisher (declarado global) que publica en el topic /gazebo/set_model_state
  pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);

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
