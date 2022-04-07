#define _USE_MATH_DEFINES
#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h" // Mensajes que leemos de /odom para saber la posicion
#include "geometry_msgs/Twist.h"   // Mensajes que publicamos en /cmd_vel para enviar velocidades

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
geometry_msgs::Twist mensaje;

// Goal path and orientations
std::vector<float> goal_x {1, 1, 1};
std::vector<float> goal_y {1, 1, 1};
std::vector<float> goal_yaw {0,0,0};//{1.5707, 3.1415, -1.5707}; // 90, 180 y 270 grados
int step = 0; // The current point in the path
bool reached_x = false;
bool reached_y = false;

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

// Callback que se ejecuta cada vez que el subscriber recibe la pose del robot
void savePose(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Guardamos posicion actual
  curr_x = msg->pose.pose.position.x;
  curr_y = msg->pose.pose.position.y;
  curr_z = msg->pose.pose.position.z;

  // Guardamos orientacion actual en el struct Quaternion
  curr_q.x = msg->pose.pose.orientation.x;
  curr_q.y = msg->pose.pose.orientation.y;
  curr_q.z = msg->pose.pose.orientation.z;
  curr_q.w = msg->pose.pose.orientation.w;

  // Convertimos orientacion actual a RPY
  curr_rpy = ToEulerAngles(curr_q);

  ROS_INFO("ACTUAL-> x: %.2f, y: %.2f, z: %.2f, yaw: %.2f", 
    curr_x,
    curr_y,
    curr_z,
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
  // Giramos en yaw para encarar hacia el punto objetivo
  if(deltaYaw() > tolerance)
  {
    mensaje.angular.z = 0.5;
  }
  else if(deltaYaw() < -tolerance)
  {
    mensaje.angular.z = -0.5;
  }
  else
  {
    mensaje.angular.z = 0;
    ROS_INFO("Reached orientation in YAW");  
  }

  if(deltaX() > tolerance){
  //mensaje.linear.x = Kp*deltaX();
  mensaje.linear.x = 1;
  }
  else if(deltaX() < -tolerance){
    //mensaje.linear.x = -Kp*-deltaX();
    mensaje.linear.x = -1;
  }
  else{
    mensaje.linear.x = 0;
    ROS_INFO("Reached distance in X");
    reached_x = true;
  }

  if(deltaY() > tolerance){
    //mensaje.linear.y = Kp*deltaY();
    mensaje.linear.y = 1;
  }
  else if(deltaY() < -tolerance){
    //mensaje.linear.y = -Kp*-deltaY();
    mensaje.linear.y = -1;
  }
  else{
    mensaje.linear.y = 0;
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

  ROS_INFO("VELOC X: %f",  mensaje.linear.x);
  ROS_INFO("VELOC Y: %f",  mensaje.linear.y);
  ROS_INFO("VELOC YAW: %f",  mensaje.angular.z);

  pub.publish(mensaje);
}

int main(int argc, char **argv)
{
  // Iniciamos el nodo y el nodeHandler
  ros::init(argc, argv, "mecanum");
  ros::NodeHandle n;
  
  // Creamos un subscriber del topic /gazebo/model_states 
  ros::Subscriber sub = n.subscribe("/odom", 1000, savePose);

  // Publisher (declarado global) que publica en el topic /gazebo/set_model_state
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  
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
