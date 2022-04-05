#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

class joyHandler
{
public:
  joyHandler();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;
  ros::Publisher joy_pub_buttons;
  ros::Publisher joy_pub_axes;
  ros::Subscriber joy_sub_;
};

joyHandler::joyHandler() {
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &joyHandler::joyCallback, this);
  joy_pub_buttons = nh_.advertise<std_msgs::Int32MultiArray>("joyarduinobu", 9);
  joy_pub_axes = nh_.advertise<std_msgs::Float32MultiArray>("joyarduinoax", 9);
}

void joyHandler::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Int32MultiArray array_bu;
  std_msgs::Float32MultiArray array_ax;
  
  array_bu.data.clear();
  for (int i = 0; i < 9; i++){
  	array_bu.data.push_back(joy->buttons[i]);
  }
  
  array_ax.data.clear();
  for (int i = 0; i < 6; i++){
  	array_ax.data.push_back(joy->axes[i]);
  }
  
  joy_pub_buttons.publish(array_bu);
  joy_pub_axes.publish(array_ax);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_handler");
  joyHandler joy_handler;
  ros::spin();
}
