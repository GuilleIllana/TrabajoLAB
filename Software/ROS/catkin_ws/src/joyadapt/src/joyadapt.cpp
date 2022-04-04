#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32MultiArray.h>

class joyHandler
{
public:
  joyHandler();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;
  ros::Publisher joy_pub_;
  ros::Subscriber joy_sub_;
};

joyHandler::joyHandler() {
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &joyHandler::joyCallback, this);
  joy_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("joyarduino", 9);
}

void joyHandler::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Int32MultiArray array;
  
  array.data.clear();
  for (int i = 0; i < 9; i++){
  	array.data.push_back(joy->buttons[i]);
  }
  joy_pub_.publish(array);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_handler");
  joyHandler joy_handler;
  ros::spin();
}
