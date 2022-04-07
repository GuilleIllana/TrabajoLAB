#include "ros/ros.h"
#include "laboratorio/Point.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_client");
  
  // User help in case of wrong number of arguments
  if (argc != 4)
  {
    ROS_INFO("Usage help: point x y yaw");
    return 1;
  }

  // Starts the node
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<laboratorio::Point>("point");
  laboratorio::Point srv;
  
  // Sets the request variable values
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  srv.request.yaw = atoll(argv[3]);
  
  // Calls the service
  if (client.call(srv)) {
    ROS_INFO("Success: %d", srv.response.success);
  }
  else {
    ROS_ERROR("Failed to call go_to service");
    return 1;
  }
  
  return 0;
}
