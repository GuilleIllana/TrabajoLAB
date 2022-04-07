#include "ros/ros.h"
#include "laboratorio/Path.h"
#include <cstdlib>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_client");
  
  //User help in case of wrong number of arguments
  if (argc != 4)
  {
    ROS_INFO("Usage help: path [x1,x2,x3] [y1,y2,y3] [yaw1,yaw2,yaw3]");
    return 1;
  }
  
  // Starts the node
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<laboratorio::Path>("path");
  laboratorio::Path srv;

  srv.request.x = (std::vector<int64_t>){100,100,100};
  srv.request.y = (std::vector<int64_t>){100,100,100};
  srv.request.yaw = (std::vector<int64_t>){100,100,100};
  
  srv.request.x[0] = argv[1][1] - '0';
  srv.request.x[1] = argv[1][3] - '0';
  srv.request.x[2] = argv[1][5] - '0';

  srv.request.y[0] = argv[2][1] - '0';
  srv.request.y[1] = argv[2][3] - '0';
  srv.request.y[2] = argv[2][5] - '0';

  srv.request.yaw[0] = argv[3][1] - '0';
  srv.request.yaw[1] = argv[3][3] - '0';
  srv.request.yaw[2] = argv[3][5] - '0';

  cout << "bien\n";

  // for (int i = 0; i < 3; i++){
  //   cout << "bucle x i:" << i << endl;
  //   cout << srv.request.x[i] << endl;
  // }

  // for (int i = 0; i < 3; i++){
  //   cout << "bucle y i:" << i << endl;
  //   cout << srv.request.y[i] << endl;
  // }

  // for (int i = 0; i < 3; i++){
  //   cout << "bucle yaw i:" << i << endl;
  //   cout << srv.request.yaw[i] << endl;
  // }

  // Calls the service
  if (client.call(srv)) {
    ROS_INFO("Success: %d", srv.response.success);
  }
  else {
    ROS_ERROR("Failed to call path service");
    return 1;
  }
  
  return 0;
}
