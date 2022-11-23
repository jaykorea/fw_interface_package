# freeway_joy_fw

```

#include "ros/ros.h"
#include "roscpp_tutorials/TwoInts.h"

// %Tag(CLASS_DECLARATION)%
class AddTwo
{
public:
  bool add(roscpp_tutorials::TwoInts::Request& req,
           roscpp_tutorials::TwoInts::Response& res);
};
// %EndTag(CLASS_DECLARATION)%

bool AddTwo::add(roscpp_tutorials::TwoInts::Request& req,
                 roscpp_tutorials::TwoInts::Response& res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("  sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

// %Tag(SERVICE_SERVER)%
  AddTwo a;
  ros::ServiceServer ss = n.advertiseService("add_two_ints", &AddTwo::add, &a);
// %EndTag(SERVICE_SERVER)%

  ros::spin();

  return 0;
}

```
