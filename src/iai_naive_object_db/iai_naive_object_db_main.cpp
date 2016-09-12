#include <iai_naive_object_db/iai_naive_object_db_node.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iai_naive_object_db_main");
  ros::NodeHandle nh;
  iai_naive_object_db::ObjectDBNode my_object_db_node(nh);  // Write the correct class

  try
  {
    my_object_db_node.start(ros::Duration(0.1));
  }
  catch(const std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
    return 0;
  }
  
  ros::spin();

  return 0;
}
