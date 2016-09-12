#include <iai_naive_object_db/iai_naive_object_db.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iai_naive_object_db_main");
  ros::NodeHandle nh;
  iai_naive_object_db::ObjectDB my_object_db(nh);

  try
  {
    my_object_db.start(ros::Duration(0.1));
  }
  catch(const std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
    return 0;
  }
  
  ros::spin();

  return 0;
}
