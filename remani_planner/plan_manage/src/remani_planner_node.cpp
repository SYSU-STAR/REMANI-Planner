#include <ros/ros.h>
#include <plan_manage/remani_replan_fsm.h>

using namespace remani_planner;

int main(int argc, char **argv){
  ros::init(argc, argv, "remani_planner_node");
  ros::NodeHandle nh("~");

  REMANIReplanFSM remani_replan;

  remani_replan.init(nh);
  ros::spin();

  return 0;
}
