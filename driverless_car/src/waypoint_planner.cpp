#include <pluginlib/class_list_macros.h>
 #include "driverless_car/waypoint_planner.h"
#include <std_msgs/String.h>
#include"driverless_car/more.h"
#include"driverless_car/random.h"
 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(waypoint_planner::WaypointPlanner, nav_core::BaseGlobalPlanner)
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
 ROS_INFO("I heard: [%s]", msg->data.c_str());
}

 using namespace std;

 //Default Constructor
 namespace waypoint_planner {

 WaypointPlanner::WaypointPlanner (){

 }

 WaypointPlanner::WaypointPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }


 void WaypointPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

 }

 bool WaypointPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
plan.clear();    
geometry_msgs::PoseStamped new_goal = start;
    plan.push_back(start);
    driverless_car::more temp;
   // sensor_msgs::CameraInfoConstPtr right_camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>(right_camera_info_topic, ros::Duration(30));
   std::cout<<start.pose.position.x<<std::endl;
   temp=*(ros::topic::waitForMessage<driverless_car::more>("contact"));
   std::cout<<temp.msg.at(0).x<<std::endl;
   for(int i=1;i<temp.msg.size()-1;i++) {
     //tf::Quaternion goal_quat = ;
      tf::Quaternion q = tf::createQuaternionFromYaw(temp.msg.at(i).yaw);
      new_goal.pose.position.x = temp.msg.at(i).x;
      new_goal.pose.position.y = temp.msg.at(i).y;
      new_goal.pose.orientation.x=q.x();
      new_goal.pose.orientation.y=q.y();
      new_goal.pose.orientation.z=q.z();
      new_goal.pose.orientation.w = q.w();

   plan.push_back(new_goal);
   }
  //std::cout<<new_goal.pose.position.y <<endl;
   
   plan.push_back(goal);
  //publishPlan(plan);
  return true;
 }

 };

