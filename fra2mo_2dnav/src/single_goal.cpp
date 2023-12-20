#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Global variables
std::vector<double> aruco_pose(7,0.0);
bool aruco_pose_available = false;

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);

    std::cout<<"Aruco position: x: "<<aruco_pose[0]<<", y: "<<aruco_pose[1]<<", z: "<<aruco_pose[2]<<std::endl;
    std::cout<<"Aruco orientation: or.x: "<<aruco_pose[3]<<", or.y: "<<aruco_pose[4]<<", or.z: "<<aruco_pose[5]<<", or.w:"<<aruco_pose[6]<<std::endl;


}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
 
  move_base_msgs::MoveBaseGoal goal;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
 
  goal.target_pose.pose.position.x = -15.5;
  goal.target_pose.pose.position.y = 8.0;
  goal.target_pose.pose.orientation.z = 1.0;

    // Init node
    ros::init(argc, argv, "fra2mo_2dnav_aruco");
    ros::NodeHandle n;

    // Rate
    ros::Rate loop_rate(500);


  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  std::cout<<"prima"<<std::endl;
  
  ac.waitForResult();

  std::cout<<"dopo"<<std::endl;

  // Subscribers
  ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);
   
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved");
    else
      ROS_INFO("The base failed to move for some reason");



 
   return 0;
 }