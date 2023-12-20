#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class TF_NAV
{
    public:
        TF_NAV();
        void run();
        void tf_listener_fun();
        void position_pub();
        void goal_listener();
        void send_goal();

    private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;

        Eigen::Vector3d _home_pos;

        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;

        Eigen::Vector3d _goal1_pos, _goal2_pos, _goal3_pos, _goal4_pos;
        Eigen::Vector3d _goal3_1_pos, _goal3_2_pos, _goal3_3_pos, _goal3_4_pos, _goal3_5_pos, _goal3_6_pos, _goal3_7_pos, _goal3_8_pos;
        Eigen::Vector3d _goal_aruco_pos;

        Eigen::Vector4d _goal1_or, _goal2_or, _goal3_or, _goal4_or;
        Eigen::Vector4d _goal3_1_or, _goal3_2_or, _goal3_3_or, _goal3_4_or, _goal3_5_or, _goal3_6_or, _goal3_7_or, _goal3_8_or;
        Eigen::Vector4d _goal_aruco_or;

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
};