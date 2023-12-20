#include "../include/tf_nav.h"
#include <tf/transform_broadcaster.h>

std::vector<double>aruco_pose(7,0.0);
bool aruco_pose_available = false;

TF_NAV::TF_NAV()
{
    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );
    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 1.0;
    _home_pos << -3.0, 5.0, 0.0;

    //Point 2
    _goal1_pos << 0.0, 0.0, 0.0;
    _goal1_or << 0.0, 0.0, 0.0, 1.0;
    _goal2_pos << 0.0, 0.0, 0.0;
    _goal2_or << 0.0, 0.0, 0.0, 1.0;
    _goal3_pos << 0.0, 0.0, 0.0;
    _goal3_or << 0.0, 0.0, 0.0, 1.0;
    _goal4_pos << 0.0, 0.0, 0.0;
    _goal4_or << 0.0, 0.0, 0.0, 1.0;

    //Point 3
    _goal3_1_pos << 0.0, 0.0, 0.0;
    _goal3_1_or << 0.0, 0.0, 0.0, 1.0;
    _goal3_2_or << 0.0, 0.0, 0.0, 1.0;
    _goal3_2_pos << 0.0, 0.0, 0.0;
    _goal3_3_pos << 0.0, 0.0, 0.0;
    _goal3_3_or << 0.0, 0.0, 0.0, 1.0;
    _goal3_4_pos << 0.0, 0.0, 0.0;  //redundant
    _goal3_4_or << 0.0, 0.0, 0.0, 1.0;
    _goal3_5_pos << 0.0, 0.0, 0.0;
    _goal3_5_or << 0.0, 0.0, 0.0, 1.0;
    _goal3_6_pos << 0.0, 0.0, 0.0;
    _goal3_6_or << 0.0, 0.0, 0.0, 1.0;
    _goal3_7_pos << 0.0, 0.0, 0.0;  //redundant
    _goal3_7_or << 0.0, 0.0, 0.0, 1.0;
    _goal3_8_pos << 0.0, 0.0, 0.0;
    _goal3_8_or << 0.0, 0.0, 0.0, 1.0;

    //Point 4
    _goal_aruco_pos << 0.0, 0.0, 0.0;
    _goal_aruco_or << 0.0, 0.0, 0.0, 1.0;

}

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
    //ROS_INFO("aruco Position: %f %f %f", aruco_pose[0], aruco_pose[1], aruco_pose[2]);


}

void TF_NAV::tf_listener_fun()
{
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }
}

void TF_NAV::position_pub()
{
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];

    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];

    _position_pub.publish(pose);
}

void TF_NAV::goal_listener()
{
    ros::Rate r(1);
    tf::TransformListener listener;
    tf::StampedTransform transform1, transform2, transform3, transform4;
    tf::StampedTransform transform3_1, transform3_2, transform3_3, transform3_4, transform3_5, transform3_6, transform3_7, transform3_8;
    tf::StampedTransform transform_aruco;

    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform("map", "goal1", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal1", ros::Time(0), transform1);

            listener.waitForTransform("map", "goal2", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal2", ros::Time(0), transform2);

            listener.waitForTransform("map", "goal3", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal3", ros::Time(0), transform3);

            listener.waitForTransform("map", "goal4", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal4", ros::Time(0), transform4);


            listener.waitForTransform("map", "goal3_1", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal3_1", ros::Time(0), transform3_1);

            listener.waitForTransform("map", "goal3_2", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal3_2", ros::Time(0), transform3_2);

            listener.waitForTransform("map", "goal3_3", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal3_3", ros::Time(0), transform3_3);

            listener.waitForTransform("map", "goal3_4", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal3_4", ros::Time(0), transform3_4);

            listener.waitForTransform("map", "goal3_5", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal3_5", ros::Time(0), transform3_5);

            listener.waitForTransform("map", "goal3_6", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal3_6", ros::Time(0), transform3_6);

            listener.waitForTransform("map", "goal3_7", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal3_7", ros::Time(0), transform3_7);

            listener.waitForTransform("map", "goal3_8", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal3_8", ros::Time(0), transform3_8);

            listener.waitForTransform("map", "goal_aruco", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "goal_aruco", ros::Time(0), transform_aruco);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal1_pos << transform1.getOrigin().x(), transform1.getOrigin().y(), transform1.getOrigin().z();
        _goal1_or << transform1.getRotation().w(),  transform1.getRotation().x(), transform1.getRotation().y(), transform1.getRotation().z();

        _goal2_pos << transform2.getOrigin().x(), transform2.getOrigin().y(), transform2.getOrigin().z();
        _goal2_or << transform2.getRotation().w(),  transform2.getRotation().x(), transform2.getRotation().y(), transform2.getRotation().z();
        
        _goal3_pos << transform3.getOrigin().x(), transform3.getOrigin().y(), transform3.getOrigin().z();
        _goal3_or << transform3.getRotation().w(),  transform3.getRotation().x(), transform3.getRotation().y(), transform3.getRotation().z();
        
        _goal4_pos << transform4.getOrigin().x(), transform4.getOrigin().y(), transform4.getOrigin().z();
        _goal4_or << transform4.getRotation().w(),  transform4.getRotation().x(), transform4.getRotation().y(), transform4.getRotation().z();


        _goal3_1_pos << transform3_1.getOrigin().x(), transform3_1.getOrigin().y(), transform3_1.getOrigin().z();
        _goal3_1_or << transform3_1.getRotation().w(),  transform3_1.getRotation().x(), transform3_1.getRotation().y(), transform3.getRotation().z();

        _goal3_2_pos << transform3_2.getOrigin().x(), transform3_2.getOrigin().y(), transform3_2.getOrigin().z();
        _goal3_2_or << transform3_2.getRotation().w(),  transform3_2.getRotation().x(), transform3_2.getRotation().y(), transform3.getRotation().z();

        _goal3_3_pos << transform3_3.getOrigin().x(), transform3_3.getOrigin().y(), transform3_3.getOrigin().z();
        _goal3_3_or << transform3_3.getRotation().w(),  transform3_3.getRotation().x(), transform3_3.getRotation().y(), transform3.getRotation().z();

        _goal3_4_pos << transform3_4.getOrigin().x(), transform3_4.getOrigin().y(), transform3_4.getOrigin().z();
        _goal3_4_or << transform3_4.getRotation().w(),  transform3_4.getRotation().x(), transform3_4.getRotation().y(), transform3.getRotation().z();

        _goal3_5_pos << transform3_5.getOrigin().x(), transform3_5.getOrigin().y(), transform3_5.getOrigin().z();
        _goal3_5_or << transform3_5.getRotation().w(),  transform3_5.getRotation().x(), transform3_5.getRotation().y(), transform3.getRotation().z();

        _goal3_6_pos << transform3_6.getOrigin().x(), transform3_6.getOrigin().y(), transform3_6.getOrigin().z();
        _goal3_6_or << transform3_6.getRotation().w(),  transform3_6.getRotation().x(), transform3_6.getRotation().y(), transform3.getRotation().z();

        _goal3_7_pos << transform3_7.getOrigin().x(), transform3_7.getOrigin().y(), transform3_7.getOrigin().z();
        _goal3_7_or << transform3_7.getRotation().w(),  transform3_7.getRotation().x(), transform3_7.getRotation().y(), transform3.getRotation().z();

        _goal3_8_pos << transform3_8.getOrigin().x(), transform3_8.getOrigin().y(), transform3_8.getOrigin().z();
        _goal3_8_or << transform3_8.getRotation().w(),  transform3_8.getRotation().x(), transform3_8.getRotation().y(), transform3.getRotation().z();

        _goal_aruco_pos << transform_aruco.getOrigin().x(), transform_aruco.getOrigin().y(), transform_aruco.getOrigin().z();
        _goal_aruco_or << transform_aruco.getRotation().w(),  transform_aruco.getRotation().x(), transform_aruco.getRotation().y(), transform_aruco.getRotation().z();

        r.sleep();
    }    
}

void TF_NAV::send_goal()
{
    ros::Rate r(5);
    int cmd;
    move_base_msgs::MoveBaseGoal goal, goal1, goal2, goal3, goal4, goal3_1, goal3_2, goal3_3, goal3_4, goal3_5, goal3_6, goal3_7, goal3_8, goal_aruco;

    goal1.target_pose.header.frame_id = "map";
    goal2.target_pose.header.frame_id = "map";
    goal3.target_pose.header.frame_id = "map";
    goal4.target_pose.header.frame_id = "map";

    goal3_1.target_pose.header.frame_id = "map";
    goal3_2.target_pose.header.frame_id = "map";
    goal3_3.target_pose.header.frame_id = "map";
    goal3_4.target_pose.header.frame_id = "map";
    goal3_5.target_pose.header.frame_id = "map";
    goal3_6.target_pose.header.frame_id = "map";
    goal3_7.target_pose.header.frame_id = "map";
    goal3_8.target_pose.header.frame_id = "map";

    goal_aruco.target_pose.header.frame_id = "map";

    while ( ros::ok() )
    {
        std::cout<<"\nInsert 1 to send goal from TF "<<std::endl;
        std::cout<<"Insert 2 to send home position goal "<<std::endl;
        std::cout<<"Insert 3 to point 3 of the Homework "<<std::endl;
        std::cout<<"Insert 4 to point 4 of the Homework "<<std::endl;
        std::cout<<"Insert your choice"<<std::endl;
        std::cin>>cmd;

        if (cmd == 1)
        {
            MoveBaseClient ac("move_base", true);


            while(!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            //Goal 3
            goal3.target_pose.header.stamp = ros::Time::now();
            
            goal3.target_pose.pose.position.x = _goal3_pos[0];
            goal3.target_pose.pose.position.y = _goal3_pos[1];
            goal3.target_pose.pose.position.z = _goal3_pos[2];

            goal3.target_pose.pose.orientation.w = _goal3_or[0];
            goal3.target_pose.pose.orientation.x = _goal3_or[1];
            goal3.target_pose.pose.orientation.y = _goal3_or[2];
            goal3.target_pose.pose.orientation.z = _goal3_or[3];

            ROS_INFO("Sending goal3");
            std::cout<<"Goal 3 pose:  x: "<<_goal3_pos[0]<<", y: "<<_goal3_pos[1]<<", z: "<<_goal3_pos[2]<<std::endl;
            ac.sendGoal(goal3);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");

            //Goal 4
            goal4.target_pose.header.stamp = ros::Time::now();
            
            goal4.target_pose.pose.position.x = _goal4_pos[0];
            goal4.target_pose.pose.position.y = _goal4_pos[1];
            goal4.target_pose.pose.position.z = _goal4_pos[2];

            goal4.target_pose.pose.orientation.w = _goal4_or[0];
            goal4.target_pose.pose.orientation.x = _goal4_or[1];
            goal4.target_pose.pose.orientation.y = _goal4_or[2];
            goal4.target_pose.pose.orientation.z = _goal4_or[3];

            ROS_INFO("Sending goal4");
            std::cout<<"Goal 4 pose:  x: "<<_goal4_pos[0]<<", y: "<<_goal4_pos[1]<<", z: "<<_goal4_pos[2]<<std::endl;
            ac.sendGoal(goal4);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");

            //Goal 2
            goal2.target_pose.header.stamp = ros::Time::now();
            
            goal2.target_pose.pose.position.x = _goal2_pos[0];
            goal2.target_pose.pose.position.y = _goal2_pos[1];
            goal2.target_pose.pose.position.z = _goal2_pos[2];

            goal2.target_pose.pose.orientation.w = _goal2_or[0];
            goal2.target_pose.pose.orientation.x = _goal2_or[1];
            goal2.target_pose.pose.orientation.y = _goal2_or[2];
            goal2.target_pose.pose.orientation.z = _goal2_or[3];

            ROS_INFO("Sending goal2");
            std::cout<<"Goal 2 pose:  x: "<<_goal2_pos[0]<<", y: "<<_goal2_pos[1]<<", z: "<<_goal2_pos[2]<<std::endl;
            ac.sendGoal(goal2);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");

            //Goal 1
            goal1.target_pose.header.stamp = ros::Time::now();
            
            goal1.target_pose.pose.position.x = _goal1_pos[0];
            goal1.target_pose.pose.position.y = _goal1_pos[1];
            goal1.target_pose.pose.position.z = _goal1_pos[2];

            goal1.target_pose.pose.orientation.w = _goal1_or[0];
            goal1.target_pose.pose.orientation.x = _goal1_or[1];
            goal1.target_pose.pose.orientation.y = _goal1_or[2];
            goal1.target_pose.pose.orientation.z = _goal1_or[3];

            ROS_INFO("Sending goal1");
            std::cout<<"Goal 1 pose:  x: "<<_goal1_pos[0]<<", y: "<<_goal1_pos[1]<<", z: "<<_goal1_pos[2]<<std::endl;
            ac.sendGoal(goal1);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if (cmd == 2)
        {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;

            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if (cmd == 3)
        {
            MoveBaseClient ac("move_base", true);

            while(!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            //Goal 3_1
            goal3_1.target_pose.header.stamp = ros::Time::now();
            
            goal3_1.target_pose.pose.position.x = _goal3_1_pos[0];
            goal3_1.target_pose.pose.position.y = _goal3_1_pos[1];
            goal3_1.target_pose.pose.position.z = _goal3_1_pos[2];

            goal3_1.target_pose.pose.orientation.w = _goal3_1_or[0];
            goal3_1.target_pose.pose.orientation.x = _goal3_1_or[1];
            goal3_1.target_pose.pose.orientation.y = _goal3_1_or[2];
            goal3_1.target_pose.pose.orientation.z = _goal3_1_or[3];

            ROS_INFO("Sending goal3_1");
            std::cout<<"Goal 3_1 pose:  x: "<<_goal3_1_pos[0]<<", y: "<<_goal3_1_pos[1]<<", z: "<<_goal3_1_pos[2]<<std::endl;
            ac.sendGoal(goal3_1);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");
        
        
            //Goal 3_2
            goal3_2.target_pose.header.stamp = ros::Time::now();
            
            goal3_2.target_pose.pose.position.x = _goal3_2_pos[0];
            goal3_2.target_pose.pose.position.y = _goal3_2_pos[1];
            goal3_2.target_pose.pose.position.z = _goal3_2_pos[2];

            goal3_2.target_pose.pose.orientation.w = _goal3_2_or[0];
            goal3_2.target_pose.pose.orientation.x = _goal3_2_or[1];
            goal3_2.target_pose.pose.orientation.y = _goal3_2_or[2];
            goal3_2.target_pose.pose.orientation.z = _goal3_2_or[3];

            ROS_INFO("Sending goal3_2");
            std::cout<<"Goal 3_2 pose:  x: "<<_goal3_2_pos[0]<<", y: "<<_goal3_2_pos[1]<<", z: "<<_goal3_2_pos[2]<<std::endl;
            ac.sendGoal(goal3_2);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");


            //Goal 3_3
            goal3_3.target_pose.header.stamp = ros::Time::now();
            
            goal3_3.target_pose.pose.position.x = _goal3_3_pos[0];
            goal3_3.target_pose.pose.position.y = _goal3_3_pos[1];
            goal3_3.target_pose.pose.position.z = _goal3_3_pos[2];

            goal3_3.target_pose.pose.orientation.w = _goal3_3_or[0];
            goal3_3.target_pose.pose.orientation.x = _goal3_3_or[1];
            goal3_3.target_pose.pose.orientation.y = _goal3_3_or[2];
            goal3_3.target_pose.pose.orientation.z = _goal3_3_or[3];

            ROS_INFO("Sending goal3_3");
            std::cout<<"Goal 3_3 pose:  x: "<<_goal3_3_pos[0]<<", y: "<<_goal3_3_pos[1]<<", z: "<<_goal3_3_pos[2]<<std::endl;
            ac.sendGoal(goal3_3);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");


            //Goal 3_4
            // goal3_4.target_pose.header.stamp = ros::Time::now();
            
            // goal3_4.target_pose.pose.position.x = _goal3_4_pos[0];
            // goal3_4.target_pose.pose.position.y = _goal3_4_pos[1];
            // goal3_4.target_pose.pose.position.z = _goal3_4_pos[2];

            // goal3_4.target_pose.pose.orientation.w = _goal3_4_or[0];
            // goal3_4.target_pose.pose.orientation.x = _goal3_4_or[1];
            // goal3_4.target_pose.pose.orientation.y = _goal3_4_or[2];
            // goal3_4.target_pose.pose.orientation.z = _goal3_4_or[3];

            // ROS_INFO("Sending goal3_4");
            // std::cout<<"Goal 3_4 pose:  x: "<<_goal3_4_pos[0]<<", y: "<<_goal3_4_pos[1]<<", z: "<<_goal3_4_pos[2]<<std::endl;
            // ac.sendGoal(goal3_4);

            // ac.waitForResult();

            // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            // ROS_INFO("The mobile robot arrived in the TF goal");
            // else
            //     ROS_INFO("The base failed to move for some reason");


            //Goal 3_5
            goal3_5.target_pose.header.stamp = ros::Time::now();
            
            goal3_5.target_pose.pose.position.x = _goal3_5_pos[0];
            goal3_5.target_pose.pose.position.y = _goal3_5_pos[1];
            goal3_5.target_pose.pose.position.z = _goal3_5_pos[2];

            goal3_5.target_pose.pose.orientation.w = _goal3_5_or[0];
            goal3_5.target_pose.pose.orientation.x = _goal3_5_or[1];
            goal3_5.target_pose.pose.orientation.y = _goal3_5_or[2];
            goal3_5.target_pose.pose.orientation.z = _goal3_5_or[3];

            ROS_INFO("Sending goal3_5");
            std::cout<<"Goal 3_5 pose:  x: "<<_goal3_5_pos[0]<<", y: "<<_goal3_5_pos[1]<<", z: "<<_goal3_5_pos[2]<<std::endl;
            ac.sendGoal(goal3_5);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");


            //Goal 3_6
            goal3_6.target_pose.header.stamp = ros::Time::now();
            
            goal3_6.target_pose.pose.position.x = _goal3_6_pos[0];
            goal3_6.target_pose.pose.position.y = _goal3_6_pos[1];
            goal3_6.target_pose.pose.position.z = _goal3_6_pos[2];

            goal3_6.target_pose.pose.orientation.w = _goal3_6_or[0];
            goal3_6.target_pose.pose.orientation.x = _goal3_6_or[1];
            goal3_6.target_pose.pose.orientation.y = _goal3_6_or[2];
            goal3_6.target_pose.pose.orientation.z = _goal3_6_or[3];

            ROS_INFO("Sending goal3_6");
            std::cout<<"Goal 3_6 pose:  x: "<<_goal3_6_pos[0]<<", y: "<<_goal3_6_pos[1]<<", z: "<<_goal3_6_pos[2]<<std::endl;
            ac.sendGoal(goal3_6);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");


            //Goal 3_7
            // goal3_7.target_pose.header.stamp = ros::Time::now();
            
            // goal3_7.target_pose.pose.position.x = _goal3_7_pos[0];
            // goal3_7.target_pose.pose.position.y = _goal3_7_pos[1];
            // goal3_7.target_pose.pose.position.z = _goal3_7_pos[2];

            // goal3_7.target_pose.pose.orientation.w = _goal3_7_or[0];
            // goal3_7.target_pose.pose.orientation.x = _goal3_7_or[1];
            // goal3_7.target_pose.pose.orientation.y = _goal3_7_or[2];
            // goal3_7.target_pose.pose.orientation.z = _goal3_7_or[3];

            // ROS_INFO("Sending goal3_7 (redundant)");
            // std::cout<<"Goal 3_7 pose:  x: "<<_goal3_7_pos[0]<<", y: "<<_goal3_7_pos[1]<<", z: "<<_goal3_7_pos[2]<<std::endl;
            // ac.sendGoal(goal3_7);

            // ac.waitForResult();

            // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            // ROS_INFO("The mobile robot arrived in the TF goal");
            // else
            //     ROS_INFO("The base failed to move for some reason");

            //Goal 3_8
            goal3_8.target_pose.header.stamp = ros::Time::now();
            
            goal3_8.target_pose.pose.position.x = _goal3_8_pos[0];
            goal3_8.target_pose.pose.position.y = _goal3_8_pos[1];
            goal3_8.target_pose.pose.position.z = _goal3_8_pos[2];

            goal3_8.target_pose.pose.orientation.w = _goal3_8_or[0];
            goal3_8.target_pose.pose.orientation.x = _goal3_8_or[1];
            goal3_8.target_pose.pose.orientation.y = _goal3_8_or[2];
            goal3_8.target_pose.pose.orientation.z = _goal3_8_or[3];

            ROS_INFO("Sending goal3_8");
            std::cout<<"Goal 3_8 pose:  x: "<<_goal3_8_pos[0]<<", y: "<<_goal3_8_pos[1]<<", z: "<<_goal3_8_pos[2]<<std::endl;
            ac.sendGoal(goal3_8);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the TF goal");
            else
                ROS_INFO("The base failed to move for some reason");
        }
        else if(cmd == 4)
        {            
            ros::NodeHandle n;

            MoveBaseClient ac("move_base", true);

            while(!ac.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            //Goal Aruco
            goal_aruco.target_pose.header.stamp = ros::Time::now();
            
            goal_aruco.target_pose.pose.position.x = _goal_aruco_pos[0];
            goal_aruco.target_pose.pose.position.y = _goal_aruco_pos[1];
            goal_aruco.target_pose.pose.position.z = _goal_aruco_pos[2];

            goal_aruco.target_pose.pose.orientation.w = _goal_aruco_or[0];
            goal_aruco.target_pose.pose.orientation.x = _goal_aruco_or[1];
            goal_aruco.target_pose.pose.orientation.y = _goal_aruco_or[2];
            goal_aruco.target_pose.pose.orientation.z = _goal_aruco_or[3];
         
         
            ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);

            ROS_INFO("Sending goal_aruco");
            std::cout<<"Goal aruco pose:  x: "<<_goal_aruco_pos[0]<<", y: "<<_goal_aruco_pos[1]<<", z: "<<_goal_aruco_pos[2]<<std::endl;
            ac.sendGoal(goal_aruco);

            ac.waitForResult();
            bool arrived = false;
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                arrived = true;
                ROS_INFO("The mobile robot arrived in the TF goal");
            }
            else
                ROS_INFO("The base failed to move for some reason");

            if(aruco_pose_available && arrived){
                goal_aruco.target_pose.pose.position.x = aruco_pose[0]+1;
                goal_aruco.target_pose.pose.position.y = aruco_pose[1];
                goal_aruco.target_pose.pose.position.z = aruco_pose[2];

                goal_aruco.target_pose.pose.orientation.w = _goal_aruco_or[0];
                goal_aruco.target_pose.pose.orientation.x = _goal_aruco_or[1];
                goal_aruco.target_pose.pose.orientation.y = _goal_aruco_or[2];
                goal_aruco.target_pose.pose.orientation.z = _goal_aruco_or[3];


                ROS_INFO("Positioning 1 meter from Aruco");
                ac.sendGoal(goal_aruco);

                ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){

                ROS_INFO("The mobile robot is in front of the Aruco");
                
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(aruco_pose[0],aruco_pose[1],aruco_pose[2]));
                tf::Quaternion q(aruco_pose[3],aruco_pose[4],aruco_pose[5],aruco_pose[6]);
                transform.setRotation(q);

                // Pubblica la trasformazione TF
                static tf::TransformBroadcaster br;
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "aruco_marker_frame"));
            }
            else
                ROS_INFO("The base failed to move for some reason");

            }
        }
        else 
        {
            ROS_INFO("Wrong input!");
        }

        r.sleep();
    }
}

void TF_NAV::run() {
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );
    boost::thread tf_listener_goal_t( &TF_NAV::goal_listener, this );
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
    ros::spin();
}

int main( int argc, char** argv ) {
    ros::init(argc, argv, "tf_navigation");
    TF_NAV tfnav;
    tfnav.run();

    return 0;
}