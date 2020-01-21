#ifndef NAVIGATION_TIAGO_H
#define NAVIGATION_TIAGO_H

#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>
#include <map>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <perception_msgs/arrived.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
namespace NAVIGATIONTIAGO{
class Navigation_tiago
{
    private:


        // this is for gazebo simulation!!!!!
        // double Targetx[3] = {-1.8,0.029,0.0};
        // double Targety[3] = {2.4,2.4,0.0};
        // double Targetz[3] = {0.7071,0.7071,0.7071};
        // double Targetw[3] = {0.7071,0.7071,0.7071};
        // char Target[3] = {'A', 'B','C'};

        ros::NodeHandle n;
        ros::NodeHandle nh_;
        ros::NodeHandle priv_nh_;
        perception_msgs::arrived success;
        geometry_msgs::Twist msg;
        // std::string msg1;

        move_base_msgs::MoveBaseGoal goal;
        std_srvs::Empty srv;
        std::map<std::string, double> lift_position;
        std::map<std::string, double> home_position;
        std::map<std::string, double> ready_position;
        std::map<std::string, double> drop_pose;


        ros::ServiceClient client_loc = n.serviceClient<std_srvs::Empty>("/global_localization");
        ros::ServiceClient client_clear = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 100);
        ros::Publisher pub_success = n.advertise<perception_msgs::arrived>("/arrived", 100);
        
        ros::ServiceClient client_grasp2 = n.serviceClient<std_srvs::Empty>("/move_1");
        ros::ServiceClient client_grasp3 = n.serviceClient<std_srvs::Empty>("/gripper_controller/grasp");
        ros::ServiceClient client_grasp1 = n.serviceClient<std_srvs::Empty>("/move_rectify");

        int movearm(std::map<std::string, double> target_position);
        move_base_msgs::MoveBaseGoal Next_point(int id);
        



    public:
    // double Targetx[3] = {2.5893,3.726,0.070};
    // double Targety[3] = {-0.822,-0.963,0.811};
    // double Targetz[3] = {-0.745,-0.745,-0.745};
    // double Targetw[3] = {0.667,0.667,0.667};
    // char Target[3] = {'A', 'B','C'};

    //double Targetx[3] = {-0.748,3.726,0.070};
    //double Targety[3] = {-1.123,-0.963,0.811};
    //double Targetz[3] = {-0.745,-0.745,-0.745};
    //double Targetw[3] = {0.667,0.667,0.667};
    //char Target[3] = {'A', 'B','C'};

    double Targetx[3] = {-1.819,-0.707,-1.747};
    double Targety[3] = {-1.752,-1.233,-1.940};
    double Targetz[3] = {-0.745,-0.707,-0.691};
    double Targetw[3] = {0.667,0.707,0.723};
    char Target[3] = {'A', 'B','C'};

    // ros::Subscriber sub_left = n.subscribe("table", 1000, &NAVIGATIONTIAGO::Navigation_tiago::TableCallback,this);

    // void TableCallback(const std_msgs::String::ConstPtr& msg_table)
    // {
    //     msg1 = msg_table->data.c_str();
    // }


    Navigation_tiago(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

        ros::Rate r(60);
        
        //define positions
        lift_position["torso_lift_joint"] = 0.20;
        lift_position["arm_1_joint"] = 0.44;
        lift_position["arm_2_joint"] = -0.01;
        lift_position["arm_3_joint"] = -1.64;
        lift_position["arm_4_joint"] = 2.08;
        lift_position["arm_5_joint"] = -1.73;
        lift_position["arm_6_joint"] = -0.94;
        lift_position["arm_7_joint"] = -1.68;


        home_position["torso_lift_joint"] = 0.15;
        home_position["arm_1_joint"] = 0.20;
        home_position["arm_2_joint"] = -1.34;
        home_position["arm_3_joint"] = -0.2;
        home_position["arm_4_joint"] = 1.96;
        home_position["arm_5_joint"] = -1.57;
        home_position["arm_6_joint"] = 1.37;
        home_position["arm_7_joint"] = 0.00;

        
        //tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
        }
        //call the service to localize Tiago and clear cost map

        ros::AsyncSpinner spinner(3);
        spinner.start();

        ros::Duration(5).sleep();
        ROS_INFO("Localize TIAGo");
        client_loc.call(srv);
        system("rosrun play_motion move_joint head_2_joint -0.45 3");

        ros::Duration(5).sleep();
        ROS_INFO("Rotate the robot");    
        msg.angular.z = 60*3.14;
        for(double t0=0;t0<100;t0=t0+0.1){
            pub.publish(msg);
            ros::spinOnce();
            r.sleep();
        }
        //clear costmaps
        ros::Duration(5).sleep();
        ROS_INFO("Clear costmaps");
        client_clear.call(srv);
        ros::Duration(2).sleep();

        //go to point
        while (ros::ok()){
            while(true){
                //go to point 0
                // ROS_INFO("%s",msg1);
                // if(msg1 == "left")
                // {
                //     goal = Next_point(0);
                //     ROS_INFO("Sending goal");
                //     ac.sendGoal(goal);
                //     ac.waitForResult();

                //     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                //         {ROS_INFO("The Robot has achieved Point %c",Target[0]);
                //         success.x = (bool) 1;
                //         pub_success.publish(success);
                //         break;}
                //     else
                //         ROS_INFO("The Robot fail to achieve Point %c",Target[0]);
                // }
                goal = Next_point(1);
                ROS_INFO("Sending goal");
                ac.sendGoal(goal);
                ac.waitForResult();

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {ROS_INFO("The Robot has achieved Point %c",Target[0]);
                    success.x = (bool) 1;
                    pub_success.publish(success);
                    break;}
                else
                    ROS_INFO("The Robot fail to achieve Point %c",Target[0]);
            }
            ros::Duration(5).sleep();

            movearm(lift_position);
            // fix the accurate place to grasp in y aixs

            client_grasp1.call(srv);
            // go to grasp position

            // call the grasp service
            if (client_grasp2.call(srv))
            {
                ROS_INFO("you can start grasp the bottle ");
                ros::Duration(2).sleep();
                client_grasp3.call(srv);            
            }
            else
            {
                ROS_ERROR("Failed to call service ");
            }
            ros::Duration(2).sleep();

            // // after grasping first rotate a little bit
            // // msg.angular.z = 60*3.14;
            // // for(double t0=0;t0<5;t0=t0+0.1){
            // //     ROS_INFO("Rotate the robot"); 
            // //     pub.publish(msg);
            // //     ros::spinOnce();
            // //     r.sleep();
            // // }

            ////////////////////////////////////////////////////////////
            // then move backward a little to assure robot action wont touch table
            msg.linear.x = -0.3;
            msg.angular.z = 0;
            for(double t0=0;t0<8;t0=t0+0.1){
                ROS_INFO("backward"); 
                pub.publish(msg);
                ros::spinOnce();
                r.sleep();
            }

            // go back to home position
            movearm(home_position);
            // go to point 1
            while(true){
            goal = Next_point(2);
            ROS_INFO("Sending goal");
            ac.sendGoal(goal);
            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {ROS_INFO("The Robot has achieved Point %c",Target[1]);
                ros::Duration(3).sleep();
                // first lift position
                // movearm(lift_position);
                // ros::Duration(3).sleep();

                //// then drop position
                // // movearm(drop_pose);
                // // // then move forward a little to assure bottle can be on table
                // // msg.linear.x = 0.5;
                // // msg.angular.z = 0;
                // // for(double t0=0;t0<2;t0=t0+0.1){
                // //     ROS_INFO("foward"); 
                // //     pub.publish(msg);
                // //     ros::spinOnce();
                // //     r.sleep();
                // // }

                // call service to loose the gripper
                system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");
                // after droping sleep 300 s
                ros::Duration(300).sleep();

                }
            else
                ROS_INFO("The Robot fail to achieve Point %c",Target[1]);

            }

        spinner.stop();
        r.sleep();
    }

    };
    ~Navigation_tiago() {}
};


}

#endif
