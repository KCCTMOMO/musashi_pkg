#include <stdio.h>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include "Data/Data.h"
#include "Behavior/Behavior.h"

#define SIMULATION

//Callback functions
void models_callback(const gazebo_msgs::ModelStates &models_msgs);

Data *data;
Behavior *behavior;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behavior");
    ros::NodeHandle nh;
    ros::Rate loop_rate(33);

    data = new Data();
    behavior = new Behavior();

#ifdef SIMULATION
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 3, models_callback);
#else
#endif

    data->initTime();

    while(ros::ok()){
        data->updateTime();

        //call Behavior main function
        behavior->main(data);

        //publish?

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void models_callback(const gazebo_msgs::ModelStates &models_msgs)
{
    //get number of models existing in gazebo world
    int n_models = models_msgs.name.size();

    for(int i = 0; i < n_models; i++){

        std::string name = models_msgs.name[i];
        geometry_msgs::Pose pose = models_msgs.pose[i];
        geometry_msgs::Twist twist = models_msgs.twist[i];
        // std::cout << name << std::endl << pose << std::endl;
        
        if(name == "musashi_player"){
            // std::cout << name << std::endl << pose << std::endl;

            // data->X.pos.x() = pose.position.x;
            // data->X.pos.y() = pose.position.y;
            data->X.pose.x = pose.position.x;
            data->X.pose.y = pose.position.y;

            //Quaternion to Euler angle
            tf::Quaternion q;
            double r,p,y;
            tf::quaternionMsgToTF(pose.orientation, q);
            tf::Matrix3x3(q).getRPY(r, p, y);
            // data->X.theta = y;
            data->X.pose.theta = y;

            // data->X.v.x() = twist.linear.x;
            // data->X.v.y() = twist.linear.y;
            // data->X.omega = twist.angular.z;    //rotation velocity around Z axis
            data->X.velo.x = twist.linear.x;
            data->X.velo.y = twist.linear.y;
            data->X.velo.omega = twist.angular.z;

            /*
            std::cout << data->X.pose.x << " "
            << data->X.pose.y << " "
            << data->X.pose.theta << std::endl;
            */
        }

        if(name == "football"){
            data->Ball.pose.x = pose.position.x;
            data->Ball.pose.y = pose.position.y;

            //Quaternion to Euler angle
            tf::Quaternion q;
            double r,p,y;
            tf::quaternionMsgToTF(pose.orientation, q);
            tf::Matrix3x3(q).getRPY(r, p, y);
            data->Ball.pose.theta = y;

            data->Ball.velo.x = twist.linear.x;
            data->Ball.velo.y = twist.linear.y;
            data->Ball.velo.omega = twist.angular.z;
        }
    }
    // std::cout << std::endl;

 
}