#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>

#include "Behavior/Behavior.h"
#include "Data/Data.h"

#define SIMULATION

// Callback functions
void coachbox_callback(const std_msgs::String &comm);
void models_callback(const gazebo_msgs::ModelStates &models_msgs);

Data *data;
Behavior *behavior;

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior");
  ros::NodeHandle nh;
  ros::Rate loop_rate(33);

  data = new Data();
  behavior = new Behavior();

  ros::Subscriber sub_coachbox_comm =
      nh.subscribe("coachbox/command", 3, coachbox_callback);

  ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Publisher pub_dw = nh.advertise<std_msgs::Float32MultiArray>("/dw", 10);

#ifdef SIMULATION
  ros::Subscriber sub =
      nh.subscribe("/gazebo/model_states", 3, models_callback);
#else
#endif

  data->initTime();

  while (ros::ok()) {
    data->updateTime();

    // call Behavior main function
    behavior->main(data);

    // Publish control input
    geometry_msgs::Twist msg;
    msg.linear.x = data->Uin.x;
    msg.linear.y = data->Uin.y;
    msg.linear.z = 0.0;
    msg.angular.x = msg.angular.y = 0.0;
    msg.angular.z = data->Uin.omega;

    pub_cmd.publish(msg);


    // Publish dynamic window array
    std_msgs::Float32MultiArray dw_msgs;
    for(int i = 0; i < data->Uarray.size(); i++){
      dw_msgs.data.push_back(data->Uarray[i].x);
    }
    pub_dw.publish(dw_msgs);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void coachbox_callback(const std_msgs::String &comm) {

  ROS_INFO("Coachbox comm: %s", comm.data.c_str());
  KeyCoachBoxCmdAndColor key = {comm.data, data->color};
  data->action = CoachboxCommand::mapToAct.at(key);
  ROS_INFO("Set action: %s", Action::str.at(data->action).c_str());
  return;
}

void models_callback(const gazebo_msgs::ModelStates &models_msgs) {
  // get number of models existing in gazebo world
  int n_models = models_msgs.name.size();

  for (int i = 0; i < n_models; i++) {

    std::string name = models_msgs.name[i];
    geometry_msgs::Pose pose = models_msgs.pose[i];
    geometry_msgs::Twist twist = models_msgs.twist[i];

    if (name == "musashi_player") {
      data->X.pose.x = pose.position.x;
      data->X.pose.y = pose.position.y;

      // Quaternion to Euler angle
      tf::Quaternion q;
      double r, p, y;
      tf::quaternionMsgToTF(pose.orientation, q);
      tf::Matrix3x3(q).getRPY(r, p, y);
      data->X.pose.theta = y;

      data->X.velo.x = twist.linear.x;
      data->X.velo.y = twist.linear.y;
      data->X.velo.omega = twist.angular.z;
    }

    if (name == "football") {
      data->Ball.pose.x = pose.position.x;
      data->Ball.pose.y = pose.position.y;

      // Quaternion to Euler angle
      tf::Quaternion q;
      double r, p, y;
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
