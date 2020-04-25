#include <stdio.h>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

#define SIMULATION

void models_callback(const gazebo_msgs::ModelStates &models_msgs)
{
    int n_models = models_msgs.name.size();

    for(int i = 0; i < n_models; i++){
        std::cout << models_msgs.name[i] << " "
        << models_msgs.pose[i] << std::endl;
    }
    std::cout << " " << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_sub_model_states");
    ros::NodeHandle nh;


#ifdef SIMULATION
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 
                                        10,
                                        models_callback);
#else
#endif


    ros::spin();
    return 0;
}