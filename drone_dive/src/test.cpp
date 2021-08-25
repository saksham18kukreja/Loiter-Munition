#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

mavros_msgs::AttitudeTarget angle_correct;
geometry_msgs::PoseStamped pose_correct;
mavros_msgs::State state;

void callback(const mavros_msgs::AttitudeTarget::ConstPtr& data){
    angle_correct = *data;
    //printf("the values x camera %f\n",angle_correct.orientation.x);
}

// void hello_drone(){
//     state = ros::topic::waitForMessage("/mavros/state",mavros_msgs::State);
//     printf("the values x %f\n",angle_correct.orientation.x);
//     printf("the values y %f\n",angle_correct.orientation.y);
//     printf("the values z %f\n",angle_correct.orientation.z);

// }

void pos_callback(const geometry_msgs::PoseStamped::ConstPtr& data){
    pose_correct = *data;
    printf("hello \n");
    printf("the values x camera %f\n",angle_correct.orientation.x);
    printf("the values x %f\n",pose_correct.pose.position.x);
   
    //printf("the values y %f\n",pose_correct.pose.orientation.y);
    //printf("the values z %f\n",pose_correct.pose.orientation.z);
    //angle_correct = *ros::topic::waitForMessage<mavros_msgs::AttitudeTarget>("/plane_cam/usb_cam/att_correction");
    printf("the values x %f\n",angle_correct.orientation.x);

}

// void state_callback(const mavros_msgs::State::ConstPtr& msg){
//     state = *msg;
//     printf("%b",state.armed);
// }

int main(int argc, char **argv){
    ros::init(argc,argv,"hello_drone");
    ros::NodeHandle nh;
    ros::Rate rate = 20;
    while(ros::ok()){
        ros::Subscriber att = nh.subscribe<mavros_msgs::AttitudeTarget>("/plane_cam/usb_cam/att_correction",10,callback);
        ros::Subscriber pos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",1,pos_callback);
        //ros::Subscriber st = nh.subscribe<mavros_msgs::State>("/mavros/state",1,state_callback);
        //hello_drone();
        
        ros::spinOnce();
        rate.sleep(); 
    }

}