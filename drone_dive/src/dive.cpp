#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <math.h>


mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_position;
double pitch_angle=0;
mavros_msgs::AttitudeTarget attitude;
tf2::Quaternion myq;
ros::Publisher attitude_pub;
geometry_msgs::PoseStamped pose;
ros::Publisher local_pos_pub;
bool find_pitch=true;
mavros_msgs::AttitudeTarget camera_correct;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void camera_correction(const mavros_msgs::AttitudeTarget::ConstPtr& msg){
    camera_correct = *msg;
    printf("print check %f \n",camera_correct.orientation.x);
}



void calculate_pitch(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
    
    double radius = sqrt(pow(local_position.pose.position.x,2)+pow(local_position.pose.position.y,2));

    if(radius>80){
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = local_position.pose.position.z;
        //printf("the value of setpoint %f \n",(double)pose.pose.position.z);
        local_pos_pub.publish(pose);
    }
    
    else{

        if(find_pitch){
            pitch_angle = atan(local_position.pose.position.z/radius) + 0.18;
            find_pitch=false;
        }

        //ROS_INFO("pitch angle is %f",(double)pitch_angle);
        //ROS_INFO("pitch angle is %f",local_position.pose.orientation.y);

        camera_correct = *ros::topic::waitForMessage<mavros_msgs::AttitudeTarget>("/plane_cam/usb_cam/att_correction");

        myq.setRPY(0,pitch_angle,0);
        myq.normalize();
        attitude.orientation.x = /*myq[0] +*/ camera_correct.orientation.x;
        attitude.orientation.y = /*myq[1] +*/ camera_correct.orientation.y;
        attitude.orientation.z = /*myq[2] +*/ camera_correct.orientation.z;
        attitude.orientation.w = /*myq[3] +*/ camera_correct.orientation.w;
        attitude.thrust = 0.1;
        attitude.body_rate.x = 10;
        attitude.body_rate.y = 10;
        attitude.body_rate.z = 10;
        ROS_INFO("pitch angle is %f",(double)attitude.orientation.x);
        attitude_pub.publish(attitude);    
    }

    

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    //tf2::Quaternion myq;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",1,calculate_pitch);
    

    mavros_msgs::AttitudeTarget attitude;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
       ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",1,calculate_pitch);
       //ros::Subscriber camera_att_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/plane_cam/usb_cam/att_correction",1,camera_correction);
       
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        

        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
