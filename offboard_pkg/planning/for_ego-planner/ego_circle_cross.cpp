/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <offboard_pkg/PositionCommand.h>


#include "math_utils.h"


mavros_msgs::State current_state;

int flag = 1;

double cur_position[3]={0,0,0};
double ego_poscmd[3]={0,0,0};
double quaternion_x;
double quaternion_y;
double quaternion_z;
double quaternion_w;
double yaw_set;

int ego_poscmd_count = 0;

int state_flag = 0;

float current_roll;
float current_pitch;
float current_yaw;



/*******************************************************************************************************************
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    setpoint_raw_local_pre_pub = n_.advertise<mavros_msgs::PositionTarget>("/setpoint_raw/local_pre", 10);
 
    //Topic you want to subscribe
    ego_pos_sub = n_.subscribe("/planning/pos_cmd", 1, &SubscribeAndPublish::callback, this);
  }
 
  void callback(const offboard_pkg::PositionCommand::ConstPtr& msg)
  {
            position[0] = msg->position.x;
            position[1] = msg->position.y;
            position[2] = msg->position.z;
            yaw_set = msg->yaw;
            
            mavros_msgs::PositionTarget setpoint_raw_local;
            setpoint_raw_local.type_mask =  0b100111111000; 
            setpoint_raw_local.coordinate_frame = 1;
            setpoint_raw_local.position.x = position[0];
            setpoint_raw_local.position.y = position[1];
            setpoint_raw_local.position.z = position[2];
            setpoint_raw_local.yaw = yaw_set;

            setpoint_raw_local_pre_pub.publish(setpoint_raw_local);
  }
 
private:
  ros::NodeHandle n_; 
  ros::Publisher setpoint_raw_local_pre_pub;
  ros::Subscriber ego_pos_sub;
 
};
**************************************************************************************************************/



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
            cur_position[0] = msg->pose.position.x;
            cur_position[1] = msg->pose.position.y;
            cur_position[2] = msg->pose.position.z;

            //排序是 roll,pitch,yaw
            Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

            Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
            current_roll = euler_fcu[0];
            current_pitch = euler_fcu[1];
            current_yaw = euler_fcu[2];

}

void ego_pos_cb(const offboard_pkg::PositionCommand::ConstPtr& msg){
            ego_poscmd[0] = msg->position.x;
            ego_poscmd[1] = msg->position.y;
            ego_poscmd[2] = msg->position.z;
            yaw_set = msg->yaw;
           
            ego_poscmd_count = ego_poscmd_count + 1;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_circle_cross");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber ego_pos_sub = nh.subscribe<offboard_pkg::PositionCommand>
            ("/planning/pos_cmd", 10, ego_pos_cb);


    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::Publisher ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    //ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //SubscribeAndPublish SAPObject;

    //the setpoint publishing rate MUST be faster than 2Hz
    //ros::Rate rate(20.0);
    ros::Rate rate(30.0); //这是30hz应该是考虑到ego输出的期望位姿频率

    // wait for FCU connection
    /******************************************************
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ********************************************************/

    /***********************************
    geometry_msgs::PoseStamped goal_pos;
    goal_pos.pose.position.x = 3;
    goal_pos.pose.position.y = 0;
    goal_pos.pose.position.z = 0.7;
    goal_pos.pose.orientation.x = 0;
    goal_pos.pose.orientation.y = 0;
    goal_pos.pose.orientation.z = 0;
    goal_pos.pose.orientation.w = 1;
    ***********************************/

    //mavros_msgs::PositionTarget setpoint_raw_local;


    ros::Time last_request = ros::Time::now();

    ros::Time time_snap1 = ros::Time::now();



    while(ros::ok()){

    mavros_msgs::PositionTarget pos_setpoint;

    if(cur_position[2] < 0.7)
    {
       time_snap1 = ros::Time::now();
    }

    //起飞
    if(state_flag == 0)
    {
    ROS_INFO("state_flag is 0");
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = 0;
    pos_setpoint.position.y = 0;
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = 1.0;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //定点悬停多少秒之后再进入下一个状态进行圆跟踪  10秒
    if((abs(cur_position[0]) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-1.0)) < 0.1)&&(ros::Time::now() - time_snap1 > ros::Duration(7.0)))
      {
           geometry_msgs::PoseStamped ego_goal_pos;
           ego_goal_pos.pose.position.x = 2.5;
           ego_goal_pos.pose.position.y = 0;
           ego_goal_pos.pose.position.z = 0.7;
           ego_goal_pos.pose.orientation.x = 0;
           ego_goal_pos.pose.orientation.y = 0;
           ego_goal_pos.pose.orientation.z = 0;
           ego_goal_pos.pose.orientation.w = 1;
           ego_goal_pub.publish(ego_goal_pos);
           if(ego_poscmd_count > 1)
            {
              state_flag = 1;
            }
      }
    }


    if(state_flag == 1)
    {
        ROS_INFO("state_flag is 1");
        pos_setpoint.type_mask =  0b100111111000; 
        pos_setpoint.coordinate_frame = 1;
        pos_setpoint.position.x = ego_poscmd[0];
        pos_setpoint.position.y = ego_poscmd[1];
        pos_setpoint.position.z = ego_poscmd[2];
        pos_setpoint.yaw = yaw_set;
        setpoint_raw_local_pub.publish(pos_setpoint);
    }


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


