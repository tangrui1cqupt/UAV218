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
#include <string.h>

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
    ros::init(argc, argv, "ego_to_mavros");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    float TAKEOFF_HEIGHT;
    float INIT_X,INIT_Y,INIT_Z;
    float GOAL_X,GOAL_Y,GOAL_Z;
    //std::string POS_CMD_TOPIC_NAME;

    nh1.param<float>("takeoff_height", TAKEOFF_HEIGHT, 0.8);
    //nh1.param<float>("init_x", INIT_X, 0);
    //nh1.param<float>("init_y", INIT_Y, 0);
    //nh1.param<float>("init_z", INIT_Z, 0);
    //nh1.param<float>("goal_x", GOAL_X, 2);
    //nh1.param<float>("goal_y", GOAL_Y, 0);
    //nh1.param<float>("goal_z", GOAL_Z, 0.8);
    //nh1.param<std::string>("pos_cmd_topic_name", POS_CMD_TOPIC_NAME, "/drone_0_planning/pos_cmd");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber ego_pos_sub = nh.subscribe<offboard_pkg::PositionCommand>
            ("planning/pos_cmd", 10, ego_pos_cb);
    //ros::Publisher ego_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    //ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    //ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //SubscribeAndPublish SAPObject;

    //the setpoint publishing rate MUST be faster than 2Hz
    //ros::Rate rate(20.0);
    ros::Rate rate(30.0);

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

    mavros_msgs::PositionTarget setpoint_raw_local;


    ros::Time last_request = ros::Time::now();
    ros::Time time_snap1 = ros::Time::now();

    while(ros::ok()){


        //if(flag == 1)
        //{

        //while(setpoint_raw_local_pre_pub.getNumSubscribers()<1)
        
        //if( position[2] < 0.1 )
        //if( position[2] < 0.3 )
        //{
    if(cur_position[2] < (TAKEOFF_HEIGHT - 0.2))
    {
       time_snap1 = ros::Time::now();
    }

        //起飞
        if(state_flag == 0)
        {
        ROS_INFO("state_flag is 0");

        setpoint_raw_local.type_mask =  0b100111111000; 
        setpoint_raw_local.coordinate_frame = 1;
        //setpoint_raw_local.position.x = 0;
        //setpoint_raw_local.position.y = 0;
        //setpoint_raw_local.position.z = 0.8;
        setpoint_raw_local.position.x = 0;
        setpoint_raw_local.position.y = 0;
        setpoint_raw_local.position.z = TAKEOFF_HEIGHT;
        //setpoint_raw_local.position.z = 0.3;
        setpoint_raw_local.yaw = 0;
        ROS_INFO("send message");
        setpoint_raw_local_pub.publish(setpoint_raw_local);

        if((abs(cur_position[0]) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2] - TAKEOFF_HEIGHT)) < 0.1)&&(ego_poscmd_count > 1))
         {    
                state_flag = 1;
         }

        }
        //flag = 2;
        //}

        
        //if( (ego_poscmd_count > 0)&&(position[2] >= 0.1) )
        //if( (ego_poscmd_count > 0)&&(position[2] >= 0.3) )
        //{
        if(state_flag == 1)
        {
        ROS_INFO("state_flag is 1");
        ROS_INFO("send ego pos cmd");
        setpoint_raw_local.type_mask =  0b100111111000; 
        setpoint_raw_local.coordinate_frame = 1;
        setpoint_raw_local.position.x = ego_poscmd[0];
        setpoint_raw_local.position.y = ego_poscmd[1];
        setpoint_raw_local.position.z = ego_poscmd[2];
        setpoint_raw_local.yaw = yaw_set;
        setpoint_raw_local_pub.publish(setpoint_raw_local);
        }


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


