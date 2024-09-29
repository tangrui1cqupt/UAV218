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

double TAKEOFF_HEIGHT;

double GOAL1_X,GOAL1_Y,GOAL1_Z,GOAL2_X,GOAL2_Y,GOAL2_Z,GOAL3_X,GOAL3_Y,GOAL3_Z,GOAL4_X,GOAL4_Y,GOAL4_Z;
float GOAL1_YAW,GOAL2_YAW,GOAL3_YAW,GOAL4_YAW;   //单位为度
float GOAL1_YAW_RAD,GOAL2_YAW_RAD,GOAL3_YAW_RAD,GOAL4_YAW_RAD;  //单位为弧度

float pai = 3.1416;

//double goal1[3]={2,0,1};
//double goal2[3]={2,2,1};
//double goal3[3]={0,2,1};
//double goal4[3]={0,0,1};

int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;

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
    ros::init(argc, argv, "ego_four_goals");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    nh1.param<double>("takeoff_height", TAKEOFF_HEIGHT, 1);

    nh1.param<double>("goal1_x", GOAL1_X, 2);
    nh1.param<double>("goal1_y", GOAL1_Y, 0);
    nh1.param<double>("goal1_z", GOAL1_Z, 1);
    nh1.param<float>("goal1_yaw", GOAL1_YAW, 0);

    nh1.param<double>("goal2_x", GOAL2_X, 2);
    nh1.param<double>("goal2_y", GOAL2_Y, 2);
    nh1.param<double>("goal2_z", GOAL2_Z, 1);
    nh1.param<float>("goal2_yaw", GOAL2_YAW, 90);

    nh1.param<double>("goal3_x", GOAL3_X, 0);
    nh1.param<double>("goal3_y", GOAL3_Y, 2);
    nh1.param<double>("goal3_z", GOAL3_Z, 1);
    nh1.param<float>("goal3_yaw", GOAL3_YAW, 180);

    nh1.param<double>("goal4_x", GOAL4_X, 0);
    nh1.param<double>("goal4_y", GOAL4_Y, 0);
    nh1.param<double>("goal4_z", GOAL4_Z, 1);
    nh1.param<float>("goal4_yaw", GOAL4_YAW, 270);


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
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


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


    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    double goal1[3]={GOAL1_X,GOAL1_Y,GOAL1_Z};
    double goal2[3]={GOAL2_X,GOAL2_Y,GOAL2_Z};
    double goal3[3]={GOAL3_X,GOAL3_Y,GOAL3_Z};
    double goal4[3]={GOAL4_X,GOAL4_Y,GOAL4_Z};

    ros::Time last_request = ros::Time::now();

    ros::Time time_snap1 = ros::Time::now();



    while(ros::ok()){

    mavros_msgs::PositionTarget pos_setpoint;

    if(cur_position[2] < (TAKEOFF_HEIGHT - 0.2))
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
    //pos_setpoint.position.z = 1.0;
    pos_setpoint.position.z = TAKEOFF_HEIGHT;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //定点悬停多少秒之后再进入下一个状态进行圆跟踪  10秒
    if((abs(cur_position[0]) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2] - TAKEOFF_HEIGHT)) < 0.1)&&(ros::Time::now() - time_snap1 > ros::Duration(7.0)))
      {
           geometry_msgs::PoseStamped ego_goal_pos;
           ego_goal_pos.pose.position.x = goal1[0];
           ego_goal_pos.pose.position.y = goal1[1];
           ego_goal_pos.pose.position.z = goal1[2];
           //下面四元数代表的是，偏航0度
           //ego_goal_pos.pose.orientation.x = 0;
           //ego_goal_pos.pose.orientation.y = 0;
           //ego_goal_pos.pose.orientation.z = 0;
           //ego_goal_pos.pose.orientation.w = 1;
           
           //度数 * (π / 180） = 弧度 ，下面将度转为弧度
            GOAL1_YAW_RAD = GOAL1_YAW * (pai/180);
           //下面是将欧拉角转为四元数的代码，这里欧拉角的单位是弧度，如果是角度，先转为弧度再用下面代码转为四元数。单位为弧度的欧拉角，注意一个细节易错点，就是在欧拉角转四元数的计算的最开始是先除以一个2，这个容易漏掉。这里三个欧拉角(单位弧度),roll和pitch是0,yaw是yaw_set，转出的四元数是quaternion_x，quaternion_y，quaternion_z，quaternion_w。
            float p = 0;
            float y = GOAL1_YAW_RAD/2;
            float r = 0;

            float sinp = sin(p);
            float siny = sin(y);
            float sinr = sin(r);
            float cosp = cos(p);
            float cosy = cos(y);
            float cosr = cos(r);

            quaternion_x = sinr * cosp * cosy - cosr * sinp * siny;
            quaternion_y = cosr * sinp * cosy + sinr * cosp * siny;
            quaternion_z = cosr * cosp * siny - sinr * sinp * cosy;
            quaternion_w = cosr * cosp * cosy + sinr * sinp * siny;

           ego_goal_pos.pose.orientation.x = quaternion_x;
           ego_goal_pos.pose.orientation.y = quaternion_y;
           ego_goal_pos.pose.orientation.z = quaternion_z;
           ego_goal_pos.pose.orientation.w = quaternion_w;
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

        if((abs(cur_position[0] - goal1[0]) < 0.1)&&(abs(cur_position[1] - goal1[1]) < 0.1)&&((abs(cur_position[2] - goal1[2])) < 0.1))
        {
           count1 = count1 + 1;
           //大约悬停5秒钟，150/30
           if(count1 > 150)
           {
           geometry_msgs::PoseStamped ego_goal_pos;
           ego_goal_pos.pose.position.x = goal2[0];
           ego_goal_pos.pose.position.y = goal2[1];
           ego_goal_pos.pose.position.z = goal2[2];
           //下面四元数代表的是，偏航90度，俯仰，横滚都是0度，可去这个网站转换https://quaternions.online/
           //ego_goal_pos.pose.orientation.x = 0.707;
           //ego_goal_pos.pose.orientation.y = 0;
           //ego_goal_pos.pose.orientation.z = 0;
           //ego_goal_pos.pose.orientation.w = 0.707;
           
           //度数 * (π / 180） = 弧度 ，下面将度转为弧度
            GOAL2_YAW_RAD = GOAL2_YAW * (pai/180);
           //下面是将欧拉角转为四元数的代码，这里欧拉角的单位是弧度，如果是角度，先转为弧度再用下面代码转为四元数。单位为弧度的欧拉角，注意一个细节易错点，就是在欧拉角转四元数的计算的最开始是先除以一个2，这个容易漏掉。这里三个欧拉角(单位弧度),roll和pitch是0,yaw是yaw_set，转出的四元数是quaternion_x，quaternion_y，quaternion_z，quaternion_w。
            float p = 0;
            float y = GOAL2_YAW_RAD/2;
            float r = 0;

            float sinp = sin(p);
            float siny = sin(y);
            float sinr = sin(r);
            float cosp = cos(p);
            float cosy = cos(y);
            float cosr = cos(r);

            quaternion_x = sinr * cosp * cosy - cosr * sinp * siny;
            quaternion_y = cosr * sinp * cosy + sinr * cosp * siny;
            quaternion_z = cosr * cosp * siny - sinr * sinp * cosy;
            quaternion_w = cosr * cosp * cosy + sinr * sinp * siny;

           ego_goal_pos.pose.orientation.x = quaternion_x;
           ego_goal_pos.pose.orientation.y = quaternion_y;
           ego_goal_pos.pose.orientation.z = quaternion_z;
           ego_goal_pos.pose.orientation.w = quaternion_w;
           ego_goal_pub.publish(ego_goal_pos);
           state_flag = 2;
           }
        }
    }

    if(state_flag == 2)
    {
        ROS_INFO("state_flag is 2");
        pos_setpoint.type_mask =  0b100111111000; 
        pos_setpoint.coordinate_frame = 1;
        pos_setpoint.position.x = ego_poscmd[0];
        pos_setpoint.position.y = ego_poscmd[1];
        pos_setpoint.position.z = ego_poscmd[2];
        pos_setpoint.yaw = yaw_set;
        setpoint_raw_local_pub.publish(pos_setpoint);

        if((abs(cur_position[0] - goal2[0]) < 0.1)&&(abs(cur_position[1] - goal2[1]) < 0.1)&&((abs(cur_position[2] - goal2[2])) < 0.1))
        {
           count2 = count2 + 1;
           //大约悬停5秒钟，150/30
           if(count2 > 150)
           {
           geometry_msgs::PoseStamped ego_goal_pos;
           ego_goal_pos.pose.position.x = goal3[0];
           ego_goal_pos.pose.position.y = goal3[1];
           ego_goal_pos.pose.position.z = goal3[2];
           //下面四元数代表的是，偏航180度，俯仰，横滚都是0度，可去这个网站转换https://quaternions.online/
           //ego_goal_pos.pose.orientation.x = 1;
           //ego_goal_pos.pose.orientation.y = 0;
           //ego_goal_pos.pose.orientation.z = 0;
           //ego_goal_pos.pose.orientation.w = 0;
           
           //度数 * (π / 180） = 弧度 ，下面将度转为弧度
            GOAL3_YAW_RAD = GOAL3_YAW * (pai/180);
           //下面是将欧拉角转为四元数的代码，这里欧拉角的单位是弧度，如果是角度，先转为弧度再用下面代码转为四元数。单位为弧度的欧拉角，注意一个细节易错点，就是在欧拉角转四元数的计算的最开始是先除以一个2，这个容易漏掉。这里三个欧拉角(单位弧度),roll和pitch是0,yaw是yaw_set，转出的四元数是quaternion_x，quaternion_y，quaternion_z，quaternion_w。
            float p = 0;
            float y = GOAL3_YAW_RAD/2;
            float r = 0;

            float sinp = sin(p);
            float siny = sin(y);
            float sinr = sin(r);
            float cosp = cos(p);
            float cosy = cos(y);
            float cosr = cos(r);

            quaternion_x = sinr * cosp * cosy - cosr * sinp * siny;
            quaternion_y = cosr * sinp * cosy + sinr * cosp * siny;
            quaternion_z = cosr * cosp * siny - sinr * sinp * cosy;
            quaternion_w = cosr * cosp * cosy + sinr * sinp * siny;

           ego_goal_pos.pose.orientation.x = quaternion_x;
           ego_goal_pos.pose.orientation.y = quaternion_y;
           ego_goal_pos.pose.orientation.z = quaternion_z;
           ego_goal_pos.pose.orientation.w = quaternion_w;
           ego_goal_pub.publish(ego_goal_pos);
           state_flag = 3;
           }
        }
    }

    if(state_flag == 3)
    {
        ROS_INFO("state_flag is 3");
        pos_setpoint.type_mask =  0b100111111000; 
        pos_setpoint.coordinate_frame = 1;
        pos_setpoint.position.x = ego_poscmd[0];
        pos_setpoint.position.y = ego_poscmd[1];
        pos_setpoint.position.z = ego_poscmd[2];
        pos_setpoint.yaw = yaw_set;
        setpoint_raw_local_pub.publish(pos_setpoint);

        if((abs(cur_position[0] - goal3[0]) < 0.1)&&(abs(cur_position[1] - goal3[1]) < 0.1)&&((abs(cur_position[2] - goal3[2])) < 0.1))
        {
           count3 = count3 + 1;
           //大约悬停5秒钟，150/30
           if(count3 > 150)
           {
           geometry_msgs::PoseStamped ego_goal_pos;
           ego_goal_pos.pose.position.x = goal4[0];
           ego_goal_pos.pose.position.y = goal4[1];
           ego_goal_pos.pose.position.z = goal4[2];
           //下面四元数代表的是，偏航270度，俯仰，横滚都是0度，可去这个网站转换https://quaternions.online/
           //ego_goal_pos.pose.orientation.x = 0.707;
           //ego_goal_pos.pose.orientation.y = 0;
           //ego_goal_pos.pose.orientation.z = 0;
           //ego_goal_pos.pose.orientation.w = -0.707;
           
           //度数 * (π / 180） = 弧度 ，下面将度转为弧度
            GOAL4_YAW_RAD = GOAL4_YAW * (pai/180);
           //下面是将欧拉角转为四元数的代码，这里欧拉角的单位是弧度，如果是角度，先转为弧度再用下面代码转为四元数。单位为弧度的欧拉角，注意一个细节易错点，就是在欧拉角转四元数的计算的最开始是先除以一个2，这个容易漏掉。这里三个欧拉角(单位弧度),roll和pitch是0,yaw是yaw_set，转出的四元数是quaternion_x，quaternion_y，quaternion_z，quaternion_w。
            float p = 0;
            float y = GOAL4_YAW_RAD/2;
            float r = 0;

            float sinp = sin(p);
            float siny = sin(y);
            float sinr = sin(r);
            float cosp = cos(p);
            float cosy = cos(y);
            float cosr = cos(r);

            quaternion_x = sinr * cosp * cosy - cosr * sinp * siny;
            quaternion_y = cosr * sinp * cosy + sinr * cosp * siny;
            quaternion_z = cosr * cosp * siny - sinr * sinp * cosy;
            quaternion_w = cosr * cosp * cosy + sinr * sinp * siny;

           ego_goal_pos.pose.orientation.x = quaternion_x;
           ego_goal_pos.pose.orientation.y = quaternion_y;
           ego_goal_pos.pose.orientation.z = quaternion_z;
           ego_goal_pos.pose.orientation.w = quaternion_w;
           ego_goal_pub.publish(ego_goal_pos);
           state_flag = 4;
           }
        }
    }

    if(state_flag == 4)
    {
        ROS_INFO("state_flag is 4");
        pos_setpoint.type_mask =  0b100111111000; 
        pos_setpoint.coordinate_frame = 1;
        pos_setpoint.position.x = ego_poscmd[0];
        pos_setpoint.position.y = ego_poscmd[1];
        pos_setpoint.position.z = ego_poscmd[2];
        pos_setpoint.yaw = yaw_set;
        setpoint_raw_local_pub.publish(pos_setpoint);

        if((abs(cur_position[0] - goal4[0]) < 0.1)&&(abs(cur_position[1] - goal4[1]) < 0.1)&&((abs(cur_position[2] - goal4[2])) < 0.1))
        {
          count4 = count4 + 1;
          //大约悬停5秒钟，150/30
          if(count4 > 150)
          {
            state_flag = 5;
          }
        }
    }

    //切land降落
    if(state_flag == 5)
    {
         set_mode_client.call(land_set_mode);
         if(land_set_mode.response.mode_sent)
           {
             ROS_INFO("land enabled");   
           }  
    }


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


