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
#include <offboard_pkg/ArucoInfo.h>


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

double setpoints0[16][3] = {
   {3,0,0.3},
   {3,-0.1,0.3},
   {3,-0.2,0.3},
   {3,-0.3,0.3},
   {3,-0.4,0.3},
   {3,-0.5,0.3},
   {3,-0.6,0.3},
   {3,-0.7,0.3},
   {3,-0.8,0.3},
   {3,-0.9,0.3},
   {3,-1.0,0.3},
   {3,-1.1,0.3},
   {3,-1.2,0.3},
   {3,-1.3,0.3},
   {3,-1.4,0.3},
   {3,-1.5,0.3}
   };

double setpoints1[13][3] = {
   {3,-1.5,0.3},
   {3,-1.6,0.3},
   {3,-1.7,0.3},
   {3,-1.8,0.3},
   {3,-1.9,0.3},
   {3,-2.0,0.3},
   {3,-2.1,0.3},
   {3,-2.2,0.3},
   {3,-2.3,0.3},
   {3,-2.4,0.3},
   {3,-2.5,0.3},
   {3,-2.6,0.3},
   {3,-2.65,0.3}
   };

double setpoints2[43][3] = {
   {3.0,-4.65,1.4},
   {3.0,-4.65,1.3},
   {3.0,-4.65,1.2},
   {3.0,-4.65,1.1},
   {3.0,-4.65,1},
   {2.9,-4.65,1},
   {2.8,-4.65,1},
   {2.7,-4.65,1},
   {2.6,-4.65,1},
   {2.5,-4.65,1},
   {2.4,-4.65,1},
   {2.3,-4.65,1},
   {2.2,-4.65,1},
   {2.1,-4.65,1},
   {2.0,-4.65,1},
   {1.9,-4.65,1},
   {1.8,-4.65,1},
   {1.7,-4.65,1},
   {1.6,-4.65,1},
   {1.5,-4.65,1},
   {1.4,-4.65,1},
   {1.3,-4.65,1},
   {1.2,-4.65,1},
   {1.1,-4.65,1},
   {1.0,-4.65,1},
   {1.0,-4.6,1},
   {1.0,-4.6,1},
   {1.0,-4.5,1},
   {1.0,-4.4,1},
   {1.0,-4.3,1},
   {1.0,-4.2,1},
   {1.0,-4.1,1},
   {1.0,-4.0,1},
   {0.9,-4.0,1},
   {0.8,-4.0,1},
   {0.7,-4.0,1},
   {0.6,-4.0,1},
   {0.5,-4.0,1},
   {0.4,-4.0,1},
   {0.3,-4.0,1},
   {0.2,-4.0,1},
   {0.1,-4.0,1},
   {0.0,-4.0,1}
   };

int setpoints0_i = 0;
int setpoints1_i = 0;
int setpoints2_i = 0;

bool aruco_detected_once;
bool aruco_detected_flag;

int aruco_det_num_regain = 0;
int aruco_det_num_lost = 0;
int VISION_THRES = 13;

ros::Time aruco_redetect_time;

double aruco_position_det[3]={0,0,0};
double aruco_pos_enu[3]={0,0,0};
double aruco_preland_pos_enu[3]={0,0,0};

double cur_position_tempstore[3]={0,0,0};

double current_yaw_tempstore;

int state20_count = 0;

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

void aruco_det_cb(const offboard_pkg::ArucoInfo::ConstPtr &msg)
        {   
            aruco_detected_once=msg->detected;
            aruco_position_det[0] = msg->position[0];
            aruco_position_det[1] = msg->position[1];
            aruco_position_det[2] = msg->position[2];

            if(aruco_detected_once)
            {
               aruco_det_num_regain++;
               aruco_det_num_lost = 0;
            }else
            {
               aruco_det_num_regain = 0;
               aruco_det_num_lost++;
            }

            // 当连续一段时间无法检测到目标时，认定目标丢失
            if(aruco_det_num_lost > VISION_THRES)
            {
               aruco_detected_flag = false;
               //存储无人机丢失二维码时的当前位置，后面作为悬停位置
               //之所以后来不放在aruco_det_cb回调函数里面是防止最开始的时候就没有检测到二维码的时候这个初始的位置记录值可能和无人机当前位置偏差较远
               if(aruco_det_num_lost == VISION_THRES+1)
               {
                 //cur_position_tempstore[0] = cur_position[0];
                 //cur_position_tempstore[1] = cur_position[1];
                 //cur_position_tempstore[2] = cur_position[2];
               }
            }

            // 当连续一段时间检测到目标时，认定目标得到
            if(aruco_det_num_regain > VISION_THRES)
            {
               aruco_detected_flag = true;
               if(aruco_det_num_regain == VISION_THRES + 1)
               {
                 aruco_redetect_time = ros::Time::now(); //记录下重新检测到aruco二维码的最开始的时刻，好后面计算连续检测到了aruco二维码多少秒
               }
            }

          }





int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_circle_cross_ego_aruco_land");
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
    ros::Subscriber aruco_det_sub = nh.subscribe<offboard_pkg::ArucoInfo>("/drone/object_detection/aruco_det", 10, aruco_det_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

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

    mavros_msgs::SetMode manual_set_mode;
    manual_set_mode.request.custom_mode = "MANUAL";

    mavros_msgs::CommandBool disarm_cmd;
    //arm_cmd.request.value = true;
    disarm_cmd.request.value = false; //这样就是上锁



    ros::Time last_request = ros::Time::now();

    ros::Time time_snap0 = ros::Time::now();

    ros::Time time_snap1;

    ros::Time time_snap3;

    ros::Time time_snap5;

    ros::Time time_snap6;

    ros::Time time_snap7;

    ros::Time time_snap8;

    ros::Time time_snap9;

    ros::Time time_snap10;

    ros::Time aruco_preland_enupos_record_time;


    while(ros::ok()){

    mavros_msgs::PositionTarget pos_setpoint;

    if(cur_position[2] < 0.7)
    {
       time_snap0 = ros::Time::now();
    }

    printf("current_yaw is %f\n", current_yaw);

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
    if((abs(cur_position[0]) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-1.0)) < 0.1)&&(ros::Time::now() - time_snap0 > ros::Duration(7.0)))
      {
           geometry_msgs::PoseStamped ego_goal_pos;
           ego_goal_pos.pose.position.x = 3;
           ego_goal_pos.pose.position.y = 0;
           ego_goal_pos.pose.position.z = 1;
           ego_goal_pos.pose.orientation.x = 0;
           ego_goal_pos.pose.orientation.y = 0;
           ego_goal_pos.pose.orientation.z = 0;
           ego_goal_pos.pose.orientation.w = 1;
           ego_goal_pub.publish(ego_goal_pos);
           if(ego_poscmd_count > 1)
            {
              state_flag = 1;
              time_snap1 = ros::Time::now();
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
        //pos_setpoint.yaw = yaw_set;
        pos_setpoint.yaw = 0;
        setpoint_raw_local_pub.publish(pos_setpoint);


    if((abs(cur_position[0] - 3) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-1.0)) < 0.1)&&(ros::Time::now() - time_snap1 > ros::Duration(15.0)))
      {
        state_flag = 2;
      }
    }


    if(state_flag == 2)
    {
     ROS_INFO("state_flag is 2");
    //发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
    /*****************************************
    pos_setpoint.type_mask = 0b100111000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = 0;
    pos_setpoint.velocity.y = 0;
    pos_setpoint.velocity.z = -0.1;

    pos_setpoint.yaw = 0;
    ******************************************/
    //发xyz速度+xy位置
    pos_setpoint.type_mask = 0b100111000100;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = 0;
    pos_setpoint.velocity.y = 0;
    pos_setpoint.velocity.z = -0.1;
    pos_setpoint.position.x = 3;
    pos_setpoint.position.y = 0;

    pos_setpoint.yaw = 0;

    setpoint_raw_local_pub.publish(pos_setpoint);

    if((abs(cur_position[2]-0.3)) < 0.1)
     {
         state_flag = 3;
         time_snap3 = ros::Time::now();
     }
    }


    if(state_flag == 3)
    {
     ROS_INFO("state_flag is 3");
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = 3;
    pos_setpoint.position.y = 0;
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = 0.3;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

   if((abs(cur_position[0] - 3) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-0.3)) < 0.1)&&(ros::Time::now() - time_snap3 > ros::Duration(4.0)))
      {
        state_flag = 4;
      }
    }



    if(state_flag == 4)
    {
     ROS_INFO("state_flag is 4");
     //发位置和期望角速度
    pos_setpoint.type_mask = 0b010111111000;  // 010 111 111 000  xyz + yawrate
    //pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = 3;
    pos_setpoint.position.y = 0;
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = 0.3;

    pos_setpoint.yaw_rate = -0.1;  //0.1弧度大概对应5.73度

    setpoint_raw_local_pub.publish(pos_setpoint);

    printf("current_yaw is %f\n", current_yaw);

    if(abs(current_yaw - (-1.57)) < 0.087)  //0.087弧度大概对应角度5度
     {
       state_flag = 5;
       time_snap5 = ros::Time::now();
     }
    }


    if(state_flag == 5)
    {
     ROS_INFO("state_flag is 5");
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = 3;
    pos_setpoint.position.y = 0;
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = 0.3;
    pos_setpoint.yaw = -1.570796;   //90度对应1.5707963267948966弧度
    setpoint_raw_local_pub.publish(pos_setpoint);

    printf("current_yaw is %f\n", current_yaw);

    if(ros::Time::now() - time_snap5 > ros::Duration(5.0))
     {
        state_flag = 6;  //调试暂时注释掉
       time_snap6 = ros::Time::now();
     }
    }

    if(state_flag == 6)
    {
     ROS_INFO("state_flag is 6");
    //这步感觉也可以通过ego走过去，避免撞到障碍物。
       if((ros::Time::now() - time_snap6 > ros::Duration(1.0))&&(setpoints0_i < 15))
       {
          setpoints0_i = setpoints0_i + 1;
          time_snap6 = ros::Time::now();
       }

    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = setpoints0[setpoints0_i][0];
    pos_setpoint.position.y = setpoints0[setpoints0_i][1];
    pos_setpoint.position.z = 0.3;
    pos_setpoint.yaw = -1.570796;   //90度对应1.5707963267948966弧度
    setpoint_raw_local_pub.publish(pos_setpoint);

    if((abs(cur_position[0] - 3) < 0.1)&&(abs(cur_position[1] - (-1.5)) < 0.1)&&((abs(cur_position[2]-0.3)) < 0.1)&&(ros::Time::now() - time_snap6 > ros::Duration(5.0)))
      {
        state_flag = 7;
       time_snap7 = ros::Time::now();
      }

    }

    if(state_flag == 7)
    {
     ROS_INFO("state_flag is 7");
    //这步感觉也可以通过ego走过去，避免撞到障碍物。
       if((ros::Time::now() - time_snap7 > ros::Duration(1.0))&&(setpoints1_i < 12))
       {
          setpoints1_i = setpoints1_i + 1;
          time_snap7 = ros::Time::now();
       }

    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = setpoints1[setpoints1_i][0];
    pos_setpoint.position.y = setpoints1[setpoints1_i][1];  //还可以试试-2.70
    pos_setpoint.position.z = 0.3;
    pos_setpoint.yaw = -1.570796;   //90度对应1.5707963267948966弧度
    setpoint_raw_local_pub.publish(pos_setpoint);

    if((abs(cur_position[0] - 3) < 0.1)&&(abs(cur_position[1] - (-2.65)) < 0.1)&&((abs(cur_position[2]-0.3)) < 0.1)&&(ros::Time::now() - time_snap7 > ros::Duration(5.0)))
      {
           //目标点的偏航角度需要注意
           geometry_msgs::PoseStamped ego_goal_pos;
           ego_goal_pos.pose.position.x = 3;
           //ego_goal_pos.pose.position.y = -4.8;
           ego_goal_pos.pose.position.y = -4.65;
           ego_goal_pos.pose.position.z = 1.4;
           //下面这组四元数对应的三个欧拉角是  yaw:-90度 roll:0度  pitch:0度 ，四元数和欧拉角的转换可以去这个网站：https://quaternions.online/ 我也是在这个网站把这三个欧拉角转为下面四个四元数的，实飞也验证了是OK的，可以使得无人机的期望偏航相对于初始偏航是在东北天坐标系下逆时针转90度的朝向。
           ego_goal_pos.pose.orientation.x = -0.707;
           ego_goal_pos.pose.orientation.y = 0;
           ego_goal_pos.pose.orientation.z = 0;
           ego_goal_pos.pose.orientation.w = 0.707;
           ego_goal_pub.publish(ego_goal_pos);
           if((abs(ego_poscmd[0] - 3) < 0.5)&&(abs(ego_poscmd[1] - (-2.65)) < 0.5))
            {
              state_flag = 8;   //调试暂时注释掉
              time_snap8 = ros::Time::now();
            }
      }

    }



    if(state_flag == 8)
    {
       ROS_INFO("state_flag is 8");
        pos_setpoint.type_mask =  0b100111111000; 
        pos_setpoint.coordinate_frame = 1;
        pos_setpoint.position.x = ego_poscmd[0];
        pos_setpoint.position.y = ego_poscmd[1];
        pos_setpoint.position.z = ego_poscmd[2];
        //pos_setpoint.yaw = yaw_set;
        pos_setpoint.yaw = -1.570796;
        setpoint_raw_local_pub.publish(pos_setpoint);

    if((abs(cur_position[0] - 3) < 0.1)&&(abs(cur_position[1] - (-4.65)) < 0.1)&&((abs(cur_position[2] - 1.4)) < 0.1)&&(ros::Time::now() - time_snap8 > ros::Duration(15.0)))
      {
        state_flag = 9;
        time_snap9 = ros::Time::now();
      }
    }


    if(state_flag == 9)
    {
       ROS_INFO("state_flag is 9");
       if((ros::Time::now() - time_snap9 > ros::Duration(0.8))&&(setpoints2_i < 42))
       {
          setpoints2_i = setpoints2_i + 1;
          time_snap9 = ros::Time::now();
       }
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = setpoints2[setpoints2_i][0];
    pos_setpoint.position.y = setpoints2[setpoints2_i][1];
    pos_setpoint.position.z = setpoints2[setpoints2_i][2];
    pos_setpoint.yaw = -1.570796;   //90度对应1.5707963267948966弧度
    setpoint_raw_local_pub.publish(pos_setpoint);

    if((setpoints2_i == 42)&&(abs(cur_position[0]) < 0.1)&&(abs(cur_position[1] - (-4.0)) < 0.1)&&((abs(cur_position[2]-1.0)) < 0.1)&&(ros::Time::now() - time_snap9 > ros::Duration(5.0)))
      {
        state_flag = 10;
        time_snap10 = ros::Time::now();
      }    

    }

    if(state_flag == 10)
    {
       ROS_INFO("state_flag is 10");
       //如果丢失二维码，则进入悬停状态
       if(aruco_detected_flag == false)
        {
           state_flag = 20;
        }
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    //pos_setpoint.type_mask = 0b010111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;


    //坐标系确认好
    //aruco_pos_enu[0] = cur_position[0] - aruco_position_det[1];
    //aruco_pos_enu[1] = cur_position[1] - aruco_position_det[0]; 
    aruco_pos_enu[0] = cur_position[0] - aruco_position_det[0]; //因为现在无人机转了90度，偏航是-90度，这个时候摄像头的坐标系和无人机的原始坐标系的关系有变化，所以需要改动。
    aruco_pos_enu[1] = cur_position[1] + aruco_position_det[1]; //因为现在无人机转了90度，偏航是-90度，这个时候摄像头的坐标系和无人机的原始坐标系的关系有变化，所以需要改动。
    aruco_pos_enu[2] = cur_position[2] - aruco_position_det[2]; 
    //printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0] + 2.2);
    printf("ros::Time::now() - aruco_redetect_time > ros::Duration(5.0) is %d\n",ros::Time::now() - aruco_redetect_time > ros::Duration(5.0));
    printf("aruco_position_det[0] is %f\n", aruco_position_det[0]);
    printf("aruco_position_det[1] is %f\n", aruco_position_det[1]);
    printf("aruco_position_det[2] is %f\n", aruco_position_det[2]);

    printf("aruco_pos_enu[0] is %f\n", aruco_pos_enu[0]);
    printf("aruco_pos_enu[1] is %f\n", aruco_pos_enu[1]);
    printf("aruco_pos_enu[2] is %f\n", aruco_pos_enu[2]);

    pos_setpoint.position.x = aruco_pos_enu[0];
    pos_setpoint.position.y = aruco_pos_enu[1];
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = aruco_pos_enu[2] + 0.6;
    pos_setpoint.yaw = -1.570796;
    setpoint_raw_local_pub.publish(pos_setpoint);

    

    if((ros::Time::now() - aruco_redetect_time > ros::Duration(5.0))&&(ros::Time::now() - time_snap10 > ros::Duration(3.0))&&(abs(aruco_position_det[0]) < 0.05)&&(abs(aruco_position_det[1]) < 0.05))
     {
       //记录下当前的时间戳和二维码的ENU坐标
       aruco_preland_enupos_record_time = ros::Time::now();
       aruco_preland_pos_enu[0] = aruco_pos_enu[0];
       aruco_preland_pos_enu[1] = aruco_pos_enu[1];
       aruco_preland_pos_enu[2] = aruco_pos_enu[2];

       state_flag = 11;
     }

    }


    //提前飞到二维码上方
    if(state_flag == 11)
    {
     ROS_INFO("state_flag is 11");
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;


    pos_setpoint.position.x = aruco_preland_pos_enu[0];
    pos_setpoint.position.y = aruco_preland_pos_enu[1];
    pos_setpoint.position.z = aruco_preland_pos_enu[2] + 0.20;
    pos_setpoint.yaw = -1.570796;
    setpoint_raw_local_pub.publish(pos_setpoint);


    if(ros::Time::now() - aruco_preland_enupos_record_time > ros::Duration(20.9))
     {
       state_flag = 12;
     }

    }


    //降落 先切manual再disarm
    if(state_flag == 12)
    {
     ROS_INFO("state_flag is 12");
      //此处切换会manual模式是因为:PX4默认在offboard模式且有控制的情况下没法上锁,所以先切为manual再上锁
       //前面加上判断可以避免反复一直切模式
         if(current_state.mode == "OFFBOARD")
         {
         set_mode_client.call(manual_set_mode);
         if(manual_set_mode.response.mode_sent)
           {
             ROS_INFO("manual enabled");          
           }  
         }

            //前面加上判断可以避免反复一直切上锁
            //if(current_state.armed){
            if((current_state.armed)&&(current_state.mode == "MANUAL")){ 
                if( arming_client.call(disarm_cmd) &&
                    disarm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");
                }
             }
    }





    if(state_flag == 20)
    {
     ROS_INFO("state_flag is 20");
    state20_count = state20_count + 1;
    if(state20_count == 1)
    {
       //记录刚丢失二维码时的无人机位置的值，作为悬停的位置点。
       //之所以不放在aruco_det_cb回调函数里面是防止最开始的时候就没有检测到二维码的时候这个初始的位置记录值可能和无人机当前位置偏差较远
       cur_position_tempstore[0] = cur_position[0];
       cur_position_tempstore[1] = cur_position[1];
       cur_position_tempstore[2] = cur_position[2];
       current_yaw_tempstore = current_yaw;
    }
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;


    //这个悬停比较严谨
    pos_setpoint.position.x = cur_position_tempstore[0];
    pos_setpoint.position.y = cur_position_tempstore[1];
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = cur_position_tempstore[2];
    pos_setpoint.yaw = current_yaw_tempstore;
    setpoint_raw_local_pub.publish(pos_setpoint);


    if(aruco_detected_flag == true)
     {
       state_flag = 10;
       state20_count = 0;
     }

    }




    
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


