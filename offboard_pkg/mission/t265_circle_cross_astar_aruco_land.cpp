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
#include <mavros_msgs/AttitudeTarget.h>
#include <offboard_pkg/DetectionInfo.h>

#include "math_utils.h"

#include "offboard_pkg/ControlCommand.h"
#include <offboard_pkg/ArucoInfo.h>

mavros_msgs::State current_state;

double cur_position[3]={0,0,0};
double circle_position_det[3]={0,0,0};
double circle_center_pos_enu[3]={0,0,0};
double circle_center_pos_enu_store[3]={0,0,0};
double circle_center_pos_enu_store_avr[3]={0,0,0};
bool detected_1;
bool IF_SIMULATION;

int circle_center_pos_enu_count = 0;

float current_roll;
float current_pitch;
float current_yaw;
float THRUST;

float yawratefromsim;

int state_flag = 0;

float astar_path_pos_set[3]={0,0,0};

int astar_pos_cmd_count = 0;

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

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
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

void ellipse_det_cb(const offboard_pkg::DetectionInfo::ConstPtr &msg)
        {   
            detected_1=msg->detected;
            circle_position_det[0] = msg->position[0];
            circle_position_det[1] = msg->position[1];
            circle_position_det[2] = msg->position[2];
        }

void drone_control_command_cb(const offboard_pkg::ControlCommand::ConstPtr& msg)
{
    astar_path_pos_set[0] = msg->Reference_State.position_ref[0];
    astar_path_pos_set[1] = msg->Reference_State.position_ref[1];
    astar_path_pos_set[2] = msg->Reference_State.position_ref[2];

    astar_pos_cmd_count = astar_pos_cmd_count + 1;
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
    ros::init(argc, argv, "t265_circle_cross_astar_aruco_land");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    //float yaw_rate_P;
    //float roll_rate_P;
    //float pitch_rate_P;
    

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //ros::Publisher yawrate_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    //需要订阅椭圆检测结果话题 /drone/object_detection/ellipse_det
    ros::Subscriber ellipse_det_sub = nh.subscribe<offboard_pkg::DetectionInfo>("/drone/object_detection/ellipse_det", 10, ellipse_det_cb);

    //ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    ros::Publisher astar_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/astar/goal", 10);

    ros::Subscriber drone_control_command_sub = nh.subscribe<offboard_pkg::ControlCommand>("/drone/control_command", 100, drone_control_command_cb);

    ros::Subscriber aruco_det_sub = nh.subscribe<offboard_pkg::ArucoInfo>("/drone/object_detection/aruco_det", 10, aruco_det_cb);



    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //nh1.param<float>("yaw_rate_P", yaw_rate_P, 0.05);
    //nh1.param<float>("roll_rate_P", roll_rate_P, 1);
    //nh1.param<float>("pitch_rate_P", pitch_rate_P, 1);
    //nh1.param<float>("thrust", THRUST, 0.5);
    //nh1.param<bool>("if_simulation", IF_SIMULATION, false);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    //while(ros::ok() && !current_state.connected){
    //    ros::spinOnce();
    //    rate.sleep();
    //}


//真机上应该不用再先发点了，因为手动切offboard嘛，等程序先运行起来已经发点了，再手动切offboard就够了。
    //send a few setpoints before starting
  //  for(int i = 100; ros::ok() && i > 0; --i){
  //     local_pos_pub.publish(pose);
  //      ros::spinOnce();
  //       rate.sleep();
  //  }

    //mavros_msgs::SetMode offb_set_mode;
    //offb_set_mode.request.custom_mode = "OFFBOARD";

    //mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;

    mavros_msgs::SetMode manual_set_mode;
    manual_set_mode.request.custom_mode = "MANUAL";

    mavros_msgs::CommandBool disarm_cmd;
    //arm_cmd.request.value = true;
    disarm_cmd.request.value = false; //这样就是上锁

    ros::Time time_snap1 = ros::Time::now();

    ros::Time time_snap2;

    ros::Time time_snap3;

    ros::Time time_snap5;

    ros::Time time_snap8;

    ros::Time time_snap9;

    ros::Time aruco_preland_enupos_record_time;

    while(ros::ok()){

    //参考自：Prometheus/master/Modules/control/include/command_to_mavros.h

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

    //circle_center_pos_enu[0] = cur_position[0] + circle_position_det[2] - 2; 
    //circle_center_pos_enu[1] = cur_position[1] - circle_position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - circle_position_det[1]; 
    printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0]);
    printf("circle_center_pos_enu[1] is %f\n", circle_center_pos_enu[1]);
    printf("circle_center_pos_enu[2] is %f\n", circle_center_pos_enu[2]);

    pos_setpoint.position.x = 0;
    pos_setpoint.position.y = 0;
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = 1.0;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //定点悬停多少秒之后再进入下一个状态进行圆跟踪  10秒
    if((abs(cur_position[0]) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-1.0)) < 0.1)&&(ros::Time::now() - time_snap1 > ros::Duration(7.0)))
      {
       state_flag = 1;
      }
    }


    if(state_flag == 1)
    {
    ROS_INFO("state_flag is 1");
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    //pos_setpoint.type_mask = 0b010111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + circle_position_det[2] - 2.2; 
    circle_center_pos_enu[0] = cur_position[0] + circle_position_det[2];
    circle_center_pos_enu[1] = cur_position[1] - circle_position_det[0]; 
    circle_center_pos_enu[2] = cur_position[2] - circle_position_det[1]; 
    //printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0] + 2.2);
    printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0]);
    printf("circle_center_pos_enu[1] is %f\n", circle_center_pos_enu[1]);
    printf("circle_center_pos_enu[2] is %f\n", circle_center_pos_enu[2]);

    pos_setpoint.position.x = circle_center_pos_enu[0] - 0.8;
    pos_setpoint.position.y = circle_center_pos_enu[1];
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = circle_center_pos_enu[2];
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //能否弄成十次检测数据的平均值
    if((abs(circle_position_det[0])<0.05)&&(abs(circle_position_det[1])<0.05)&&((abs(circle_position_det[2] - 0.8))<0.2))
      {  
         circle_center_pos_enu_count = circle_center_pos_enu_count + 1;
         circle_center_pos_enu_store[0] = circle_center_pos_enu_store[0] + circle_center_pos_enu[0];
         circle_center_pos_enu_store[1] = circle_center_pos_enu_store[1] + circle_center_pos_enu[1];
         circle_center_pos_enu_store[2] = circle_center_pos_enu_store[2] + circle_center_pos_enu[2];
         if(circle_center_pos_enu_count == 10)
           {
             circle_center_pos_enu_store_avr[0] = circle_center_pos_enu_store[0]/10;
             circle_center_pos_enu_store_avr[1] = circle_center_pos_enu_store[1]/10;
             circle_center_pos_enu_store_avr[2] = circle_center_pos_enu_store[2]/10;
             state_flag = 2;   //如果圆框跟踪就把这句注释掉
             time_snap2 = ros::Time::now();
           }
      }
    }





    if(state_flag == 2)
    {
    ROS_INFO("state_flag is 2");
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + circle_position_det[2] - 2.2; 
    //circle_center_pos_enu[1] = cur_position[1] - circle_position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - circle_position_det[1]; 
    printf("circle_center_pos_enu_store_avr[0] is %f\n", circle_center_pos_enu_store_avr[0]);
    printf("circle_center_pos_enu_store_avr[1] is %f\n", circle_center_pos_enu_store_avr[1]);
    printf("circle_center_pos_enu_store_avr[2] is %f\n", circle_center_pos_enu_store_avr[2]);

    //pos_setpoint.position.x = circle_center_pos_enu_store_avr[0] + 1 + 2.2;
    pos_setpoint.position.x = circle_center_pos_enu_store_avr[0] + 1;  //飞到圆心后一米的位置
    pos_setpoint.position.y = circle_center_pos_enu_store_avr[1];
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = circle_center_pos_enu_store_avr[2];
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);


    //定点悬停多少秒之后再进入下一个状态  10秒
    if((abs(cur_position[0]-(circle_center_pos_enu_store_avr[0] + 1)) < 0.1)&&(abs(cur_position[1]-circle_center_pos_enu_store_avr[1]) < 0.1)&&((abs(cur_position[2]-circle_center_pos_enu_store_avr[2])) < 0.1)&&(ros::Time::now() - time_snap2 > ros::Duration(6.0)))
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
    pos_setpoint.position.z = 1;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //定点悬停多少秒之后再进入下一个状态  10秒
    if((abs(cur_position[0] - 3) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-1)) < 0.1)&&(ros::Time::now() - time_snap3 > ros::Duration(4.0)))
      {      
           state_flag = 4;
      }

    }


    if(state_flag == 4)
    {
    ROS_INFO("state_flag is 4");
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

    if((abs(cur_position[2]-0.5)) < 0.1)
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
    pos_setpoint.position.z = 0.5;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //定点悬停多少秒之后再进入下一个状态  10秒
    if((abs(cur_position[0] - 3) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-0.5)) < 0.1)&&(ros::Time::now() - time_snap5 > ros::Duration(7.0)))
      {      
        //发布astar目标点话题。
        geometry_msgs::PoseStamped astar_goal_pos;
        astar_goal_pos.pose.position.x = 0;
        astar_goal_pos.pose.position.y = -4;
        astar_goal_pos.pose.position.z = 0.5;
        astar_goal_pos.pose.orientation.x = 0;
        astar_goal_pos.pose.orientation.y = 0;
        astar_goal_pos.pose.orientation.z = 0;
        astar_goal_pos.pose.orientation.w = 1;

        astar_goal_pub.publish(astar_goal_pos);
        if(astar_pos_cmd_count > 1)
         {
           state_flag = 6;
         }
      }

    }

 
    //走astar轨迹
    if(state_flag == 6)
    {
     ROS_INFO("state_flag is 6");
    /******************************************
    geometry_msgs::PoseStamped astar_goal_pos;
    astar_goal_pos.pose.position.x = 0;
    astar_goal_pos.pose.position.y = -4;
    astar_goal_pos.pose.position.z = 0.3;
    astar_goal_pos.pose.orientation.x = 0;
    astar_goal_pos.pose.orientation.y = 0;
    astar_goal_pos.pose.orientation.z = 0;
    astar_goal_pos.pose.orientation.w = 1;

    astar_goal_pub.publish(astar_goal_pos);
    **********************************************/
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;


    pos_setpoint.position.x = astar_path_pos_set[0];  
    pos_setpoint.position.y = astar_path_pos_set[1];
    pos_setpoint.position.z = astar_path_pos_set[2];
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);
 
    if((abs(cur_position[0]) < 0.1)&&(abs(cur_position[1] - (-4)) < 0.1)&&((abs(cur_position[2]-0.5)) < 0.1))
     {
        state_flag = 7;
     }

    }


    if(state_flag == 7)
    {
     ROS_INFO("state_flag is 7");
    //发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
    /****************************************
    pos_setpoint.type_mask = 0b100111000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = 0;
    pos_setpoint.velocity.y = 0;
    pos_setpoint.velocity.z = 0.1;

    pos_setpoint.yaw = 0;
    *****************************************/
    //发xyz速度+xy位置
    pos_setpoint.type_mask = 0b100111000100;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = 0;
    pos_setpoint.velocity.y = 0;
    pos_setpoint.velocity.z = 0.1;
    pos_setpoint.position.x = 0;
    pos_setpoint.position.y = -4;

    pos_setpoint.yaw = 0;



    setpoint_raw_local_pub.publish(pos_setpoint);

    if((abs(cur_position[2]-1)) < 0.1)
     {
         state_flag = 8;
         time_snap8 = ros::Time::now();
     }

    }


    //悬停在 0 -4  1
    if(state_flag == 8)
    {
     ROS_INFO("state_flag is 8");
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;


    pos_setpoint.position.x = 0;  
    pos_setpoint.position.y = -4;
    pos_setpoint.position.z = 1;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //定点悬停多少秒之后再进入下一个状态  
    if((abs(cur_position[0] ) < 0.1)&&(abs(cur_position[1] - (-4)) < 0.1)&&((abs(cur_position[2] - 1)) < 0.1)&&(ros::Time::now() - time_snap8 > ros::Duration(5.0)))
      {      
           state_flag = 9;
           time_snap9 = ros::Time::now();
      }

    }


    if(state_flag == 9)
    {
     ROS_INFO("state_flag is 9");
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
    aruco_pos_enu[0] = cur_position[0] - aruco_position_det[1];
    aruco_pos_enu[1] = cur_position[1] - aruco_position_det[0]; 
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
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    

    if((ros::Time::now() - aruco_redetect_time > ros::Duration(5.0))&&(ros::Time::now() - time_snap9 > ros::Duration(3.0))&&(abs(aruco_position_det[0]) < 0.05)&&(abs(aruco_position_det[1]) < 0.05))
     {
       //记录下当前的时间戳和二维码的ENU坐标
       aruco_preland_enupos_record_time = ros::Time::now();
       aruco_preland_pos_enu[0] = aruco_pos_enu[0];
       aruco_preland_pos_enu[1] = aruco_pos_enu[1];
       aruco_preland_pos_enu[2] = aruco_pos_enu[2];

       state_flag = 10;
     }
  

    }


    //提前飞到二维码上方
    if(state_flag == 10)
    {
     ROS_INFO("state_flag is 10");
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;


    pos_setpoint.position.x = aruco_preland_pos_enu[0];
    pos_setpoint.position.y = aruco_preland_pos_enu[1];
    pos_setpoint.position.z = aruco_preland_pos_enu[2] + 0.20;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);


    if(ros::Time::now() - aruco_preland_enupos_record_time > ros::Duration(20.9))
     {
       state_flag = 11;
     }

    }


    //降落 先切manual再disarm
    if(state_flag == 11)
    {
     ROS_INFO("state_flag is 11");
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
       state_flag = 9;
       state20_count = 0;
     }

    }


    ros::spinOnce();
    rate.sleep();
    }

    return 0;
}


