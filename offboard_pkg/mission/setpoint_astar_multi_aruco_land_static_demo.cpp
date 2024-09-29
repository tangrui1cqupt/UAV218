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

//int aruco_det_num_regain = 0;
//int aruco_det_num_lost = 0;
int VISION_THRES = 13;

ros::Time aruco_redetect_time;

double aruco_position_det[3]={0,0,0};
double aruco_pos_enu[3]={0,0,0};
double aruco_preland_pos_enu[3]={0,0,0};

double cur_position_tempstore[3]={0,0,0};

double current_yaw_tempstore;

int state20_count = 0;

double position_det[3]={0,0,0};
int num_regain = 0;
int num_lost = 0;
bool is_detected;

int square_flag = 0;


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



void drone_control_command_cb(const offboard_pkg::ControlCommand::ConstPtr& msg)
{
    astar_path_pos_set[0] = msg->Reference_State.position_ref[0];
    astar_path_pos_set[1] = msg->Reference_State.position_ref[1];
    astar_path_pos_set[2] = msg->Reference_State.position_ref[2];

    astar_pos_cmd_count = astar_pos_cmd_count + 1;
}


void multi_aruco_det_cb(const offboard_pkg::DetectionInfo::ConstPtr &msg)
        {   
            detected_1=msg->detected;
            position_det[0] = msg->position[0];
            position_det[1] = msg->position[1];
            position_det[2] = msg->position[2];

            if(detected_1)
            {
               num_regain++;
               num_lost = 0;
            }else
            {
               num_regain = 0;
               num_lost++;
            }

            // 当连续一段时间无法检测到目标时，认定目标丢失
            if(num_lost > VISION_THRES)
            {
               is_detected = false;
               //存储无人机丢失二维码时的当前位置，后面作为悬停位置
               //这种逻辑不严谨，会造成丢失二维码之后的悬停位置有累计漂移
               //cur_position_tempstore[0] = cur_position[0];
               //cur_position_tempstore[1] = cur_position[1];
               //cur_position_tempstore[2] = cur_position[2];
            }

            // 当连续一段时间检测到目标时，认定目标得到
            if(num_regain > VISION_THRES)
            {
               is_detected = true;
            }


        }



int main(int argc, char **argv)
{
    ros::init(argc, argv, "t265_circle_cross_astar_aruco_land");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    ros::Publisher astar_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/astar/goal", 10);

    ros::Subscriber drone_control_command_sub = nh.subscribe<offboard_pkg::ControlCommand>("/drone/control_command", 100, drone_control_command_cb);

    ros::Subscriber multi_aruco_det_sub = nh.subscribe<offboard_pkg::DetectionInfo>("/drone/object_detection/multi_aruco_det", 10, multi_aruco_det_cb);



    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //nh1.param<bool>("if_simulation", IF_SIMULATION, false);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    //while(ros::ok() && !current_state.connected){
    //    ros::spinOnce();
    //    rate.sleep();
    //}


 //真机上应该不用再先发点了，因为手动切offboard，等程序先运行起来已经发点了，再手动切offboard就可以了。
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

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool disarm_cmd;
    //arm_cmd.request.value = true;
    disarm_cmd.request.value = false; //这样就是上锁

    ros::Time time_snap1 = ros::Time::now();
    ros::Time time_snap2;
    ros::Time time_snap3;
    ros::Time time_snap5;
    ros::Time time_snap8;
    ros::Time time_snap9;
    //ros::Time aruco_preland_enupos_record_time;

    while(ros::ok()){

    mavros_msgs::PositionTarget pos_setpoint;

    if(cur_position[2] < 0.3)
    {
       time_snap1 = ros::Time::now();
    }

    //起飞
    if(state_flag == 0)
    {
    ROS_INFO("state_flag is 0");
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0]);
    printf("circle_center_pos_enu[1] is %f\n", circle_center_pos_enu[1]);
    printf("circle_center_pos_enu[2] is %f\n", circle_center_pos_enu[2]);

    pos_setpoint.position.x = 0;
    pos_setpoint.position.y = 0;
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = 0.5;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //定点悬停多少秒之后再进入下一个状态 
    if((abs(cur_position[0]) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-0.5)) < 0.1)&&(ros::Time::now() - time_snap1 > ros::Duration(7.0)))
      {
       //state_flag = 1;
       //state_flag = 5;
        //发布astar目标点话题。
        geometry_msgs::PoseStamped astar_goal_pos;
        astar_goal_pos.pose.position.x = 5;
        astar_goal_pos.pose.position.y = 0;
        astar_goal_pos.pose.position.z = 0.5;
        astar_goal_pos.pose.orientation.x = 0;
        astar_goal_pos.pose.orientation.y = 0;
        astar_goal_pos.pose.orientation.z = 0;
        astar_goal_pos.pose.orientation.w = 1;

        astar_goal_pub.publish(astar_goal_pos);
        if(astar_pos_cmd_count > 1)
         {
           state_flag = 1;
         }
      }
    }


 
    //走astar轨迹
    if(state_flag == 1)
    {
     ROS_INFO("state_flag is 1");
     pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
     pos_setpoint.coordinate_frame = 1;


     pos_setpoint.position.x = astar_path_pos_set[0];  
     pos_setpoint.position.y = astar_path_pos_set[1];
     pos_setpoint.position.z = astar_path_pos_set[2];
     pos_setpoint.yaw = 0;
     setpoint_raw_local_pub.publish(pos_setpoint);
 
     if((abs(cur_position[0] - 5) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-0.5)) < 0.1))
     {
        state_flag = 2;
     }

    }


    if(state_flag == 2)
    {
     ROS_INFO("state_flag is 2");
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
    pos_setpoint.position.x = 5;
    pos_setpoint.position.y = 0;

    pos_setpoint.yaw = 0;



    setpoint_raw_local_pub.publish(pos_setpoint);

    if((abs(cur_position[2]-1)) < 0.1)
     {
         state_flag = 3;
         time_snap2 = ros::Time::now();
     }

    }


    //悬停在 4  0  1
    if(state_flag == 3)
    {
     ROS_INFO("state_flag is 3");
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;


    pos_setpoint.position.x = 5;  
    pos_setpoint.position.y = 0;
    pos_setpoint.position.z = 1;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //定点悬停多少秒之后再进入下一个状态  
    if((abs(cur_position[0] - 5 ) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2] - 1)) < 0.1)&&(ros::Time::now() - time_snap2 > ros::Duration(5.0))&&(is_detected == true))
      {      
           state_flag = 5;
           time_snap9 = ros::Time::now();
      }

    if((abs(cur_position[0] - 5 ) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2] - 1)) < 0.1)&&(ros::Time::now() - time_snap2 > ros::Duration(8.0))&&(is_detected == false))
      {      
           state_flag = 4;
           //time_snap9 = ros::Time::now();
      }

    }

   
    //走个边长为1米的正方形寻找二维码
    if(state_flag == 4)
    {
     double setpoints0[4][3] = {{5.5,0.5,1},{4.5,0.5,1},{4.5,-0.5,1},{5.5,-0.5,1}};

     if(is_detected == true)
     {
        state_flag = 5;
        continue;
     }

     pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
     pos_setpoint.coordinate_frame = 1;
 

     pos_setpoint.position.x = setpoints0[square_flag][0];  
     pos_setpoint.position.y = setpoints0[square_flag][1];
     pos_setpoint.position.z = setpoints0[square_flag][2];
     pos_setpoint.yaw = 0;
     setpoint_raw_local_pub.publish(pos_setpoint);

     if((abs(cur_position[0] - setpoints0[square_flag][0] ) < 0.1)&&(abs(cur_position[1] - setpoints0[square_flag][1]) < 0.1)&&((abs(cur_position[2] - setpoints0[square_flag][2])) < 0.1)&&(is_detected == false))
      {  
         if(square_flag != 3)   
          {
           square_flag = square_flag + 1;
          }
          if(square_flag == 3)
          {
           square_flag = 0;
          }
      }

    }


    if(state_flag == 5)
    {
     ROS_INFO("state_flag is 5");
       //如果丢失二维码，则进入悬停状态
       if(is_detected == false)
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
    aruco_pos_enu[0] = cur_position[0] - position_det[1];
    aruco_pos_enu[1] = cur_position[1] - position_det[0]; 
    aruco_pos_enu[2] = cur_position[2] - position_det[2]; 
    //printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0] + 2.2);
    //printf("ros::Time::now() - aruco_redetect_time > ros::Duration(5.0) is %d\n",ros::Time::now() - aruco_redetect_time > ros::Duration(5.0));
    printf("aruco_position_det[0] is %f\n", position_det[0]);
    printf("aruco_position_det[1] is %f\n", position_det[1]);
    printf("aruco_position_det[2] is %f\n", position_det[2]);

    printf("aruco_pos_enu[0] is %f\n", aruco_pos_enu[0]);
    printf("aruco_pos_enu[1] is %f\n", aruco_pos_enu[1]);
    printf("aruco_pos_enu[2] is %f\n", aruco_pos_enu[2]);

    pos_setpoint.position.x = aruco_pos_enu[0];
    pos_setpoint.position.y = aruco_pos_enu[1];
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = aruco_pos_enu[2] + 0.8;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    

    if((abs(position_det[0]) < 0.05)&&(abs(position_det[1]) < 0.05))
     {
       state_flag = 6;
     }
  

    }


   //通过发期望速度的方式降落
   if(state_flag == 6)
   {
     ROS_INFO("state_flag is 6");
    //发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
    //pos_setpoint.type_mask = 0b100111000111;
    //发xyz速度+xy位置
    pos_setpoint.type_mask = 0b100111000100;


    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = 0;
    pos_setpoint.velocity.y = 0;
    pos_setpoint.velocity.z = -0.1;
    pos_setpoint.position.x = aruco_pos_enu[0];
    pos_setpoint.position.y = aruco_pos_enu[1];


    pos_setpoint.yaw = 0;

    setpoint_raw_local_pub.publish(pos_setpoint);

    //if((abs(cur_position[2] - aruco_pos_enu[2]) < 0.10))
    //改为大小叠加的二维码之后，这个第二次水平对齐的高度可以更低，这样可以进一步提升降落精度
    //if(cur_position[2]  < 0.36)
    //if(cur_position[2]  < 0.16)
    if(cur_position[2]  < 0.31)
      {
        state_flag = 7;
      }
   }


    //第二次水平对齐
    if(state_flag == 7)
    {
     ROS_INFO("state_flag is 7");
       //如果丢失二维码，则进入悬停状态
      // if(is_detected == false)
      //  {
      //     state_flag = 6;
      //  }

    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    aruco_pos_enu[0] = cur_position[0] - position_det[1];
    aruco_pos_enu[1] = cur_position[1] - position_det[0]; 
    aruco_pos_enu[2] = cur_position[2] - position_det[2]; 

    printf("aruco_pos_enu[0] is %f\n", aruco_pos_enu[0]);
    printf("aruco_pos_enu[1] is %f\n", aruco_pos_enu[1]);
    printf("aruco_pos_enu[2] is %f\n", aruco_pos_enu[2]);

    pos_setpoint.position.x = aruco_pos_enu[0];
    pos_setpoint.position.y = aruco_pos_enu[1];
    //pos_setpoint.position.z = 0;
    //pos_setpoint.position.z = aruco_pos_enu[2] + 0.20;
    //pos_setpoint.position.z = 0.35;
    //pos_setpoint.position.z = 0.15;
    pos_setpoint.position.z = 0.30;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    if((abs(position_det[0]) < 0.05)&&(abs(position_det[1]) < 0.05))
      {
        //为了跑跟踪暂时注释掉
        state_flag = 8;
      }

    }


   //通过发期望速度的方式降落
   if(state_flag == 8)
   {
     ROS_INFO("state_flag is 8");
    //发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
    //pos_setpoint.type_mask = 0b100111000111;
    //发xyz速度+xy位置
    pos_setpoint.type_mask = 0b100111000100;


    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = 0;
    pos_setpoint.velocity.y = 0;
    pos_setpoint.velocity.z = -0.1;
    pos_setpoint.position.x = aruco_pos_enu[0];
    pos_setpoint.position.y = aruco_pos_enu[1];


    pos_setpoint.yaw = 0;

    setpoint_raw_local_pub.publish(pos_setpoint);

    //if((abs(cur_position[2] - aruco_pos_enu[2]) < 0.10))
    //if(cur_position[2]  < 0.06)
    if(cur_position[2]  < 0.16)
      {
      
        //state_flag = 6;
        //state_flag = 11;  //先切manual再上锁进行降落，如果为10，则是通过切land降落
        //state_flag = 17;
        //state_flag = 3;
        state_flag = 9;
      }
   }



    //第三次水平对齐
    if(state_flag == 9)
    {
     ROS_INFO("state_flag is 9");
       //如果丢失二维码，则进入悬停状态
      // if(is_detected == false)
      //  {
      //     state_flag = 6;
      //  }

    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    aruco_pos_enu[0] = cur_position[0] - position_det[1];
    aruco_pos_enu[1] = cur_position[1] - position_det[0]; 
    aruco_pos_enu[2] = cur_position[2] - position_det[2]; 

    printf("aruco_pos_enu[0] is %f\n", aruco_pos_enu[0]);
    printf("aruco_pos_enu[1] is %f\n", aruco_pos_enu[1]);
    printf("aruco_pos_enu[2] is %f\n", aruco_pos_enu[2]);

    pos_setpoint.position.x = aruco_pos_enu[0];
    pos_setpoint.position.y = aruco_pos_enu[1];
    //pos_setpoint.position.z = 0;
    //pos_setpoint.position.z = aruco_pos_enu[2] + 0.20;
    //pos_setpoint.position.z = 0.35;
    pos_setpoint.position.z = 0.15;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    if((abs(position_det[0]) < 0.05)&&(abs(position_det[1]) < 0.05))
      {
        //为了跑跟踪暂时注释掉
        state_flag = 10;
      }

    }


   //通过发期望速度的方式降落
   if(state_flag == 10)
   {
     ROS_INFO("state_flag is 10");
    //发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
    //pos_setpoint.type_mask = 0b100111000111;
    //发xyz速度+xy位置
    pos_setpoint.type_mask = 0b100111000100;


    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = 0;
    pos_setpoint.velocity.y = 0;
    pos_setpoint.velocity.z = -0.1;
    pos_setpoint.position.x = aruco_pos_enu[0];
    pos_setpoint.position.y = aruco_pos_enu[1];


    pos_setpoint.yaw = 0;

    setpoint_raw_local_pub.publish(pos_setpoint);

    //if((abs(cur_position[2] - aruco_pos_enu[2]) < 0.10))
    if(cur_position[2]  < 0.06)
      {
      
        //state_flag = 6;
        //state_flag = 11;  //先切manual再上锁进行降落，如果为10，则是通过切land降落
        state_flag = 17;
        //state_flag = 3;
      }
   }



    //通过service切land降落
    if(state_flag == 17)
    {

    printf("aruco_pos_enu[0] is %f\n", aruco_pos_enu[0]);
    printf("aruco_pos_enu[1] is %f\n", aruco_pos_enu[1]);
    printf("aruco_pos_enu[2] is %f\n", aruco_pos_enu[2]);

         set_mode_client.call(land_set_mode);
         if(land_set_mode.response.mode_sent)
           {
             ROS_INFO("land enabled");          
           }  
    }




    //降落 先切manual再disarm
    if(state_flag == 18)
    {
     ROS_INFO("state_flag is 18");
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


    if(is_detected == true)
     {
       state_flag = 5;
       state20_count = 0;
     }

    }


    ros::spinOnce();
    rate.sleep();
    }

    return 0;
}


