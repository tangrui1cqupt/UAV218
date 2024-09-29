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

mavros_msgs::State current_state;

double cur_position[3]={0,0,0};
double position_det[3]={0,0,0};
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
            position_det[0] = msg->position[0];
            position_det[1] = msg->position[1];
            position_det[2] = msg->position[2];
        }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "t265_circle_cross");
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

    ros::Time last_time = ros::Time::now();

    while(ros::ok()){

    //参考自：Prometheus/master/Modules/control/include/command_to_mavros.h

    mavros_msgs::PositionTarget pos_setpoint;

    if(cur_position[2] < 0.7)
    {
       last_time = ros::Time::now();
    }

    //起飞
    if(state_flag == 0)
    {
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + position_det[2] - 2; 
    //circle_center_pos_enu[1] = cur_position[1] - position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - position_det[1]; 
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
    if((abs(cur_position[0]) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2] - 1.0)) < 0.1)&&(ros::Time::now() - last_time > ros::Duration(10.0)))
      {
       state_flag = 1;
      }
    }


    if(state_flag == 1)
    {
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    //pos_setpoint.type_mask = 0b010111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + position_det[2] - 2.2; 
    circle_center_pos_enu[0] = cur_position[0] + position_det[2];
    circle_center_pos_enu[1] = cur_position[1] - position_det[0]; 
    circle_center_pos_enu[2] = cur_position[2] - position_det[1]; 
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
    if((abs(position_det[0])<0.05)&&(abs(position_det[1])<0.05)&&((abs(position_det[2] - 0.8))<0.2))
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
           }
      }
    }

    if(state_flag == 2)
    {
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + position_det[2] - 2.2; 
    //circle_center_pos_enu[1] = cur_position[1] - position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - position_det[1]; 
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

    }

    ros::spinOnce();
    rate.sleep();
    }

    return 0;
}


