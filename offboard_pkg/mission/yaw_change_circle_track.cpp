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

double position[3]={0,0,0};
double position_det[3]={0,0,0};
bool detected_1;
bool IF_SIMULATION;

float current_roll;
float current_pitch;
float current_yaw;
float THRUST;

float yawratefromsim;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            position[0] = msg->pose.position.x;
            position[1] = msg->pose.position.y;
            position[2] = msg->pose.position.z;
    
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

void yawratefromsim_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
        {   
             yawratefromsim=msg->yaw_rate;
        }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    float yaw_rate_P;
    float roll_rate_P;
    float pitch_rate_P;
    

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher yawrate_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    //需要订阅椭圆检测结果话题 /drone/object_detection/ellipse_det
    ros::Subscriber ellipse_det_sub = nh.subscribe<offboard_pkg::DetectionInfo>("/drone/object_detection/ellipse_det", 10, ellipse_det_cb);

    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    nh1.param<float>("yaw_rate_P", yaw_rate_P, 0.05);
    nh1.param<float>("roll_rate_P", roll_rate_P, 1);
    nh1.param<float>("pitch_rate_P", pitch_rate_P, 1);
    nh1.param<float>("thrust", THRUST, 0.5);
    nh1.param<bool>("if_simulation", IF_SIMULATION, false);

    //if(IF_SIMULATION==true)
    //{
    ros::Subscriber yawratefromsim_sub = nh.subscribe<mavros_msgs::PositionTarget>("circle/rslt/yawrate", 10, yawratefromsim_cb);
    //}

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    //while(ros::ok() && !current_state.connected){
    //    ros::spinOnce();
    //    rate.sleep();
    //}

    //geometry_msgs::PoseStamped pose;
    //pose.pose.position.x = 0;
    //pose.pose.position.y = 0;
    //pose.pose.position.z = 1;

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

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

    //参考自：Prometheus/master/Modules/control/include/command_to_mavros.h

    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b010111111000;  // 100 111 111 000  xyz + yaw
    //pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = 0;
    pos_setpoint.position.y = 0;
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = 1;




    mavros_msgs::AttitudeTarget  yawrate_thrust;
    yawrate_thrust.type_mask=128;
    //既然是负反馈，角度为正，那么期望角速度就得为负，反方向的，所以可以暂时不用核对坐标系，加个负号期望角速度方向应该就没错。
    yawrate_thrust.body_rate.x=-1*roll_rate_P*current_roll;
    yawrate_thrust.body_rate.y=-1*pitch_rate_P*current_pitch;
    yawrate_thrust.thrust=THRUST;

    if(IF_SIMULATION==false)
    {
    //pos_setpoint.yaw = 0;
    if(detected_1==true)
    {
    //pos_setpoint.yaw_rate = -0.05*position_det[0]; //这里就弄一个简单的比例负反馈控制就可以了，还注意需要考虑没有检测到圆的时候就没有角速度，无人机不会转。单位是弧度每秒。
    // pos_setpoint.yaw_rate = -1*yaw_rate_P*position_det[0];
    pos_setpoint.yaw_rate = -1*yaw_rate_P*position_det[0];
    yawrate_thrust.body_rate.z=-1*yaw_rate_P*position_det[0];
    }
    else{
    pos_setpoint.yaw_rate = 0 ; 
    yawrate_thrust.body_rate.z=0; 
    }
    //pos_setpoint.yaw = 0 ;
    }
    if(IF_SIMULATION==true)
    {
    yawrate_thrust.body_rate.z=yawratefromsim;
    //yawrate_thrust.body_rate.z=0.1;
    }
    //setpoint_raw_local_pub.publish(pos_setpoint);
    yawrate_thrust_pub.publish(yawrate_thrust);


        geometry_msgs::PoseStamped vision;

        vision.pose.position.x = 0;
        vision.pose.position.y = 0;
        vision.pose.position.z = 0;
        //vision.pose.position.x = position[0];
        //vision.pose.position.y = position[1];
        //vision.pose.position.z = position[2];


        vision.pose.orientation.x = 0;
        vision.pose.orientation.y = 0;
        vision.pose.orientation.z = 0;
        vision.pose.orientation.w = 1;


        vision.header.stamp = ros::Time::now();
        vision_pub.publish(vision);



        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


