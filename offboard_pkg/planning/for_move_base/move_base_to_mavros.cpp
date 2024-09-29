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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>


mavros_msgs::State current_state;

double position[3]={0,0,0};
double position_det[3]={0,0,0};
bool detected_1;

int point = 1;
double x_vel_set = 0;
double y_vel_set = 0;

//bool first_time = true;
int first_time_flag = 1;
//string current_mode;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            position[0] = msg->pose.position.x;
            position[1] = msg->pose.position.y;
            position[2] = msg->pose.position.z;
        }

void move_base_vel_cb(const geometry_msgs::Twist::ConstPtr &msg)
        {
            x_vel_set = msg->linear.x;
            y_vel_set = msg->linear.y;
        }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_to_mavros");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    float yaw_rate_P;
    float A ;
    float w ;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
    ros::Subscriber move_base_vel_sub = nh.subscribe<geometry_msgs::Twist>("/px4_vel", 10, move_base_vel_cb);
    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::Publisher setpoint_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    //需要订阅椭圆检测结果话题 /drone/object_detection/ellipse_det
    //ros::Subscriber ellipse_det_sub = nh.subscribe<offboard_pkg::DetectionInfo>("/drone/object_detection/ellipse_det", 10, ellipse_det_cb);

    //ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    nh1.param<float>("yaw_rate_P", yaw_rate_P, 0.05);
    nh1.param<float>("A", A, 1.0);
    nh1.param<float>("w", w, 1.0);
    

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

    //ros::Time begin_time = ros::Time::now();
    ros::Time begin_time; 

    while(ros::ok()){

    //参考自：Prometheus/master/Modules/control/include/command_to_mavros.h

    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    //pos_setpoint.type_mask = 0b010111111000;  // 100 111 111 000  xyz + yaw
    //pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    //pos_setpoint.type_mask = 0b110111111000; // only x y z

    // 此处由于飞控暂不支持位置－速度追踪的复合模式，因此type_mask设定如下
    pos_setpoint.type_mask = 0b100111000011;   // 100 111 000 011  vx vy vz z + yaw


    pos_setpoint.coordinate_frame = 1;

    //pos_setpoint.position.x = 1;
    //pos_setpoint.position.y = 0;
    //pos_setpoint.position.z = 0;
    //pos_setpoint.position.z = 0;
    pos_setpoint.velocity.x = x_vel_set;
    pos_setpoint.velocity.y = y_vel_set;
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.position.z = 0.3;


    setpoint_raw_local_pub.publish(pos_setpoint);


    //ros::Time begin_time; 

    /*********************************************************************
    if((current_state.mode=="OFFBOARD")&& (first_time_flag == 1))
    {
     begin_time = ros::Time::now();     //记录刚切offboard的时间
     first_time_flag=first_time_flag+1;    
    }
    if(current_state.mode!="OFFBOARD")
    {
      first_time_flag = 1;
    }
    *********************************************************************/
    //float A = 1;
    //float w = 1;
    //x_vel_set = A*sin(w*(ros::Time::now().toSec() - begin_time.toSec() ));
    //y_vel_set = A*cos(w*(ros::Time::now().toSec() - begin_time.toSec() ));
    //x_vel_set = A*sin(w*(ros::Time::now().toSec() - begin_time.toSec() )+1.5708); //加上二分之pai
    //y_vel_set = A*cos(w*(ros::Time::now().toSec() - begin_time.toSec() )+1.5708); //加上二分之pai
    //printf("%16f\n",ros::Time::now().toSec());

    //if(current_state.mode!="OFFBOARD")
    //{
    //   x_vel_set=0;
    //   y_vel_set=0;    //确保刚切offboard的时候初始速度是0
    //}

    //geometry_msgs::TwistStamped vel_setpoint;
    //vel_setpoint.twist.linear.x = x_vel_set;
    //vel_setpoint.twist.linear.y = y_vel_set;
    //vel_setpoint.header.stamp = ros::Time::now();
    
    //setpoint_velocity_pub.publish(vel_setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}




