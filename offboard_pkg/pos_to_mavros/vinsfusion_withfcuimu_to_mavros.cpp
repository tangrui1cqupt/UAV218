/**
在跑基于飞控imu的vinsfusion的时候用的，把vins输出的odom转为/mavros/vision_pose/pose发给mavros
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>


Eigen::Vector3d pos_drone_vins;
Eigen::Quaterniond q_vins;



void vins_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    //if (msg->header.frame_id == "t265_odom_frame")
    {
        pos_drone_vins = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        // pos_drone_t265[0] = msg->pose.pose.position.x + pos_offset[0];
        // pos_drone_t265[1] = msg->pose.pose.position.y + pos_offset[1];
        // pos_drone_t265[2] = msg->pose.pose.position.z + pos_offset[2];

        q_vins = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        //Euler_t265 = quaternion_to_euler(q_gazebo);
        // Euler_t265[2] = Euler_t265[2] + yaw_offset;
        // q_t265 = quaternion_from_rpy(Euler_t265);
    }
    //else
    //{
        
    //}
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "vinsfusion_to_mavros");
    ros::NodeHandle nh;


        //  【订阅】vins估计位置
    ros::Subscriber vins_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 100, vins_cb);

    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);



    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
    



        geometry_msgs::PoseStamped vision;

        vision.pose.position.x = pos_drone_vins[0];
        vision.pose.position.y = pos_drone_vins[1];
        vision.pose.position.z = pos_drone_vins[2];

        vision.pose.orientation.x = q_vins.x();
        vision.pose.orientation.y = q_vins.y();
        vision.pose.orientation.z = q_vins.z();
        vision.pose.orientation.w = q_vins.w();


        vision.header.stamp = ros::Time::now();
        vision_pub.publish(vision);



  

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


