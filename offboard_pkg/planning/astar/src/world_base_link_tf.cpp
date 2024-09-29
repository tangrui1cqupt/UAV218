#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

//std::string turtle_name;
// https://mp.weixin.qq.com/s/urfVkw57E5EqU6-dD2uDLg

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
   static tf::TransformBroadcaster br;
   tf::Transform transform;
   transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
   //tf::Quaternion quaternion;
    transform.setRotation( tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w) );
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

int main(int argc, char** argv){
   ros::init(argc, argv, "my_tf_broadcaster");
  // if (argc != 2) {
    // ROS_ERROR("need turtle name as argument");
    // return -1;
  //};
  // turtle_name = argv[1];

   ros::NodeHandle node;
   ros::Subscriber sub = node.subscribe("/mavros/vision_pose/pose", 10, &poseCallback);

   ros::spin();
   return 0;
};


