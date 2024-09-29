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
#include <mavros_msgs/RCIn.h>
#include <offboard_pkg/DetectionInfo.h>
#include <offboard_pkg/ArucoInfo.h>

#include "math_utils.h"

#include "tic_toc.h"

#include <cstdio>
#include <vector>
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
DEFINE_double(robust_threshold,
              0.0,
              "Robust loss parameter. Set to 0 for normal squared error (no "
              "robustification).");


mavros_msgs::State current_state;

double cur_position[3]={0,0,0};
double position_det[3]={0,0,0};
double circle_center_pos_enu[3]={0,0,0};
double circle_center_pos_enu_store[3]={0,0,0};
double circle_center_pos_enu_store_avr[3]={0,0,0};

double cur_position_tempstore[3]={0,0,0};
float current_yaw_tempstore;

double aruco_pos_enu[3]={0,0,0};

bool detected_1;
bool IF_SIMULATION;

int circle_center_pos_enu_count = 0;

float current_roll;
float current_pitch;
float current_yaw;
float THRUST;

float yawratefromsim;

int state_flag = 0;
//int state_flag = 1;//调试时用

int num_regain = 0;
int num_lost = 0;
bool is_detected;
int VISION_THRES = 13;

int rc_thrust = 1500;

int state5_count = 0;

int state2_height_flag = 0;

double ceres_circle_xx_yy_tt_observe[60][3];
int ceres_circle_xx_yy_tt_observe_count = 0;

int ceres_circle_xx_yy_tt_observe_time_first_sec = 0;
int ceres_circle_xx_yy_tt_observe_time_first_nsec = 0;
double ceres_circle_xx_yy_tt_observe_time_first_sec_sum = 0;

//double x = 1.5;
//double y = 0;
//double a = 0.2;
//double b = 0;
//double initial_x = x;
//double initial_y = y;
//double initial_a = a;
//double initial_b = b;
int num_points = 0;

int ceres_flag = 0;

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

/**************************************************************
void aruco_det_cb(const offboard_pkg::ArucoInfo::ConstPtr &msg)
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
               cur_position_tempstore[0] = cur_position[0];
               cur_position_tempstore[1] = cur_position[1];
               cur_position_tempstore[2] = cur_position[2];
            }

            // 当连续一段时间检测到目标时，认定目标得到
            if(num_regain > VISION_THRES)
            {
               is_detected = true;
            }


        }
**************************************************************************/


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

void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg){
    //current_state = *msg;
    rc_thrust = msg->channels[2];
}


/************************************************************************************************
class DistanceFromCircleTrajectoryCost {
 public:
  //DistanceFromCircleCost(double xx, double yy) : xx_(xx), yy_(yy) {}
  //DistanceFromCircleCost输入的参数是观测值,我这里观测值有三个  x y t 
  DistanceFromCircleTrajectoryCost(double xx, double yy, double tt) : xx_(xx), yy_(yy), tt_(tt) {}
  template <typename T>
  bool operator()(const T* const x,
                  const T* const y,
                  const T* const a, 
                  const T* const b, 
                  T* residual) const {
    // Since the radius is parameterized as m^2, unpack m to get r.
    //T r = *m * *m;
    // Get the position of the sample in the circle's coordinate system.
    //T xp = xx_ - *x;
    //T yp = yy_ - *y;
    // It is tempting to use the following cost:
    //
    //   residual[0] = r - sqrt(xp*xp + yp*yp);
    //
    // which is the distance of the sample from the circle. This works
    // reasonably well, but the sqrt() adds strong nonlinearities to the cost
    // function. Instead, a different cost is used, which while not strictly a
    // distance in the metric sense (it has units distance^2) it produces more
    // robust fits when there are outliers. This is because the cost surface is
    // more convex.
    //residual[0] = r * r - xp * xp - yp * yp;
    //residual[0] = x - (xc + 1*cos(a*t+b))   //这里面应该就两种值，观测值和估计值，DistanceFromCircleCost输入的参数是观测值，输入的参数是估计值
    //residual[0] = xx_ - (*x + 0.5*cos(*a * tt_ + *b));
    //residual[1] = yy_ - (*y + 0.5*sin(*a * tt_ + *b));
    residual[0] = xx_ - (*x + 0.5*sin(*a * tt_ + *b));
    residual[1] = yy_ - (*y + 0.5*cos(*a * tt_ + *b));
    //residual[0] = y - (yc + 1*sin(a*t+b))
    return true;
  }
 private:
  // The measured x,y coordinate that should be on the circle.
  double xx_, yy_, tt_;
};
***********************************************************************************************/

//刚刚出现0.2优化成0.3的情况，导致飞得比较快了，基于此我觉得就强行固定为0.2，不优化此参数了 由于有时候a会被优化成和0.2距离比较大的值，这样比较危险，所以干脆不优化a了，就固定成0.2把，这个短时间内跟一下应该没有问题，主要优化相位b吧。
class DistanceFromCircleTrajectoryCost {
 public:
  //DistanceFromCircleCost(double xx, double yy) : xx_(xx), yy_(yy) {}
  //DistanceFromCircleCost输入的参数是观测值,我这里观测值有三个  x y t 
  DistanceFromCircleTrajectoryCost(double xx, double yy, double tt) : xx_(xx), yy_(yy), tt_(tt) {}
  template <typename T>
  bool operator()(const T* const x,
                  const T* const y, 
                  const T* const b, 
                  T* residual) const {
    // Since the radius is parameterized as m^2, unpack m to get r.
    //T r = *m * *m;
    // Get the position of the sample in the circle's coordinate system.
    //T xp = xx_ - *x;
    //T yp = yy_ - *y;
    // It is tempting to use the following cost:
    //
    //   residual[0] = r - sqrt(xp*xp + yp*yp);
    //
    // which is the distance of the sample from the circle. This works
    // reasonably well, but the sqrt() adds strong nonlinearities to the cost
    // function. Instead, a different cost is used, which while not strictly a
    // distance in the metric sense (it has units distance^2) it produces more
    // robust fits when there are outliers. This is because the cost surface is
    // more convex.
    //residual[0] = r * r - xp * xp - yp * yp;
    //residual[0] = x - (xc + 1*cos(a*t+b))   //这里面应该就两种值，观测值和估计值，DistanceFromCircleCost输入的参数是观测值，输入的参数是估计值
    //residual[0] = xx_ - (*x + 0.5*cos(*a * tt_ + *b));
    //residual[1] = yy_ - (*y + 0.5*sin(*a * tt_ + *b));
    residual[0] = xx_ - (*x + 0.5*sin(0.2 * tt_ + *b));
    residual[1] = yy_ - (*y + 0.5*cos(0.2 * tt_ + *b));
    //residual[0] = y - (yc + 1*sin(a*t+b))
    return true;
  }
 private:
  // The measured x,y coordinate that should be on the circle.
  double xx_, yy_, tt_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_track");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    //float yaw_rate_P;
    //float roll_rate_P;
    //float pitch_rate_P;
    double X,Y,A,B;
    

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //ros::Publisher yawrate_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    //需要订阅椭圆检测结果话题 /drone/object_detection/ellipse_det
    //ros::Subscriber ellipse_det_sub = nh.subscribe<offboard_pkg::DetectionInfo>("/drone/object_detection/ellipse_det", 10, ellipse_det_cb);

    //ros::Subscriber ellipse_det_sub = nh.subscribe<offboard_pkg::ArucoInfo>("/drone/object_detection/aruco_det", 10, aruco_det_cb);

    ros::Subscriber multi_aruco_det_sub = nh.subscribe<offboard_pkg::DetectionInfo>("/drone/object_detection/multi_aruco_det", 10, multi_aruco_det_cb);

    //ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, rc_cb);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    nh1.param<double>("x", X, 1.5);
    nh1.param<double>("y", Y, 0);
    nh1.param<double>("a", A, 0.2);
    nh1.param<double>("b", B, 0);
    //nh1.param<float>("thrust", THRUST, 0.5);
    //nh1.param<bool>("if_simulation", IF_SIMULATION, false);

    double x = x;
    double y = Y;
    double a = A;
    double b = B;
    double initial_x = x;
    double initial_y = y;
    double initial_a = a;
    double initial_b = b;


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

    mavros_msgs::CommandBool disarm_cmd;
    //arm_cmd.request.value = true;
    disarm_cmd.request.value = false; //这样是否就是上锁

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::SetMode manual_set_mode;
    manual_set_mode.request.custom_mode = "MANUAL";


    ros::Time last_time = ros::Time::now();
    ros::Time time_snap0 = ros::Time::now();
    ros::Time time_snap1 = ros::Time::now();
    ros::Time ceres_complete = ros::Time::now();
    ros::Time ceres_circle_xx_yy_tt_observe_time = ros::Time::now();
    ros::Time ceres_circle_xx_yy_tt_observe_time_first = ros::Time::now();

      Problem problem;
      // Configure the loss function.
      LossFunction* loss = nullptr;
      if (CERES_GET_FLAG(FLAGS_robust_threshold)) {
        loss = new CauchyLoss(CERES_GET_FLAG(FLAGS_robust_threshold));
      }
      // Add the residuals.
      double xx, yy, tt;

    while(ros::ok()){

    //参考自：Prometheus/master/Modules/control/include/command_to_mavros.h

    mavros_msgs::PositionTarget pos_setpoint;

    if(cur_position[2] < 0.3)
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
    pos_setpoint.position.z = 0.7;
    //pos_setpoint.position.z = 1;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //定点悬停多少秒之后再进入下一个状态进行圆跟踪  10秒
    if((abs(cur_position[0]) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-0.7)) < 0.1)&&(ros::Time::now() - last_time > ros::Duration(10.0)))
      {
       //state_flag = 1;
       state_flag = 22;
       time_snap0 = ros::Time::now();
      }
    }

    if(state_flag == 22)
    {
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + position_det[2] - 2; 
    //circle_center_pos_enu[1] = cur_position[1] - position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - position_det[1]; 
    //printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0]);
    //printf("circle_center_pos_enu[1] is %f\n", circle_center_pos_enu[1]);
    //printf("circle_center_pos_enu[2] is %f\n", circle_center_pos_enu[2]);

    pos_setpoint.position.x = 1;
    pos_setpoint.position.y = 0;
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = 0.7;
    //pos_setpoint.position.z = 1;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

      if((abs(cur_position[0] - 1) < 0.1)&&(abs(cur_position[1]) < 0.1)&&((abs(cur_position[2]-0.7)) < 0.1)&&(ros::Time::now() - time_snap0 > ros::Duration(10.0)))
      {
       //state_flag = 1;
       state_flag = 1;
      }

    }




/*******************************************************************************************************************************
    if(state_flag == 1)
    {
       //如果丢失二维码，则进入悬停状态
       if(is_detected == false)
        {
           state_flag = 5;
        }
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    //pos_setpoint.type_mask = 0b010111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + position_det[2] - 2.2; 
    //circle_center_pos_enu[0] = cur_position[0] + position_det[2];
    //circle_center_pos_enu[1] = cur_position[1] - position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - position_det[1]; 
    //坐标系确认好
    aruco_pos_enu[0] = cur_position[0] - position_det[1];
    aruco_pos_enu[1] = cur_position[1] - position_det[0]; 
    aruco_pos_enu[2] = cur_position[2] - position_det[2]; 
    //printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0] + 2.2);
    printf("aruco_pos_enu[0] is %f\n", aruco_pos_enu[0]);
    printf("aruco_pos_enu[1] is %f\n", aruco_pos_enu[1]);
    printf("aruco_pos_enu[2] is %f\n", aruco_pos_enu[2]);

    pos_setpoint.position.x = aruco_pos_enu[0];
    pos_setpoint.position.y = aruco_pos_enu[1];
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = aruco_pos_enu[2] + 0.6;
    //pos_setpoint.position.z = aruco_pos_enu[2] + 0.3; //基于两个大小叠加二维码可以0.3米的高度稳定跟住，不丢。
    //pos_setpoint.position.z = aruco_pos_enu[2] + 0.2;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    if(is_detected == true)
    {
    //每间隔一秒取一个点
    if((ros::Time::now() - ceres_circle_xx_yy_tt_observe_time > ros::Duration(1.0))&&(ceres_circle_xx_yy_tt_observe_count < 50))
    {
     ceres_circle_xx_yy_tt_observe[ceres_circle_xx_yy_tt_observe_count][0] = aruco_pos_enu[0];
     ceres_circle_xx_yy_tt_observe[ceres_circle_xx_yy_tt_observe_count][1] = aruco_pos_enu[1];

     //如何获得两个时间戳的差值呢
     ceres_circle_xx_yy_tt_observe_time = ros::Time::now();
     if(ceres_circle_xx_yy_tt_observe_count == 0)
      {
       ceres_circle_xx_yy_tt_observe_time_first = ceres_circle_xx_yy_tt_observe_time;
       ceres_circle_xx_yy_tt_observe_time_first_sec = ceres_circle_xx_yy_tt_observe_time_first.sec;
       ceres_circle_xx_yy_tt_observe_time_first_nsec = ceres_circle_xx_yy_tt_observe_time_first.nsec;
       ceres_circle_xx_yy_tt_observe_time_first_sec_sum = ceres_circle_xx_yy_tt_observe_time_first_sec + ceres_circle_xx_yy_tt_observe_time_first_nsec*(1e-9);
      }
     int sec = ceres_circle_xx_yy_tt_observe_time.sec;
     int nsec = ceres_circle_xx_yy_tt_observe_time.nsec;
     double sec_sum = sec + nsec*(1e-9);
     double sec_sum_relate = sec_sum - ceres_circle_xx_yy_tt_observe_time_first_sec_sum;
     ceres_circle_xx_yy_tt_observe[ceres_circle_xx_yy_tt_observe_count][2] = sec_sum_relate;
     ceres_circle_xx_yy_tt_observe_count = ceres_circle_xx_yy_tt_observe_count + 1;
    }
    
    TicToc t_ceres;
    if((ceres_circle_xx_yy_tt_observe_count == 50)&&(ceres_flag == 0))
    {
      //double initial_x = x;
      //double initial_y = y;
      //double initial_a = a;
      //double initial_b = b;
      Problem problem;
      // Configure the loss function.
      LossFunction* loss = nullptr;
      if (CERES_GET_FLAG(FLAGS_robust_threshold)) {
        loss = new CauchyLoss(CERES_GET_FLAG(FLAGS_robust_threshold));
      }
      // Add the residuals.
      double xx, yy, tt;
      //int num_points = 0;
      for(int residual_count = 0;residual_count++;residual_count<50)
      {
        xx = ceres_circle_xx_yy_tt_observe[residual_count][0];
        yy = ceres_circle_xx_yy_tt_observe[residual_count][1];
        tt = ceres_circle_xx_yy_tt_observe[residual_count][2];
        CostFunction* cost =
           new AutoDiffCostFunction<DistanceFromCircleTrajectoryCost, 2, 1, 1, 1, 1>(
              new DistanceFromCircleTrajectoryCost(xx, yy, tt));
        problem.AddResidualBlock(cost, loss, &x, &y, &a, &b);
        num_points++;
        std::cout << "num_points: " << num_points << "\n";
      }
      std::cout << "Got " << num_points << " points.\n";
      // Build and solve the problem.
      Solver::Options options;
      options.max_num_iterations = 500;
      options.linear_solver_type = ceres::DENSE_QR;
      Solver::Summary summary;
      Solve(options, &problem, &summary);
      std::cout << summary.BriefReport() << "\n";
      std::cout << "x : " << initial_x << " -> " << x << "\n";
      std::cout << "y : " << initial_y << " -> " << y << "\n";
      std::cout << "a : " << initial_a << " -> " << a << "\n";
      std::cout << "b : " << initial_b << " -> " << b << "\n";

     
      ceres_flag = 1;
    }
    ROS_INFO("ceres costs: %fms", t_ceres.toc());
    }
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    std::cout << "y : " << initial_y << " -> " << y << "\n";
    std::cout << "a : " << initial_a << " -> " << a << "\n";
    std::cout << "b : " << initial_b << " -> " << b << "\n";
    std::cout << "num_points: " << num_points << "\n";
    std::cout << "ceres_circle_xx_yy_tt_observe_count: " << ceres_circle_xx_yy_tt_observe_count << "\n";
    std::cout << "ceres_flag: " << ceres_flag << "\n";


    if((abs(position_det[0]) < 0.05)&&(abs(position_det[1]) < 0.05))
      {
        //为了跑跟踪暂时注释掉
        //state_flag = 2;
      }
  
    }
********************************************************************************************************************/
    
    if(state_flag == 1)
    {
       //如果丢失二维码，则进入悬停状态
       if(is_detected == false)
        {
           state_flag = 5;
           continue;
        }
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    //pos_setpoint.type_mask = 0b010111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + position_det[2] - 2.2; 
    //circle_center_pos_enu[0] = cur_position[0] + position_det[2];
    //circle_center_pos_enu[1] = cur_position[1] - position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - position_det[1]; 
    //坐标系确认好
    aruco_pos_enu[0] = cur_position[0] - position_det[1];
    aruco_pos_enu[1] = cur_position[1] - position_det[0]; 
    aruco_pos_enu[2] = cur_position[2] - position_det[2]; 
    //printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0] + 2.2);
    printf("aruco_pos_enu[0] is %f\n", aruco_pos_enu[0]);
    printf("aruco_pos_enu[1] is %f\n", aruco_pos_enu[1]);
    printf("aruco_pos_enu[2] is %f\n", aruco_pos_enu[2]);

    pos_setpoint.position.x = aruco_pos_enu[0];
    pos_setpoint.position.y = aruco_pos_enu[1];
    //pos_setpoint.position.z = 0;
    //pos_setpoint.position.z = aruco_pos_enu[2] + 0.6;
    pos_setpoint.position.z = aruco_pos_enu[2] + 0.3;
    //pos_setpoint.position.z = aruco_pos_enu[2] + 0.3; //基于两个大小叠加二维码可以0.3米的高度稳定跟住，不丢。
    //pos_setpoint.position.z = aruco_pos_enu[2] + 0.2;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //if(is_detected == true)
    //{
    //每间隔一秒取一个点
    if((ros::Time::now() - ceres_circle_xx_yy_tt_observe_time > ros::Duration(1.0))&&(ceres_circle_xx_yy_tt_observe_count < 60))
    {
     ceres_circle_xx_yy_tt_observe[ceres_circle_xx_yy_tt_observe_count][0] = aruco_pos_enu[0];
     ceres_circle_xx_yy_tt_observe[ceres_circle_xx_yy_tt_observe_count][1] = aruco_pos_enu[1];

     //如何获得两个时间戳的差值呢
     ceres_circle_xx_yy_tt_observe_time = ros::Time::now();
     if(ceres_circle_xx_yy_tt_observe_count == 0)
      {
       ceres_circle_xx_yy_tt_observe_time_first = ceres_circle_xx_yy_tt_observe_time;
       ceres_circle_xx_yy_tt_observe_time_first_sec = ceres_circle_xx_yy_tt_observe_time_first.sec;
       ceres_circle_xx_yy_tt_observe_time_first_nsec = ceres_circle_xx_yy_tt_observe_time_first.nsec;
       ceres_circle_xx_yy_tt_observe_time_first_sec_sum = ceres_circle_xx_yy_tt_observe_time_first_sec + ceres_circle_xx_yy_tt_observe_time_first_nsec*(1e-9);
      }
     int sec = ceres_circle_xx_yy_tt_observe_time.sec;
     int nsec = ceres_circle_xx_yy_tt_observe_time.nsec;
     double sec_sum = sec + nsec*(1e-9);
     double sec_sum_relate = sec_sum - ceres_circle_xx_yy_tt_observe_time_first_sec_sum;
     ceres_circle_xx_yy_tt_observe[ceres_circle_xx_yy_tt_observe_count][2] = sec_sum_relate;

        xx = ceres_circle_xx_yy_tt_observe[ceres_circle_xx_yy_tt_observe_count][0];
        yy = ceres_circle_xx_yy_tt_observe[ceres_circle_xx_yy_tt_observe_count][1];
        tt = ceres_circle_xx_yy_tt_observe[ceres_circle_xx_yy_tt_observe_count][2];
        CostFunction* cost =
           new AutoDiffCostFunction<DistanceFromCircleTrajectoryCost, 2, 1, 1, 1>(
              new DistanceFromCircleTrajectoryCost(xx, yy, tt));
        problem.AddResidualBlock(cost, loss, &x, &y, &b);
        num_points++;

     ceres_circle_xx_yy_tt_observe_count = ceres_circle_xx_yy_tt_observe_count + 1;
    }
    
    TicToc t_ceres;
   
    if((ceres_circle_xx_yy_tt_observe_count == 60)&&(ceres_flag == 0))
    {
      //double initial_x = x;
      //double initial_y = y;
      //double initial_a = a;
      //double initial_b = b;

      //int num_points = 0;
       /*************************************************************************
      for(int residual_count = 0;residual_count++;residual_count<50)
      {
        xx = ceres_circle_xx_yy_tt_observe[residual_count][0];
        yy = ceres_circle_xx_yy_tt_observe[residual_count][1];
        tt = ceres_circle_xx_yy_tt_observe[residual_count][2];
        CostFunction* cost =
           new AutoDiffCostFunction<DistanceFromCircleTrajectoryCost, 2, 1, 1, 1, 1>(
              new DistanceFromCircleTrajectoryCost(xx, yy, tt));
        problem.AddResidualBlock(cost, loss, &x, &y, &a, &b);
        num_points++;
        std::cout << "num_points: " << num_points << "\n";
      }
      ***************************************************************************/
      std::cout << "Got " << num_points << " points.\n";
      // Build and solve the problem.
      Solver::Options options;
      options.max_num_iterations = 500;
      options.linear_solver_type = ceres::DENSE_QR;
      Solver::Summary summary;
      Solve(options, &problem, &summary);
      std::cout << summary.BriefReport() << "\n";
      std::cout << "x : " << initial_x << " -> " << x << "\n";
      std::cout << "y : " << initial_y << " -> " << y << "\n";
      std::cout << "a : " << initial_a << " -> " << a << "\n";
      std::cout << "b : " << initial_b << " -> " << b << "\n";

     
      ceres_flag = 1;
      ceres_complete = ros::Time::now();
    }
    ROS_INFO("ceres costs: %fms", t_ceres.toc());
    //}
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    std::cout << "y : " << initial_y << " -> " << y << "\n";
    std::cout << "a : " << initial_a << " -> " << a << "\n";
    std::cout << "b : " << initial_b << " -> " << b << "\n";
    std::cout << "num_points: " << num_points << "\n";
    std::cout << "ceres_circle_xx_yy_tt_observe_count: " << ceres_circle_xx_yy_tt_observe_count << "\n";
    std::cout << "ceres_flag: " << ceres_flag << "\n";


    //if((abs(position_det[0]) < 0.05)&&(abs(position_det[1]) < 0.05))
      {
        //为了跑跟踪暂时注释掉
        //state_flag = 2;
      }
      if((ceres_flag == 1)&&(ros::Time::now() - ceres_complete > ros::Duration(5.0)))
      {
        //state_flag = 11;
        state_flag = 2;
        time_snap1 = ros::Time::now();
      }
  
    }


    //基于ceres优化出的参数得到的圆轨迹方程进行指点飞圆轨迹，不跟二维码飞。
    if(state_flag == 11)
    {
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + position_det[2] - 2; 
    //circle_center_pos_enu[1] = cur_position[1] - position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - position_det[1]; 
    ros::Time now_time = ros::Time::now();
     int now_time_sec = now_time.sec;
     int now_time_nsec = now_time.nsec;
     double now_time_sec_sum = now_time_sec + now_time_nsec*(1e-9);
     double now_time_relate = now_time_sec_sum - ceres_circle_xx_yy_tt_observe_time_first_sec_sum;


    pos_setpoint.position.x = x + 0.5*sin(a*now_time_relate + b);
    pos_setpoint.position.y = y + 0.5*cos(a*now_time_relate + b);
    //pos_setpoint.position.z = 0;
    //pos_setpoint.position.z = 0.7;
    pos_setpoint.position.z = 0.6;
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    }


    /********************************************************************************
    if(state_flag == 2)
    {
    //pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    //pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + position_det[2] - 2.2; 
    //circle_center_pos_enu[1] = cur_position[1] - position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - position_det[1]; 
    printf("aruco_pos_enu[0] is %f\n", aruco_pos_enu[0]);
    printf("aruco_pos_enu[1] is %f\n", aruco_pos_enu[1]);
    printf("aruco_pos_enu[2] is %f\n", aruco_pos_enu[2]);

    //pos_setpoint.position.x = circle_center_pos_enu_store_avr[0] + 1 + 2.2;
    //pos_setpoint.position.x = circle_center_pos_enu_store_avr[0] + 1;  //飞到圆心后一米的位置
    //pos_setpoint.position.y = circle_center_pos_enu_store_avr[1];
    //pos_setpoint.position.z = 0;
    //pos_setpoint.position.z = circle_center_pos_enu_store_avr[2];
    //pos_setpoint.yaw = 0;
    //setpoint_raw_local_pub.publish(pos_setpoint);
 
         set_mode_client.call(land_set_mode);
         if(land_set_mode.response.mode_sent)
           {
             ROS_INFO("land enabled");          
           }  
    }
    **********************************************************************************/
    if(state_flag == 2)
    {
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;

    //circle_center_pos_enu[0] = cur_position[0] + position_det[2] - 2; 
    //circle_center_pos_enu[1] = cur_position[1] - position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - position_det[1]; 
    ros::Time now_time = ros::Time::now();
     int now_time_sec = now_time.sec;
     int now_time_nsec = now_time.nsec;
     double now_time_sec_sum = now_time_sec + now_time_nsec*(1e-9);
     double now_time_relate = now_time_sec_sum - ceres_circle_xx_yy_tt_observe_time_first_sec_sum;

     int time_snap1_sec = time_snap1.sec;
     int time_snap1_nsec = time_snap1.nsec;
     double time_snap1_sec_sum = time_snap1_sec + time_snap1_nsec*(1e-9);
     double now_time_relate_from_state1 = now_time_sec_sum - time_snap1_sec_sum;


    pos_setpoint.position.x = x + 0.5*sin(a*now_time_relate + b);
    pos_setpoint.position.y = y + 0.5*cos(a*now_time_relate + b);
    //pos_setpoint.position.z = 0;
    //pos_setpoint.position.z = 0.7;
    pos_setpoint.position.z = 0.6 - now_time_relate_from_state1*0.05;
     if(cur_position[2] < 0.41)
     {
       state2_height_flag = 1;  //防止高度不小心因为控制偏差再飞到0.4米以上的时候，又再次按照上面基于时间的下降指点，这样会造成突变，可能会让无人机在0.4米高度来回跳动。
     }
     //if((cur_position[2] < 0.4)&&(rc_thrust >= 1250)&&(state2_height_flag == 1))
     //if((state2_height_flag == 1)&&(rc_thrust >= 1250))
     //不用if((state2_height_flag == 1)&&(rc_thrust >= 1250))是因为会有(state2_height_flag == 1)&&(rc_thrust < 1250)的情况出现，就是我油门打低后第一次循环执行state2的时候，可能会出现期望高度又变为之前按照时间降低的高度，可能会发生期望高度的跳变，虽然这样只会执行一次，因为再执行到下面就会跳转到state7切manual降落了，但是有这么一次我觉得也较为危险，所以改为仅剩state2_height_flag == 1的条件。
     if(state2_height_flag == 1)
     {
        pos_setpoint.position.z = 0.4;
     }
    pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    //if(cur_position[2] < 0.4)
     //if((cur_position[2] < 0.4)&&(rc_thrust < 1250))
     if((cur_position[2] < 0.43)&&(rc_thrust < 1250))
     {
        state_flag = 7;
     }

    }


    //没有检测到二维码时保持悬停
    if(state_flag == 5)
    {
    state5_count = state5_count + 1;
    if(state5_count == 1)
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

    //circle_center_pos_enu[0] = cur_position[0] + position_det[2] - 2; 
    //circle_center_pos_enu[1] = cur_position[1] - position_det[0]; 
    //circle_center_pos_enu[2] = cur_position[2] - position_det[1]; 
    //printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0]);
    //printf("circle_center_pos_enu[1] is %f\n", circle_center_pos_enu[1]);
    //printf("circle_center_pos_enu[2] is %f\n", circle_center_pos_enu[2]);
    //这个悬停说实话不是很严谨，可能有累计偏移。
    //pos_setpoint.position.x = cur_position[0];
    //pos_setpoint.position.y = cur_position[1];
    //pos_setpoint.position.z = 0;
    //pos_setpoint.position.z = cur_position[2];
    //这个悬停比较严谨
    pos_setpoint.position.x = cur_position_tempstore[0];
    pos_setpoint.position.y = cur_position_tempstore[1];
    //pos_setpoint.position.z = 0;
    pos_setpoint.position.z = cur_position_tempstore[2];
    pos_setpoint.yaw = current_yaw_tempstore;
    setpoint_raw_local_pub.publish(pos_setpoint);


    if(is_detected == true)
     {
       state_flag = 1;
       state5_count = 0;
     }

    }


    //通过service进行上锁。
    if(state_flag == 6)
    {
            if(current_state.armed){
                if( arming_client.call(disarm_cmd) &&
                    disarm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");
                }
             }
    }

    //通过service先切manual，再上锁。此处切换会manual模式是因为:PX4默认在offboard模式且有控制的情况下没法上锁,所以先切为manual再上锁
    if(state_flag == 7)
    {
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



    ros::spinOnce();
    rate.sleep();
    }

    return 0;
}


