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

#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

struct CICLE_FITTING_COST
{
    CICLE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const x0y0r,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        //(x-x0)^2+(y-y0)^2-r^2
        residual[0]=x0y0r[2]-(T ( _x )-x0y0r[0])*(T ( _x )-x0y0r[0])-(T ( _y )-x0y0r[1])*(T ( _y )-x0y0r[1]);
        return true;
    }
    const double _x, _y;    // x,y数据
};


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_fit");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
  
    //*************************************************************************************************

    double x0=-3.0, y0=-4.0, r=2.0;         // 真实参数值
    int N=100;                          // 数据点
    double w_sigma=0.001;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double x0y0r[3] = {0,0,0};            // abc参数的估计值

    vector<double> x_data, y_data;      // 数据

    cout<<"generating data: "<<endl;

    for ( int i=0; i<N; i++ )
    {
        double x = x0+r*cos(i*M_PI/50);
        double noise_y=y0+r*sin(i*M_PI/50)+rng.gaussian ( w_sigma );
        x_data.push_back ( x );
        y_data.push_back (noise_y);
        cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }
    ceres::Problem problem;
    for ( int i=0; i<N; i++ )
    {
        problem.AddResidualBlock (     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<CICLE_FITTING_COST, 1, 3> (
                new CICLE_FITTING_COST ( x_data[i], y_data[i] )
            ),
            nullptr,            // 核函数，这里不使用，为空
            x0y0r                 // 待估计参数
        );
    }
    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;
    // 输出结果
    cout<<summary.BriefReport() <<endl;
//    cout<<"estimated x0,y0,r = ";
    cout<<"x0 "<<x0y0r[0]<<"y0= "<<x0y0r[1]<<" r= "<<sqrt(x0y0r[2]);
    return 0;


    //***************************************************************************************************
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
      
        //local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


