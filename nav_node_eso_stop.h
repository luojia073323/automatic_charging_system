#ifndef NAV_NODE_H
#define NAV_NODE_H

#include <nav_msgs/Path.h> 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <queue>
#include "ros/ros.h"
#include <tf/tf.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <mutex>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <algorithm> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <stdlib.h>
#include <dirent.h>
#include <sstream>
#include <csignal>
#include <iomanip>
#include <sys/types.h>
#include <sys/stat.h>
#include "MPC.h"
#include <cmath>
#include <cppad/ipopt/solve.hpp>
#include <cppad/cppad.hpp>
#include <limits>  // 引入 limits 库以使用 std::numeric_limits

#define pi 3.1415926
std::mutex mtx;
std::mutex o_mtx;

struct point
{
    int index;
    double x;
    double y;
    double yaw;
};

class NavNode
{
public:
    NavNode();
    void navigateTo(double x, double y, double yaw);
    void addNavigationTarget(double x, double y, double yaw);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void laserCloudHandler(const sensor_msgs::LaserScan::ConstPtr& pc);
    void performNavigation();

    void start_to_pause();
    void pause_to_start();
    void mpc_control(double x2, double y2, double yaw2);
    MPC mpc_tool;
	ros::NodeHandle nh;

	// //new eso
	// vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double ref_v) {
	// 	// 代价函数增加扰动项权重
	// 	for (int t = 0; t < N; t++) {
	// 		fg[0] += 0.1 * CppAD::pow(vars[disturbance_start + t], 2);
	// 	}

	// 	// 状态方程加入扰动影响
	// 	AD<double> disturbance = vars[disturbance_start + t - 1];
	// 	x1 += (v * CppAD::cos(psi1) + disturbance) * dt;
	// }

private:
    
    ros::Publisher cmd_vel_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber laser_sub;

    ros::Subscriber path_sub;  // new
	std::vector<geometry_msgs::PoseStamped> path_points;  // new
	std::mutex path_mtx;  //  new
	size_t current_target_idx = 0;  // new
	
	void pathCallback(const nav_msgs::Path::ConstPtr& msg);  //new
	void processPath();  // new
    
    geometry_msgs::Twist cmd_vel_msg;
    point odom;
    std::queue<std::tuple<double, double, double>> navigation_queue; // 导航点队列
    bool navigating;
    bool x_forward(double x);
    bool y_forward(double y);
    bool rotation(double yaw);

    bool hasReachedTarget(double target_x, double target_y, double tolerance);
    bool pause_nav;
    bool cancel_nav;
    bool allow_forward; 
    bool allow_backward; 

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
    Eigen::Matrix4f transformation;
    double last_goalx;
    double last_goaly;
	//eso
	double z_n1;    // ESO状态观测值
	double z_n2;    // ESO扰动估计值
	double eso_p;   // 观测器带宽参数
	double eso_b0;  // 控制增益估计值
	double T;       // 采样时间间隔

};

void NavNode::start_to_pause()
{
    pause_nav = true;
}
void NavNode::pause_to_start()
{
    pause_nav = false;
}


NavNode::NavNode() : cmd_vel_pub(nh.advertise<geometry_msgs::Twist>("/cmd_vel_a", 10)), navigating(false)
{

	// ESO参数初始化
	z_n1 = 0.0;
	z_n2 = 0.0;
	eso_p = 0.5;    // 需要根据系统特性调整
	eso_b0 = 0.8;   // 需要根据实际控制增益调整
	T = 0.1;        // 对应10Hz控制频率

    // odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom_centre", 1000, &NavNode::odomCallback, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/robot/dlo/odom_node/odom", 1, &NavNode::odomCallback, this);
    //odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1000, &NavNode::odomCallback, this);
	path_sub = nh.subscribe<nav_msgs::Path>("/move_base_node/NavfnROS/plan", 1, &NavNode::pathCallback, this);//new
    laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &NavNode::laserCloudHandler, this);
    odom = {0};
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.linear.y = 0;
    cmd_vel_msg.linear.z = 0;
    cmd_vel_msg.angular.x = 0;
    cmd_vel_msg.angular.y = 0;
    cmd_vel_msg.angular.z = 0;
    pause_nav = false;
    cancel_nav = false;

    // 初始化其他ROS相关的操作
    transformation.setIdentity();
    laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());

    double last_goalx = 0;
    double last_goaly = 0;
    allow_forward = true;  
    allow_backward = true; 
}

void NavNode::laserCloudHandler(const sensor_msgs::LaserScan::ConstPtr& pc)
{
    sensor_msgs::PointCloud2 cloud;
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*pc, cloud); 
    pcl::fromROSMsg(cloud, *laserCloudIn);
    // float f_dangerouspoint = 0;
    // float b_dangerouspoint = 0;

    float min_front_distance = std::numeric_limits<float>::max(); // 初始化最近前方障碍物距离为最大值
    int front_obstacle_count = 0; // 前方障碍物点的数量

    // 遍历点云数据，检测前方障碍物
    for (const auto& point : laserCloudIn->points) {
        // 计算当前点到雷达的水平距离（忽略 z 坐标）
        float distance = std::sqrt(point.x * point.x + point.y * point.y);

        // 判断当前点是否位于前方障碍物范围内（x 范围 [0, 1] 米，y 范围 [-0.5, 0.5] 米）
        if (point.x >= 0 && point.x <= 0.3 && point.y >= -0.2 && point.y <= 0.2) {
            // 更新最小前方障碍物距离
            min_front_distance = std::min(min_front_distance, distance);
            // 统计前方障碍物的点的数量
            front_obstacle_count++;
        }
    }
  
   //    std::cout << "前方最近障碍物距离：" << min_front_distance << " 米" << std::endl;
   // std::cout << "前方障碍物点的数量：" << front_obstacle_count << std::endl;
    // 控制逻辑：根据障碍物点的数量来决定是否允许前进
    if (front_obstacle_count > 20) {
        allow_forward = false;
        std::cout << "stop forward" << std::endl;
    } else {
        allow_forward = true;
    }
        // if(laserCloudIn->points[i].x<0.3 && laserCloudIn->points[i].x>0 &&
        // laserCloudIn->points[i].y<0.5 && laserCloudIn->points[i].y>-0.5){
        //     f_dangerouspoint++;
        //     continue;
        // }

        // if(laserCloudIn->points[i].x < 0 && laserCloudIn->points[i].x > -0.3 &&
        // laserCloudIn->points[i].y<0.5 && laserCloudIn->points[i].y>-0.5){
        //     b_dangerouspoint++;
        //     continue;
        // }   
    // if(f_dangerouspoint > 20) {
    //     allow_forward=false;
    //     std::cout << "stop forward" << std::endl;
    // }
    // else {
    //     allow_forward = true;
    // }
    // if(b_dangerouspoint > 20) {
    //     allow_backward = false;
    //     std::cout << "stop backward" << std::endl;
    // }
    // else {
    //     allow_backward =true;
    // }
}

void NavNode::navigateTo(double x, double y, double yaw)
{
    addNavigationTarget(x, y, yaw);
    if (!navigating)
    {
        navigating = true;
        performNavigation();
    }
}

void NavNode::mpc_control(double x2, double y2, double yaw2)
{
	// 新增ESO更新逻辑
	// 获取当前系统输出η(k)（这里使用位置误差作为观测）
	double eta_k = sqrt(pow(odom.x - x2, 2) + pow(odom.y - y2, 2));

	// ESO状态更新（式4-5）
	double z_n1_prev = z_n1;
	double z_n2_prev = z_n2;

	z_n1 = z_n1_prev + T * (z_n2_prev - eso_p*(z_n1_prev - eta_k) + eso_b0 * cmd_vel_msg.linear.x);
	z_n2 = z_n2_prev - T * pow(eso_p, 2)*(z_n1_prev - eta_k);
    cout<<"z_n2: " << z_n2 <<endl;

	// 控制量计算（式4-6）
	double eta_star = 0.0;  // 目标误差（需要根据实际情况定义）
	double eta_control = 1.0*(eta_star - z_n1);  // 比例系数k=1.0
	double tau_k = eta_control - (1 / eso_b0)*z_n2;
    cout<<"tau_k: " << tau_k <<endl;

	// 原有MPC控制逻辑修改
	// 在状态向量中加入扰动估计
    std::cout << "mpc" << std::endl;
    double last_v = 0;
    double last_w = 0;
    double last_yaw = 0;
    double a,b;
    bool y_axis = false;
 
    if(last_goalx - x2 != 0)
    {
        b = (last_goalx * y2 - x2 * last_goaly) / (last_goalx - x2); // y=ax+b   这是b
        a = (last_goaly - y2) / (last_goalx - x2);             // 这是a
    }
    else{
        y_axis = true;
        a=0;
        b=0;
    }
    ros::Rate mpc_rate(10);
    float s_distance = (odom.x - x2)*(odom.x - x2) + (odom.y - y2)*(odom.y - y2);
    //origincout
    //std::cout << "s_distance" << s_distance << std::endl;
    while (abs(y2 - odom.y) > 0.03 || abs(x2 - odom.x) > 0.03)
    {

        float distance = (odom.x - x2)*(odom.x - x2) + (odom.y - y2)*(odom.y - y2);
	 //origincout
    //std::cout << "distance" << distance << std::endl;
     
        double x,y;
        if(y_axis)
        {
            x = x2;
            if(y2 > odom.y)
            {
                y = odom.y+0.5;

            }
            else{
                y = odom.y-0.5;
            }
        }
        else{
            if(x2 > odom.x)
            {
                x = std::min(odom.x+0.5,x2);
                y = a*x+b;
            }
            else{
                x = std::max(odom.x-0.5,x2);
                y = a*x+b;
            }
        }


	    if (abs(y2 - odom.y) < 0.5 && abs(x2 - odom.x) < 0.5)
        {
            x=x2;
            y=y2;
        }
        double x2_car = cos(odom.yaw) * (x - odom.x) + sin(odom.yaw) * (y - odom.y);
        double y2_car = -sin(odom.yaw) * (x - odom.x) + cos(odom.yaw) * (y - odom.y);

        double coeffs = (0 * y2_car - x2_car * 0) / (0 - x2_car); // y=ax+b   这是b
        double coeffs1 = (0 - y2_car) / (0 - x2_car);             // 这是a
        double coeffs2 = y2_car/(x2_car*x2_car);
        //cout<<"a: " << a <<"b: " << b << "coeffs: "<<coeffs<<"coeffs1: "<<coeffs1<<endl;
        //cout<<"path: " << x <<"|"<<y<<endl;
        //cout<<"a: " << a <<"b: "<<b<<endl;

        // vector<double> v_w;

        Eigen::VectorXd state(8);
        state[0] = last_v * CppAD::cos(last_yaw) * 0.1;
        state[1] = 0;
        state[2] = last_w * 0.1;
        state[3] = last_v;
        state[4] = coeffs + coeffs1 * 0 + last_v * sin(last_yaw) * 0.1;
        state[5] = CppAD::atan(coeffs1) - last_yaw;
        state[6] = last_w;
		state[7] = z_n2;  // 加入扰动估计

        Eigen::VectorXd coe(4);
        coe[0] = coeffs;
        coe[1] = coeffs1;//0
        coe[2] = 0;//coeffs2
        coe[3] = 0;
        
        // 新增一个变量来存储上一次发布的实际速度
        static double last_actual_v = 0.0;
        double ref_v = 0;
        double ratio = 0;
        static double smoothed_v = 0;  // 平滑后的速度
        const double alpha = 0.15;  // 平滑系数
        const double max_speed_change = 0.05;  // 最大速度变化值
        if (x2_car > 0)
        {
            ratio = distance / s_distance;
            if (ratio > 1){
		    ratio = 1;
	        }
	        if (ratio < 0){
		    ratio = 0;
	        }
    
     //origincout
	  //  cout<<"ratio: " << ratio <<endl;
         // 定义不同阶段的速度
         double high_speed = 0.1;
         double medium_speed = 0.1;
         double low_speed = 0.05;
         double min_speed = 0.05;

    if (ratio >= 0.9)
    {
        ref_v = high_speed;
    }
    else if (ratio >= 0.7)
    {
        ref_v = high_speed;
    }
    else if (ratio >= 0.1)
    {
        ref_v = high_speed;
    }
    else if (ratio >= 0.003)
    {
        ref_v = low_speed;
    }
        else if (ratio >= 0.002)
    {
        ref_v = low_speed;
    }
    else
    {
        ref_v = low_speed;
    }

    // 指数平滑
    double smoothed_v = alpha * ref_v + (1 - alpha) * smoothed_v;

    // // 限制速度变化
    // if (temp_smoothed_v - smoothed_v > max_speed_change)
    // {
    //     smoothed_v = smoothed_v + max_speed_change;
    // }
    // else if (smoothed_v - temp_smoothed_v > max_speed_change)
    // {
    //     smoothed_v = smoothed_v - max_speed_change;
    // }
    // else
    // {
    //     smoothed_v = temp_smoothed_v;
    // }

    ref_v = smoothed_v;
    
            //ref_v = 0.25 * distance / s_distance;
            //ref_v = ref_v < 0.2 ? 0.2 : ref_v;
            //ref_v = 0.1;
            if (abs(y2 - odom.y) < 0.1 && abs(x2 - odom.x) < 0.1)
            {
                ref_v = 0.05;
            } 
        }
        else
        {
            ref_v = -0.25 * distance / s_distance;
            ref_v = ref_v > -0.1 ? -0.1 : ref_v;
            if (abs(y2 - odom.y) < 0.1 && abs(x2 - odom.x) < 0.1)
            {
                ref_v = -0.01;
            } 

        }

        if(!allow_forward && ref_v > 0){
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = 0;
            cmd_vel_pub.publish(cmd_vel_msg);
            continue;
        }
        if(!allow_backward && ref_v < 0){
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = 0;
            cmd_vel_pub.publish(cmd_vel_msg);
            continue;
        }
        if(pause_nav)
        {
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = 0;
            cmd_vel_pub.publish(cmd_vel_msg);
            continue;

        }
			

		// 在模型约束中考虑扰动：
		// for (int t = 1; t < N; t++) {
		// 	// 原状态变量
		// 	AD<double> x0 = vars[x_start + t - 1];
		// 	AD<double> y0 = vars[y_start + t - 1];
		// 	AD<double> psi0 = vars[psi_start + t - 1];
		// 	AD<double> v0 = vars[v_start + t - 1];

			// 从ESO获取扰动估计（前馈补偿项）
			// AD<double> disturbance = z_n2; // 使用正确的成员变量 // z_n2为ESO输出的扰动估计值

		// 	// 修正后的状态方程（加入前馈扰动）
		// 	x1 = x0 + (v0 * CppAD::cos(psi0) + disturbance) * dt;
		// 	y1 = y0 + (v0 * CppAD::sin(psi0)) * dt;
		// 	psi1 = psi0 + (v0 * delta0 / Lf) * dt;
		// }

		// // new eso在MPC代价函数中加入扰动补偿项
		// 		for (int t = 0; t < N; t++) {
		// 			// 原有状态代价
		// 			fg[0] += CppAD::pow(vars[delta_start + t], 2);
		// 			fg[0] += CppAD::pow(vars[a_start + t], 2);
		// 		}

		// eso修改控制量计算（补偿扰动）
		vector<double> v_w = mpc_tool.Solve(state, coe, ref_v);
		double current_actual_v = v_w[1];
		double compensated_control = current_actual_v - tau_k;  // 扰动补偿
        // 添加速度限幅
        compensated_control = std::clamp(compensated_control, -0.3, 0.3);
      //  cout<<"compensated_control: " << compensated_control <<endl;
        
        // 减速阶段的判断条件，这里简单假设 ratio < 0.9 为减速阶段
        if (ratio < 0.007 && current_actual_v > last_actual_v) {
            current_actual_v = last_actual_v;
         v_w[1] = current_actual_v;
        }

        //eso
		cmd_vel_msg.linear.x = compensated_control;
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.linear.z = 0;
        cmd_vel_msg.angular.x = 0;
        cmd_vel_msg.angular.y = 0;
        cmd_vel_msg.angular.z = v_w[0];
        cmd_vel_pub.publish(cmd_vel_msg);

       //origincout
       //std::cout << "v: " << v_w[1] << "w: " << v_w[0] <<"ref_v: " << ref_v << std::endl; 
        //std::cout << "odom.x: " << odom.x << "odom.y: " << odom.y << "odom.yaw" << odom.yaw << std::endl;
        last_w = cmd_vel_msg.angular.z;
        last_v = cmd_vel_msg.linear.x;
        last_yaw = odom.yaw;

        // 更新上一次发布的实际速度
        last_actual_v = current_actual_v;
        mpc_rate.sleep();
    }

    cmd_vel_msg.linear.y = 0;
    cmd_vel_msg.linear.z = 0;
    cmd_vel_msg.angular.x = 0;
    cmd_vel_msg.angular.y = 0;
    cmd_vel_msg.angular.z = 0;
    cmd_vel_msg.linear.x = 0;
    cmd_vel_pub.publish(cmd_vel_msg);
    std::cout << "mpc success" << std::endl;
}

void NavNode::addNavigationTarget(double x, double y, double yaw)
{
    std::cout << "addNavigationTarget" << std::endl;
    navigation_queue.push(std::make_tuple(x, y, yaw));
}

void NavNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double roll, pitch, yaw;

    if (!msg)
    {
        ROS_ERROR("odom信息为空,节点退出.");
        ros::shutdown(); // 关闭ROS节点
        return;
    }

    odom.x = msg->pose.pose.position.x;
    odom.y = msg->pose.pose.position.y;
    Eigen::VectorXf vectorodom(4);
    Eigen::VectorXf vector_odomupdate(4);
    vectorodom << odom.x, odom.y, 0, 1;
    vector_odomupdate = transformation * vectorodom;
    odom.x = vector_odomupdate[0];
    odom.y = vector_odomupdate[1];
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    if (yaw<0) {
        yaw+=2*pi;
    }
    odom.yaw = yaw;
}

//new
void NavNode::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO("Received path message with %d points!", (int)msg->poses.size()); //
	std::lock_guard<std::mutex> lock(o_mtx);
	path_points = msg->poses;
	current_target_idx = 0;
    // ROS_INFO("[PathCallback] Reset current_target_idx to 0.");
	processPath();  
}


// new
void NavNode::processPath()
{
	std::lock_guard<std::mutex> lock(path_mtx);
    navigation_queue = std::queue<std::tuple<double, double, double>>(); 
	
    if (path_points.size() < 2) {
		ROS_WARN("Received path with %zu points, need at least 2", path_points.size());
		return;
	}

	std::vector<geometry_msgs::PoseStamped> sampled_points;
	sampled_points.push_back(path_points[0]);  
	const auto& final_point = path_points.back();  //

	for (size_t i = 1; i < path_points.size(); ++i) {
		double dx = path_points[i].pose.position.x - sampled_points.back().pose.position.x;
		double dy = path_points[i].pose.position.y - sampled_points.back().pose.position.y;
		double dist = std::hypot(dx, dy);
		
		if (dist >= 0.1) {
			sampled_points.push_back(path_points[i]);
		}
	}

	if (!sampled_points.empty() &&
		sampled_points.back().pose.position.x != final_point.pose.position.x ||
		sampled_points.back().pose.position.y != final_point.pose.position.y) {
		sampled_points.push_back(final_point);
       // ROS_INFO("[ProcessPath] Added final point to sampled path."); 
	}

	path_points.swap(sampled_points);
	navigation_queue = std::queue<std::tuple<double, double, double>>();  //

	//
	for (const auto& pose : path_points) {
		double yaw = tf::getYaw(pose.pose.orientation);
		navigation_queue.push(std::make_tuple(
			pose.pose.position.x,
			pose.pose.position.y,
			yaw
		));
	}
     ROS_INFO("[ProcessPath] New queue filled with %zu points.", navigation_queue.size());
}

bool NavNode::x_forward(double target_x)
{
    while (abs(target_x - odom.x) > 0.02)
    {
        if (target_x - odom.x < 0)
        {
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = -0.05;
        }
        else
        {
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = 0.05;
        }
        if (pause_nav)
        {
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = 0;
        }
        cmd_vel_pub.publish(cmd_vel_msg);
    }

    cmd_vel_msg.linear.y = 0;
    cmd_vel_msg.linear.z = 0;
    cmd_vel_msg.angular.x = 0;
    cmd_vel_msg.angular.y = 0;
    cmd_vel_msg.angular.z = 0;
    cmd_vel_msg.linear.x = 0;
    cmd_vel_pub.publish(cmd_vel_msg);
    return true;
}
bool NavNode::y_forward(double target_y)
{

    while (abs(target_y - odom.y) > 0.02)
    {
        if (target_y - odom.y < 0)
        {
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = 0.05;
        }
        else
        {
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = -0.05;
        }
        if (pause_nav)
        {
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = 0;
        }
        cmd_vel_pub.publish(cmd_vel_msg);
        // std::cout << "odom.y" << odom.y  <<"target_y" <<target_y<<std::endl;
    }
    cmd_vel_msg.linear.y = 0;
    cmd_vel_msg.linear.z = 0;
    cmd_vel_msg.angular.x = 0;
    cmd_vel_msg.angular.y = 0;
    cmd_vel_msg.angular.z = 0;
    cmd_vel_msg.linear.x = 0;
    cmd_vel_pub.publish(cmd_vel_msg);

    return true;
}


//new
bool NavNode::rotation(double final_yaw)
{
	double current_yaw = odom.yaw;
	double yaw_diff = final_yaw - current_yaw;

	// 考虑到角度周期性，将角度控制在[-π, π]范围内
	while (yaw_diff > M_PI)
	{
		yaw_diff -= 2 * M_PI;
	}
	while (yaw_diff < -M_PI)
	{
		yaw_diff += 2 * M_PI;
	}

	double angular_speed = 0.2;
	double tolerance = 0.05; 

	// 开始旋转，直到达到目标航向角
	while (std::abs(yaw_diff) > tolerance)
	{
		if (yaw_diff > 0)
		{
			cmd_vel_msg.angular.z = angular_speed;
		}
		else
		{
			cmd_vel_msg.angular.z = -angular_speed;
		}

		cmd_vel_pub.publish(cmd_vel_msg);

		current_yaw = odom.yaw;
		yaw_diff = final_yaw - current_yaw;

		// 考虑到角度周期性，将角度控制在[-π, π]范围内
		while (yaw_diff > M_PI)
		{
			yaw_diff -= 2 * M_PI;
		}
		while (yaw_diff < -M_PI)
		{
			yaw_diff += 2 * M_PI;
		}

		ros::Duration(0.1).sleep();
	}

	// 停止旋转
	cmd_vel_msg.angular.z = 0;
	cmd_vel_pub.publish(cmd_vel_msg);

	return true;

}



void NavNode::performNavigation()
{
    while (ros::ok())
    {
        if (navigation_queue.empty())
        {
            // new
			if (!path_points.empty() && current_target_idx < path_points.size()) {
				auto& target = path_points[current_target_idx];
                std::cout<<"path" <<path_points[current_target_idx]<< std::endl;//
				navigation_queue.push(std::make_tuple(
					target.pose.position.x,
					target.pose.position.y,
					tf::getYaw(target.pose.orientation)
				));
				current_target_idx++;
			}else{
                // 如果队列为空，停止导航
            navigating = false;
            // 发送停止命令
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = 0;
            cmd_vel_pub.publish(cmd_vel_msg);
            sleep(0.01);
            continue;
            }
        }

        // 从队列中取出下一个导航点
        auto target = navigation_queue.front();
        double target_x = std::get<0>(target);
        double target_y = std::get<1>(target);
        double target_yaw = std::get<2>(target);

        if (hasReachedTarget(target_x, target_y, 0.05)) 
        {
            ROS_INFO("Reached target. Pausing for 1 second...");
            navigation_queue.pop();

            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel_msg);
            ros::Duration(1.0).sleep(); // 暂停1秒
            continue; // 跳过当前目标点

        }
        
        if(target_yaw<0) target_yaw+=2*pi;
        double current_yaw = odom.yaw;

        double  go_yaw = std::atan2(target_y-odom.y,target_x-odom.x);
        if(go_yaw<0){
            go_yaw+=2*pi;
        }

        //origincout
       // std::cout<<"go_yaw: "<<go_yaw<<" current_yaw: "<<current_yaw<<std::endl;
      //  std::cout<<"abs(go_yaw - current_yaw): "<<abs(go_yaw - current_yaw)<<std::endl;
       // std::cout<<"2*pi - max(go_yaw,current_yaw) + min(go_yaw,current_yaw) "<<2*pi - max(go_yaw,current_yaw) + min(go_yaw,current_yaw)<<std::endl;
        // 3.48 2.79

        sleep(1);
        if( (abs(go_yaw - current_yaw) < 200.0/180.0*pi && abs(go_yaw - current_yaw) > 160.0/180.0*pi) 
            || (abs(2*pi - max(go_yaw,current_yaw) + min(go_yaw,current_yaw)) < 200.0/180.0*pi && abs(2*pi - max(go_yaw,current_yaw) + min(go_yaw,current_yaw)) > 160.0/180.0*pi) ){
            //无需旋转 后退

            mpc_control(target_x, target_y, target_yaw);
            sleep(0.2);
            //rotation(final_yaw);
            //sleep(0.2);
        }
        else{
            mpc_control(target_x, target_y, target_yaw);
            sleep(0.2);
        }

        navigation_queue.pop();
        last_goalx = target_x;
        last_goaly = target_y;
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.linear.z = 0;
        cmd_vel_msg.angular.x = 0;
        cmd_vel_msg.angular.y = 0;
        cmd_vel_msg.angular.z = 0;
        cmd_vel_msg.linear.x = 0;
        cmd_vel_pub.publish(cmd_vel_msg);
        ROS_INFO("The current target point navigation has concluded.");
        sleep(0.5);
    }
}
// bool NavNode::hasReachedTarget(double target_x, double target_y, double target_yaw, double tolerance = 0.05)
    bool NavNode::hasReachedTarget(double target_x, double target_y, double tolerance = 0.05)
{
    // 计算距离
    double distance = sqrt(pow(target_x - odom.x, 2) + pow(target_y - odom.y, 2));

    // 计算方向差异，可以考虑使用角度差异或者四元数来比较
   // double yaw_diff = target_yaw - odom.yaw;

    // // 考虑到角度周期性，将角度控制在[-π, π]范围内
    // while (yaw_diff > M_PI)
    // {
    //     yaw_diff -= 2 * M_PI;
    // }
    // while (yaw_diff < -M_PI)
    // {
    //     yaw_diff += 2 * M_PI;
    // }

    // 检查距离和方向是否都在容忍范围内
    //if (distance < tolerance && fabs(yaw_diff) < tolerance)
    if (distance < tolerance < tolerance)
    {
        return true; // 到达目标点
        // ROS_INFO("!!!!Reached target!!!!");
    }
    else
    {
        return false; // 未到达目标点
    }
}

#endif
