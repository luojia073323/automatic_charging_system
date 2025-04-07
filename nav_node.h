#pragma GCC optimize(3,"Ofast","inline")
#ifndef NAV_NODE_H
#define NAV_NODE_H

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
#include <mutex>
#include <nav_msgs/Path.h> //new 新增路径消息支持


struct Point {
    double x, y;
	bool operator<(const Point& p) const {
		return x < p.x || (x == p.x && y < p.y);
	}
};


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

private:
    ros::Publisher cmd_vel_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber laser_sub;
    ros::Publisher pub_local_path_;
    ros::Publisher pub_need_path_;
    
	ros::Subscriber path_sub;  // new新增路径订阅
	std::vector<geometry_msgs::PoseStamped> path_points;  // new存储路径点
	std::mutex path_mtx;  //  new专用路径锁
	size_t current_target_idx = 0;  // new当前跟踪路径点索引
	
	void pathCallback(const nav_msgs::Path::ConstPtr& msg);  //new 新增路径回调
	void processPath();  // new路径处理函数
	geometry_msgs::Twist cmd_vel_msg;
    point odom;
   
    float min_distance = 9999999;
    std::queue<std::tuple<double, double, double>> navigation_queue; //  导航点队列
    bool navigating;
    bool x_forward(double x);
    bool y_forward(double y);
    bool rotation(double yaw);
    bool rotation(double goal_x,double goal_y);
    bool rotation_cbf(double goal_x,double goal_y);
    bool hasReachedTarget(double target_x, double target_y, double target_yaw, double tolerance);
    bool pause_nav;
    bool cancel_nav;
    bool allow_forward; 
    bool allow_backward; 

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn;
    Eigen::Matrix4f transformation;

    double last_goalx = 0;
    double last_goaly = 0;

};

void NavNode::start_to_pause()
{
    pause_nav = true;
}
void NavNode::pause_to_start()
{
    pause_nav = false;
}


NavNode::NavNode() : cmd_vel_pub(nh.advertise<geometry_msgs::Twist>("cmd_vel_a", 1)), navigating(false)
{

    // odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom_centre", 1000, &NavNode::odomCallback, this);
    //odom_sub = nh.subscribe<nav_msgs::Odometry>("/featurematching/odom", 1, &NavNode::odomCallback, this);
	path_sub = nh.subscribe<nav_msgs::Path>("move_base_node/NavfnROS/plan", 1, &NavNode::pathCallback, this);//new
    odom_sub = nh.subscribe<nav_msgs::Odometry>("dlo/odom_node/odom", 1, &NavNode::odomCallback, this);
    laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/tb3_0/scan", 1, &NavNode::laserCloudHandler, this);
    pub_local_path_ = nh.advertise<nav_msgs::Path>("/local_path", 10);
    pub_need_path_ = nh.advertise<nav_msgs::Path>("/need_path", 10);
  
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
	allow_forward = true;  // new始终保持允许前进
	allow_backward = true; // new始终保持允许后退
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
    o_mtx.lock();
    std::cout << "mpc" << std::endl;
    double last_v = 0;
    double last_w = 0;
    double last_yaw = 0;
    double a,b;
    bool y_axis = false;
 
    if(last_goalx - x2 != 0)
    {
        a = (last_goaly - y2) / (last_goalx - x2);             
        b = (last_goalx * y2 - x2 * last_goaly) / (last_goalx - x2); // y=ax+b  
        
    }
    else{
        y_axis = true;
        a=0;
        b=0;
    }

    float s_distance = (odom.x - x2)*(odom.x - x2) + (odom.y - y2)*(odom.y - y2);

    o_mtx.unlock();


    while (abs(y2 - odom.y) > 0.05 || abs(x2 - odom.x) > 0.05)
    {
        o_mtx.lock();
        float distance = (odom.x - x2)*(odom.x - x2) + (odom.y - y2)*(odom.y - y2);
        double x,y;
        if(y_axis)
        {
            x = x2;
            if(y2 > odom.y)
            {
                y = odom.y+1.5;

            }
            else{
                //y = odom.y-1.5;
                y = y2;
            }
        }
        else{
            if(x2 > odom.x)
            {
                x = std::min(odom.x+1.5,x2);
                y = a*x+b;
            }
            else{
                x = std::max(odom.x-1.5,x2);
                y = a*x+b;
            }
        }

	    if (abs(y2 - odom.y) < 0.5 && abs(x2 - odom.x) < 0.5)
        {
            x=x2;
            y=y2;
        }
        double x2_car = cos(odom.yaw) * (x2 - odom.x) + sin(odom.yaw) * (y2 - odom.y);
        double y2_car = -sin(odom.yaw) * (x2- odom.x) + cos(odom.yaw) * (y2 - odom.y);

        double coeffs = (0 * y2_car - x2_car * 0) / (0 - x2_car); // y=ax+b   
        double coeffs1 = (0 - y2_car) / (0 - x2_car);            
        double coeffs2 = 0;

        //cout<<"path: " << x <<"|"<<y<<endl;
        //std::cout << "odom.x 1: " << odom.x << "odom.y 1: " << odom.y << "odom.yaw 1 " << odom.yaw << std::endl;
		std::vector<double> v_w;
		Eigen::VectorXd state = Eigen::VectorXd::Zero(7);
		state[3] = last_v;
		state[6] = last_w;

        Eigen::VectorXd coe(4);
        coe[0] = coeffs;
        coe[1] = coeffs1;
        coe[2] = 0;
        coe[3] = 0;
        double ref_v = 0;
        double ratio = 0;
        if (x2 > odom.x)
        {
            ratio = distance / s_distance;
            if (ratio > 1){
		    ratio = 1;
	        }
	        if (ratio < 0){
		    ratio = 0;
	        }
	        //cout<<"ratio: " << ratio <<endl;
            if (ratio >= 0.9) {
            ref_v = 0.1;
            }else if (a >= 0.1) {
            ref_v = 0.4;
            }else {
            ref_v = 0.2; 
            }
            //ref_v = 0.25 * distance / s_distance;
            ref_v = ref_v < 0.2 ? 0.2 : ref_v;
            if (abs(y2 - odom.y) < 0.05 && abs(x2 - odom.x) < 0.05)
            {
                ref_v = 0.01;
            } 
        }
        else
        {
            ref_v = -0.25 * distance / s_distance;
            ref_v = ref_v > -0.1 ? -0.1 : ref_v;
            if (abs(y2 - odom.y) < 0.05 && abs(x2 - odom.x) < 0.05)
            {
                ref_v = -0.01;
            } 

        }
        mtx.lock();
        
     
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

			double t_yaw;
			mpc_tool.c_yaw = odom.yaw;
			mpc_tool.mpc_cbf = false;
			mpc_tool.ob_size = 0;
			mpc_tool.mpc_v.clear();
			v_w = mpc_tool.Solve(state, coe, ref_v);

        }

        cmd_vel_msg.linear.x = v_w[1];
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.linear.z = 0;
        cmd_vel_msg.angular.x = 0;
        cmd_vel_msg.angular.y = 0;
        cmd_vel_msg.angular.z = v_w[0] ;
        cmd_vel_pub.publish(cmd_vel_msg);

        std::cout << "v: " << v_w[1] << "w: " << v_w[0] <<"ref_v: " << ref_v << std::endl; 
        std::cout << "odom.x 2: " << odom.x << "odom.y 2: " << odom.y << "odom.yaw 2 " << odom.yaw << std::endl;
       
        last_w = cmd_vel_msg.angular.z;
        last_v = cmd_vel_msg.linear.x;
        last_yaw = odom.yaw;

       nav_msgs::Path predict_traj;
       geometry_msgs::PoseStamped pose_msg;
       predict_traj.header.frame_id    = "base_link";
        predict_traj.header.stamp       = ros::Time::now();
        pose_msg.pose.orientation.w     = 1.0;
        pose_msg.pose.orientation.x     = 0.0;
        pose_msg.pose.orientation.y     = 0.0;
        pose_msg.pose.orientation.z     = 0.0;
        for(int i=0; i<mpc_tool.x_pred_vals.size(); i++){
            pose_msg.pose.position.x    = mpc_tool.x_pred_vals[i];
            pose_msg.pose.position.y    = mpc_tool.y_pred_vals[i];
            predict_traj.poses.push_back(pose_msg);
        }
        pub_local_path_.publish(predict_traj);

        predict_traj.poses.clear();
        pose_msg.pose.position.x    = x2_car;
        pose_msg.pose.position.y    = y2_car;
        predict_traj.poses.push_back(pose_msg);
        pose_msg.pose.position.x    = 0;
        pose_msg.pose.position.y    = 0;
        predict_traj.poses.push_back(pose_msg);
        pub_need_path_.publish(predict_traj);

        o_mtx.unlock();
        
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
		ROS_FATAL("Invalid odometry message!");
		ros::shutdown();
		exit(EXIT_FAILURE);
    }

    odom.x = msg->pose.pose.position.x;
    odom.y = msg->pose.pose.position.y;
    Eigen::VectorXf vectorodom(4);
    Eigen::VectorXf vector_odomupdate(4);
    vectorodom << odom.x, odom.y, 0, 1;
    vector_odomupdate = transformation * vectorodom;
    
    //tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    //tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double q0 = msg->pose.pose.orientation.x;
	double q1 = msg->pose.pose.orientation.y;
	double q2 = msg->pose.pose.orientation.z;
	double q3 = msg->pose.pose.orientation.w;
	double t0 = -2.0 * (q1*q1 + q2*q2) + 1.0;
	double t1 = +2.0 * (q2*q3 + q0*q1);

	double pq = atan2(t1, t0); // state[2]; 
    if(pq<0){
        pq+=2*M_PI;
    }
    odom.yaw = pq;
}


//new新增路径回调函数
void NavNode::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
	std::lock_guard<std::mutex> lock(o_mtx);
	path_points = msg->poses;
	current_target_idx = 0;
	processPath();  // 处理路径采样
}


// new路径处理函数（0.2米间隔采样）
void NavNode::processPath()
{
	std::lock_guard<std::mutex> lock(path_mtx);

	if (path_points.size() < 2) {
		ROS_WARN("Received path with %zu points, need at least 2", path_points.size());
		return;
	}

	std::vector<geometry_msgs::PoseStamped> sampled_points;
	sampled_points.push_back(path_points[0]);  
	const auto& final_point = path_points.back();  //保存原始终点

	for (size_t i = 1; i < path_points.size(); ++i) {
		double dx = path_points[i].pose.position.x - sampled_points.back().pose.position.x;
		double dy = path_points[i].pose.position.y - sampled_points.back().pose.position.y;
		double dist = std::hypot(dx, dy);

		if (dist >= 0.2) {
			sampled_points.push_back(path_points[i]);
		}
	}

	if (!sampled_points.empty() &&
		sampled_points.back().pose.position.x != final_point.pose.position.x &&
		sampled_points.back().pose.position.y != final_point.pose.position.y) {
		sampled_points.push_back(final_point);
	}

	path_points.swap(sampled_points);
	navigation_queue = std::queue<std::tuple<double, double, double>>();  // 娓呯┖鏃ч槦鍒?

	// 填充导航队列
	for (const auto& pose : path_points) {
		double yaw = tf::getYaw(pose.pose.orientation);
		navigation_queue.push(std::make_tuple(
			pose.pose.position.x,
			pose.pose.position.y,
			yaw
		));
	}
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
            cmd_vel_msg.linear.x = -0.1;
        }
        else
        {
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = 0.1;
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
            cmd_vel_msg.linear.x = 0.1;
        }
        else
        {
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = -0.1;
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
bool NavNode::rotation(double target_yaw)
{
	const int MAX_RETRY = 200;  // 约2秒超时
	int retry_count = 0;

	while (retry_count++ < MAX_RETRY &&
		abs(target_yaw - odom.yaw) > 0.01 &&
		ros::ok()) {

		if (target_yaw < 0) { target_yaw += 6.2831852; }
		if (odom.yaw < 0) { odom.yaw += 6.2831852; }
		std::cout << "target_yaw : " << target_yaw << "odom.yaw" << odom.yaw << std::endl;
		while (abs(target_yaw - odom.yaw) > 0.01 && allow_forward)
		{
			if (target_yaw > odom.yaw) {
				if (target_yaw - odom.yaw < pi) {
					cmd_vel_msg.angular.z = 0.1;
				}
				else {
					cmd_vel_msg.angular.z = -0.1;
				}
			}
			else {
				if (odom.yaw - target_yaw < pi) {
					cmd_vel_msg.angular.z = -0.1;
				}
				else {
					cmd_vel_msg.angular.z = 0.1;
				}
			}
			cmd_vel_msg.linear.y = 0;
			cmd_vel_msg.linear.z = 0;
			cmd_vel_msg.angular.x = 0;
			cmd_vel_msg.angular.y = 0;
			cmd_vel_msg.linear.x = 0.05;
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
			//sleep(0.005);
		}
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = 0;
		cmd_vel_msg.linear.x = 0;
		cmd_vel_pub.publish(cmd_vel_msg);
	               }

	return retry_count < MAX_RETRY;
}



bool NavNode::rotation(double goal_x,double goal_y)
{
    double  target_yaw = std::atan2(goal_y-odom.y  ,goal_x-odom.x);
    if(target_yaw<0){
        target_yaw+=2*pi;
    } 
    while (abs(target_yaw - odom.yaw) > 0.02)
    {
        std::cout<<"target_yaw : "<<target_yaw<<"odom.yaw"<<odom.yaw<<std::endl;
        if(target_yaw > odom.yaw){
            if(target_yaw - odom.yaw < pi){
                cmd_vel_msg.angular.z = 0.1;
            }
            else{
                cmd_vel_msg.angular.z = -0.1;
            }
        }
        else{
            if(odom.yaw - target_yaw  < pi){
                cmd_vel_msg.angular.z = -0.1;
            }
            else{
                cmd_vel_msg.angular.z = 0.1;
            }
        }
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.linear.z = 0;
        cmd_vel_msg.angular.x = 0;
        cmd_vel_msg.angular.y = 0;
        cmd_vel_msg.linear.x = 0;
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
        //sleep(0.005);
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


void NavNode::performNavigation(){

    while (ros::ok())
    {
		std::unique_lock<std::mutex> lock(path_mtx);

        if (navigation_queue.empty())
        {
			// new添加路径点动态更新逻辑
			if (!path_points.empty() && current_target_idx < path_points.size()) {
				auto& target = path_points[current_target_idx];
				navigation_queue.push(std::make_tuple(
					target.pose.position.x,
					target.pose.position.y,
					tf::getYaw(target.pose.orientation)
				));
				current_target_idx++;
			}

            // 如果队列为空，停止导航
            navigating = false;
            //  发送停止命令
            cmd_vel_msg.linear.y = 0;
            cmd_vel_msg.linear.z = 0;
            cmd_vel_msg.angular.x = 0;
            cmd_vel_msg.angular.y = 0;
            cmd_vel_msg.angular.z = 0;
            cmd_vel_msg.linear.x = 0;
            cmd_vel_pub.publish(cmd_vel_msg);
            //sleep(0.01);
            continue;
        }

		lock.unlock();

        // 从队列中取出下一个导航点
        auto target = navigation_queue.front();
        double target_x = std::get<0>(target);
        double target_y = std::get<1>(target);
        double target_yaw = std::get<2>(target);
        if(target_yaw<0) target_yaw+=2*pi;
        double current_yaw = odom.yaw;

        double  go_yaw = std::atan2(target_y-odom.y,target_x-odom.x);
        if(go_yaw<0){
            go_yaw+=2*pi;
        }
        std::cout<<"go_yaw: "<<go_yaw<<" current_yaw: "<<current_yaw<<std::endl;
        std::cout<<"abs(go_yaw - current_yaw): "<<abs(go_yaw - current_yaw)<<std::endl;
        std::cout<<"2*pi - max(go_yaw,current_yaw) + min(go_yaw,current_yaw) "<<2*pi - max(go_yaw,current_yaw) + min(go_yaw,current_yaw)<<std::endl;
        // 3.48 2.79
        //sleep(1);
        if( (abs(go_yaw - current_yaw) < 200.0/180.0*pi && abs(go_yaw - current_yaw) > 160.0/180.0*pi) 
            || (abs(2*pi - max(go_yaw,current_yaw) + min(go_yaw,current_yaw)) < 200.0/180.0*pi && abs(2*pi - max(go_yaw,current_yaw) + min(go_yaw,current_yaw)) > 160.0/180.0*pi) ){
            //无需旋转 后退

            mpc_control(target_x, target_y, target_yaw);
            //sleep(0.5);
            rotation(target_yaw);
            //sleep(3);
        }
        else{
            rotation(target_x,target_y);
            mpc_control(target_x, target_y, target_yaw);
            //sleep(0.5);
            rotation(target_yaw);
            //sleep(3);
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
        //sleep(0.5);
		ros::spinOnce();
		usleep(10000);  //适当延时
    }
}


bool NavNode::hasReachedTarget(double target_x, double target_y, double target_yaw, double tolerance = 0.5)
{
    // 计算距离
    double distance = sqrt(pow(target_x - odom.x, 2) + pow(target_y - odom.y, 2));

    //计算方向差异，可以考虑使用角度差异或者四元数来比较
    double yaw_diff = target_yaw - odom.yaw;

    //考虑到角度周期性，将角度控制在[-π, π]范围内
    while (yaw_diff > M_PI)
    {
        yaw_diff -= 2 * M_PI;
    }
    while (yaw_diff < -M_PI)
    {
        yaw_diff += 2 * M_PI;
    }

    // 检查距离和方向是否都在容忍范围内
    if (distance < tolerance && fabs(yaw_diff) < tolerance)
    {
        return true; // 到达目标点
    }
    else
    {
        return false; // 未到达目标点
    }
}


#endif
