#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h> //有向量的消息类型
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>



#include <nav_msgs/Path.h>

#include <std_msgs/String.h>
#include <eigen3/Eigen/Geometry> 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <sstream>

#include <thread>        
#include <mutex>   

using namespace std;
using namespace Eigen;
using namespace message_filters;



#define pi 3.1415927 
double distance0=0.0,distance1=0.0,distance2=0.0,distance3=0.0,distance4=0.0,distance_sum=0.0,Timing_ms=0.0;

Eigen::Matrix<double,5,5> L;
Eigen::Matrix<double, 2, 5> sum,u_sum,sum_abs,sum_abs_abs,distance_robots,position_desire,p;//距离

double Timing2 = 0.0;
double Timing = 0.0;
int time_flag = 1;

float c_0,c_1 ,c_2,c_3,c_4,c_5;
float v_3,v_4,v_5;


double epsilon_0 = 0.1; //初始角度偏移量
float sigma = 0.05;
float lambda = 0.5;
float rs = 0.08*2,rc = 5.0;
float ks = 1;
double beta_l3 ; //最小角度
double beta_r3 ; //最大角度s
double beta_l4 ; //最小角度
double beta_r4 ; //最大角度
double beta_l5 ; //最小角度
double beta_r5 ; //最大角度


Eigen::Matrix<double, 5, 7> H;//邻接矩阵

Eigen::Matrix<double, 5, 5> edge;//边

Eigen::Matrix<double, 5, 5> all_edge;//全部边
Eigen::Matrix<double, 5, 5> all_desire_edge;//全部边
Eigen::Matrix<double, 5, 5> s;//加权边（转换）
Eigen::Matrix<double, 5, 5> desire_s;//期望对加权边
Eigen::Matrix<double, 5, 5> eij;//边误差计算
Eigen::Matrix<double, 5, 5> zij;//边误差计算
Eigen::Matrix<double, 3, 1> zk;//角度误差计算


//barrier 
unsigned char barrier_number = 3;
MatrixXd barrierpoint = MatrixXd::Constant(2,barrier_number,0);

Eigen::Matrix<double, 2, 1> APF(Eigen::Matrix<double, 2, 1> robotpoint,Eigen::Matrix<double, 2, 1>  barrierpoint);


// 坐基于点的控制
void StampedTransformToTwist_Angle_Distance(tf::Transform &transform_robot,Eigen::Matrix<double, 2, 1> u,geometry_msgs::Twist &vel_msg)
{
	double distance;
	static double angle;
	double l = 0.08;//小车的半径8cm
	Eigen::Matrix<double, 2, 1> U;
	Eigen::Matrix<double, 2, 2> R_;
	Eigen::Matrix<double, 3, 3> R;

	//Eigen::Matrix<double, 2, 1> return_value;
	R << 0,1,0,
		1,0,0,
		0,0,1;
	//四元素转换为欧拉角
	Eigen::Vector4d q ;
	q << transform_robot.getRotation().getW(),transform_robot.getRotation().getX(),transform_robot.getRotation().getY(),transform_robot.getRotation().getZ();//将四元素放入矩阵中

	angle = atan2(2*(q(0)*q(3)+q(1)*q(2)),1-2*(pow(q(2),2)+pow(q(3),2)));
	if(angle<0)angle= angle+2*pi; //将-180～180的范围改到0~360
	// X_0 <<0-transform_robot.getOrigin().x(),0-transform_robot.getOrigin().y(),angle;  //目标点-小车实际坐标，小车朝向

	R_ << cos(angle),sin(angle),-sin(angle)/l,cos(angle)/l;//旋转矩阵的逆
	//U = k1 * R_ * P  ;//小车的控制器并转换为线速度和角速度
	U = R_ * u;//小车的控制器并转换为线速度和角速度
	vel_msg.linear.x= U(0);//获得小车线速度
	vel_msg.angular.z = U(1);//获得小车角速度
	// 限幅
	float finial=0.25;
	if(vel_msg.linear.x>0.22)vel_msg.linear.x=0.211243*finial;
	if(vel_msg.linear.x<-0.22)vel_msg.linear.x=-0.211243*finial;
	if(vel_msg.angular.z>2.82)vel_msg.angular.z=2.811121*finial;
	if(vel_msg.angular.z<-2.82)vel_msg.angular.z=-2.811121*finial;

}


void callback1(const ros::TimerEvent&)
{
    Timing_ms++;
}
double angle_mod(double angle)
{
	if(angle<0)angle= angle+2*pi; //将-180～180的范围改到0~360
    return angle;
}

//角度计算，返回0～360
double angle_get(Eigen::Matrix<double, 2, 1> P1,Eigen::Matrix<double, 2, 1> P2,Eigen::Matrix<double, 2, 1> P3)
{
	double angle;
	// angle = angle_mod(angle_mod(atan2(P3(1,0)-P2(1,0),P3(0,0)-P2(0,0))) - angle_mod(atan2(P1(1,0)-P2(1,0),P1(0,0)-P2(0,0))));
	angle = angle_mod(atan2(P3(1,0)-P2(1,0),P3(0,0)-P2(0,0)) - atan2(P1(1,0)-P2(1,0),P1(0,0)-P2(0,0)));
	// cout<<"32:"<<angle_mod(atan2(P3(1,0)-P2(1,0),P3(0,0)-P2(0,0)))<<endl;
	// cout<<"31:"<<angle_mod(atan2(P1(1,0)-P2(1,0),P1(0,0)-P2(0,0)))<<endl;
    return angle;
}
//加权角度计算
double angle_get_s(double angle,double angle_r,double angle_l)
{
	double angle_result;
	angle_result = -exp(-lambda*angle)/(angle-angle_l)+exp(lambda*angle)/(angle_r-angle);
    return ks*angle_result;
}	


/*
作用：2-范数计算
输入：2维向量
返回：2-范数的平方
*/
double norm2(Eigen::Matrix<double, 2, 1> vec) 
{
    double sum2 = 0.0;
    for (int i = 0; i < vec.size(); i++) 
	{
        sum2 += vec[i] * vec[i];
    }
	// cout<< "vec.size():" << vec.size() <<endl;
	// cout<< "sum2:" << sum2 <<endl;
    return sum2;
}

int main(int argc, char** argv)
{
	// 初始化ROS节点
	ros::init(argc, argv, "Distance_angle_formation");
    //ros::Timer timer1 =node.createTimer(ros::Duration(0.001),callback1);
   	// 创建节点句柄
	ros::NodeHandle node;

	//turtlebot3_listener();
	//创建新进程进行监听
    //boost::thread server(turtlebot3_listener);
	


    // 创建发布速度控制指令的发布者
	ros::Publisher tb3_0_vel = node.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 10);
	ros::Publisher tb3_1_vel = node.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 10);
	ros::Publisher tb3_2_vel = node.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel", 10);
	ros::Publisher tb3_3_vel = node.advertise<geometry_msgs::Twist>("/tb3_3/cmd_vel", 10);
	ros::Publisher tb3_4_vel = node.advertise<geometry_msgs::Twist>("/tb3_4/cmd_vel", 10);
    // ros::Timer timer1 =node.createTimer(ros::Duration(0.001),callback1);
	// 根据tb3_0与tb3_1坐标系之间的位置关系，发布速度控制指令
	geometry_msgs::Twist vel_msg0;
	geometry_msgs::Twist vel_msg1;
	geometry_msgs::Twist vel_msg2;
	geometry_msgs::Twist vel_msg3;
	geometry_msgs::Twist vel_msg4;
	//geometry_msgs::Twist vel_msg5;

    // 创建发布距离角度指令的发布者
	ros::Publisher tb3_0_robot = node.advertise<sensor_msgs::Imu>("/tb3_0/robot", 10);
	ros::Publisher tb3_1_robot = node.advertise<sensor_msgs::Imu>("/tb3_1/robot", 10);
	ros::Publisher tb3_2_robot = node.advertise<sensor_msgs::Imu>("/tb3_2/robot", 10);
	ros::Publisher tb3_3_robot = node.advertise<sensor_msgs::Imu>("/tb3_3/robot", 10);
	ros::Publisher tb3_4_robot = node.advertise<sensor_msgs::Imu>("/tb3_4/robot", 10);
	sensor_msgs::Imu robot_msg0;
	sensor_msgs::Imu robot_msg1;
	sensor_msgs::Imu robot_msg2;
	sensor_msgs::Imu robot_msg3;
	sensor_msgs::Imu robot_msg4;
	sensor_msgs::Imu robot_msg5;



    // 创建tf的监听器
    tf::TransformListener listener;
    ros::Rate rate(10);
	
		while(node.ok())
		{

			// 获取turtle1与turtle2坐标系之间的tf数据
			tf::StampedTransform transformf0;
			tf::StampedTransform transformf1;
			tf::StampedTransform transformf2;
			tf::StampedTransform transformf3;
			tf::StampedTransform transformf4;
			tf::StampedTransform transformf5;
			tf::StampedTransform transformf6;
			tf::StampedTransform transformf7;
			tf::StampedTransform transformf8;
			tf::StampedTransform transformf9;
			try
			{
				listener.waitForTransform("/map", "/tag_0", ros::Time(0), ros::Duration(3.0));
				listener.lookupTransform("/map", "/tag_0", ros::Time(0), transformf0);

				listener.waitForTransform("/map", "/tag_1", ros::Time(0), ros::Duration(3.0));
				listener.lookupTransform("/map", "/tag_1", ros::Time(0), transformf1);


				listener.waitForTransform("/map", "/tag_2", ros::Time(0), ros::Duration(3.0));
				listener.lookupTransform("/map", "/tag_2", ros::Time(0), transformf2);

				listener.waitForTransform("/map", "/tag_3", ros::Time(0), ros::Duration(3.0));
				listener.lookupTransform("/map", "/tag_3", ros::Time(0), transformf3);

				listener.waitForTransform("/map", "/tag_4", ros::Time(0), ros::Duration(3.0));
				listener.lookupTransform("/map", "/tag_4", ros::Time(0), transformf4);

				listener.waitForTransform("/map", "/tag_5", ros::Time(0), ros::Duration(3.0));
				listener.lookupTransform("/map", "/tag_5", ros::Time(0), transformf5);

				listener.waitForTransform("/map", "/tag_6", ros::Time(0), ros::Duration(3.0));
				listener.lookupTransform("/map", "/tag_6", ros::Time(0), transformf6);

				listener.waitForTransform("/map", "/tag_7", ros::Time(0), ros::Duration(3.0));
				listener.lookupTransform("/map", "/tag_7", ros::Time(0), transformf7);


				listener.waitForTransform("/map", "/tag_8", ros::Time(0), ros::Duration(3.0));
				listener.lookupTransform("/map", "/tag_8", ros::Time(0), transformf8);

				listener.waitForTransform("/map", "/tag_9", ros::Time(0), ros::Duration(3.0));
				listener.lookupTransform("/map", "/tag_9", ros::Time(0), transformf9);
				barrierpoint(0,0) = transformf5.getOrigin().x();    //将障碍物1x坐标赋值给统一的矩阵
				barrierpoint(1,0) = transformf5.getOrigin().y();  //将障碍物1y坐标赋值给统一的矩阵
				barrierpoint(0,1) = transformf6.getOrigin().x();    //将障碍物2x坐标赋值给统一的矩阵
				barrierpoint(1,1) = transformf6.getOrigin().y();  //将障碍物2y坐标赋值给统一的矩阵
				barrierpoint(0,2) = transformf7.getOrigin().x();    //将障碍物3x坐标赋值给统一的矩阵
				barrierpoint(1,2) = transformf7.getOrigin().y();  //将障碍物3y坐标赋值给统一的矩阵
				barrierpoint(0,3) = transformf8.getOrigin().x();    //将障碍物3x坐标赋值给统一的矩阵
				barrierpoint(1,3) = transformf8.getOrigin().y();  //将障碍物3y坐标赋值给统一的矩阵
				barrierpoint(0,4) = transformf9.getOrigin().x();    //将障碍物3x坐标赋值给统一的矩阵
				barrierpoint(1,4) = transformf9.getOrigin().y();  //将障碍物3y坐标赋值给统一的矩阵
				

			}
			catch (tf::TransformException &ex) 
			{
				//ROS_WARN("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}
            

            //邻结矩阵
			L << 0,0,0,0,0,
				1,0,0,0,0,
				1,1,0,0,0,
				1,1,0,0,0,
				1,0,1,0,0;
			ros::Time now = ros::Time::now();
			if(time_flag == 1)
			{
				//ros::Time Timing = now;

				Timing = now.toSec();
				time_flag = 0;
			}
			else if(time_flag == 0)
			{
				Timing2=now.toSec();
			}
			
			Timing_ms = abs(Timing2-Timing);
			cout << "Timing_ms: " << Timing_ms <<endl;
			
	        position_desire <<  0,   -0.25, 0.25,-0.5,0.5,
								0.5,  -0.5, -0.5 , 0,0;


			all_desire_edge << 0,0,0,0,0,
					0.93,0,0,0,0,
					0.93,0.58,0,0,0,
					0.58,0.58,0,0,0,
					0.58,0,0.58,0,0;
			float edge_numble = 1.6;
			all_desire_edge = edge_numble*all_desire_edge;
			// float edge_all=0.5;
			// all_desire_edge(1,0)=all_desire_edge(2,0)=sqrt(2*(edge_all+cos(pi/5)));
			// all_desire_edge(2,1)=all_desire_edge(3,0)=all_desire_edge(3,1)=all_desire_edge(4,0)=all_desire_edge(4,2)=sqrt(2*(edge_all-cos(2*pi/5)));	
            p<<transformf0.getOrigin().x(),transformf1.getOrigin().x(),transformf2.getOrigin().x(),transformf3.getOrigin().x(),transformf4.getOrigin().x(),
                transformf0.getOrigin().y(),transformf1.getOrigin().y(),transformf2.getOrigin().y(),transformf3.getOrigin().y(),transformf4.getOrigin().y();
			//距离计算
			for (size_t i = 0; i < 5; i++)
			{
				for (size_t j = 0; j < 5; j++)
				{
					if(L(i,j)==1)
					{
						edge(i,j) = sqrt(pow(p(0,i)-p(0,j),2)+pow(p(1,i)-p(1,j),2));
					}
					all_edge(i,j) = sqrt(pow(p(0,i)-p(0,j),2)+pow(p(1,i)-p(1,j),2));
					// all_desire_edge(i,j) = sqrt(pow(position_desire(0,i)-position_desire(0,j),2)+pow(position_desire(1,i)-position_desire(1,j),2));
					/* code */
				}
				/* code */
			}


			//角度计算
			float angle_3,angle_4,angle_5;
			float desire_angle_3,desire_angle_4,desire_angle_5;
			angle_3 = angle_get(p.col(0),p.col(2),p.col(1));		
			angle_4 = angle_get(p.col(0),p.col(3),p.col(1));		
			angle_5 = angle_get(p.col(0),p.col(4),p.col(2));		
			
			
			desire_angle_3 = 1.256;//acos((all_desire_edge(2,0)*all_desire_edge(2,0)+all_desire_edge(2,1)*all_desire_edge(2,1)-all_desire_edge(1,0)*all_desire_edge(1,0))/2*all_desire_edge(2,0)*all_desire_edge(2,1));
			desire_angle_4 = 2*pi-1.884;//acos((all_desire_edge(3,0)*all_desire_edge(3,0)+all_desire_edge(3,1)*all_desire_edge(3,1)-all_desire_edge(1,0)*all_desire_edge(1,0))/2*all_desire_edge(3,0)*all_desire_edge(3,1));
			desire_angle_5 = 1.884;//acos((all_desire_edge(4,0)*all_desire_edge(4,0)+all_desire_edge(4,2)*all_desire_edge(4,2)-all_desire_edge(2,0)*all_desire_edge(2,0))/2*all_desire_edge(4,0)*all_desire_edge(4.2));		


			cout << "angle_3:"<< angle_3<<endl;
			cout << "angle_4:"<< angle_4<<endl;
			cout << "angle_5:"<< angle_5<<endl;
			cout << "desire_angle_3:"<< desire_angle_3<<endl;
			cout << "desire_angle_4:"<< desire_angle_4<<endl;
			cout << "desire_angle_5:"<< desire_angle_5<<endl;
			cout << "erro_angle_3:"<< angle_3-desire_angle_3<<endl;
			cout << "erro_angle_4:"<< angle_4-desire_angle_4<<endl;
			cout << "erro_angle_5:"<< angle_5-desire_angle_5<<endl;
            // cout << "desire_angle_5:"<< desire_angle_5<<endl;
			//角度限制
			beta_l3 = 0 - epsilon_0; //最小角度
			beta_r3 = (1+exp(-sigma*Timing_ms))*pi ; //最大角度
			beta_l4 = (1-exp(-sigma*Timing_ms))*pi - epsilon_0 ; //最小角度
			beta_r4 = 2 * pi; //最大角度
	
			beta_l5 = 0 - epsilon_0 ; //最小角度
			beta_r5 = (1+exp(-sigma*Timing_ms))*pi; //最大角度 ; //最大角度		
			
			//加权边的计算
			for (size_t i = 0; i < 5; i++)
			{
				for (size_t j = 0; j < 5; j++)
				{
					if(L(i,j)==1)
					{	
						s(i,j) = -exp(-lambda*edge(i,j))/(edge(i,j)-rs)	+exp(lambda*edge(i,j))/(rc-edge(i,j));
						desire_s(i,j) = -exp(-lambda*all_desire_edge(i,j))/(all_desire_edge(i,j)-rs)+exp(lambda*all_desire_edge(i,j))/(rc-all_desire_edge(i,j));
					}
					/* code */
				}
				/* code */
			}
			//加权角的计算
			float s_angle_3,s_angle_4,s_angle_5;
			float s_angle_3_star,s_angle_4_star,s_angle_5_star;
			float k_angle_s =0.1;
			s_angle_3 = k_angle_s*angle_get_s(angle_3,beta_r3,beta_l3);
			s_angle_4 = k_angle_s*angle_get_s(angle_4,beta_r4,beta_l4);
			s_angle_5 = k_angle_s*angle_get_s(angle_5,beta_r5,beta_l5);
			s_angle_3_star = k_angle_s*angle_get_s(desire_angle_3,beta_r3,beta_l3);
			s_angle_4_star = k_angle_s*angle_get_s(desire_angle_4,beta_r4,beta_l4);
			s_angle_5_star = k_angle_s*angle_get_s(desire_angle_5,beta_r5,beta_l5);


			// cout << "s_angle_3:"<< s_angle_3<<endl;
			// cout << "s_angle_3_star:"<< s_angle_3_star<<endl;
			//转换误差计算
			//加权边的计算
			unsigned char k=0;
			for (size_t i = 0; i < 5; i++)
			{
				for (size_t j = 0; j < 5; j++)
				{
					if(L(i,j)==1)
					{	
						zij(i,j) = s(i,j)-desire_s(i,j);
						eij(i,j) = edge(i,j)-all_desire_edge(i,j);
						cout << "e("<< i<<j<<"):"<< eij(i,j) <<endl;
						// cout << "zij("<< i<<j<<"):"<< zij(i,j) <<endl;
					}
					/* code */
				}
				/* code */
			}
			zk(0) = s_angle_3 - s_angle_3_star;
			zk(1) = s_angle_4 - s_angle_4_star;
			zk(2) = s_angle_5 - s_angle_5_star;

			// cout << "角度3:"<< zk(0)<<endl;
			// cout << "角度4:"<< zk(1)<<endl;
			// cout << "角度5:"<< zk(2)<<endl;
			Eigen::Matrix<double, 2, 5> u;//控制器

			Eigen::Matrix<double, 2, 5> f;//斥力矩阵
			Eigen::Matrix<double, 2, 2> J;//转换矩阵

			J<<0,-1,
				1,0;

			//时变函数
			static float mu = 0;
			static float mu0 = 1;
			static float K_mu = 1;
			// static float T_mu = 20;
			static float T_mu = 15;
			if(Timing_ms<T_mu)	mu = K_mu*(1-cos(pi/T_mu*Timing_ms))+mu0;
			else mu = 2*K_mu+mu0;

			//control
			u.col(0)<<0,0;
			// c_1 =0.06; c_2 = 0.04;	c_3 = 0.04, c_4 = 0.06;	
			// v_3 =0.10, v_4 = 0.12, v_5 = 0.14;
			c_1 =0.10; c_2 = 0.12;	c_3 = 0.12, c_4 = 0.16;	
			v_3 =0.14, v_4 = 0.16, v_5 = 0.18;
			u.col(1) = c_1*mu*zij(1,0)*(p.col(0)-p.col(1));
			u.col(2) = c_2*mu*(zij(2,0)*(p.col(0)-p.col(2))+zij(2,1)*((p.col(1)-p.col(2))))-v_3*zk(0)*J.transpose()*((p.col(1)-p.col(2))/norm2(p.col(1)-p.col(2))-(p.col(0)-p.col(2))/norm2(p.col(0)-p.col(2)));
			u.col(3) = c_3*mu*(zij(3,1)*(p.col(1)-p.col(3))+zij(3,0)*((p.col(0)-p.col(3))))-v_4*zk(1)*J.transpose()*((p.col(1)-p.col(3))/norm2(p.col(1)-p.col(3))-(p.col(0)-p.col(3))/norm2(p.col(0)-p.col(3)));
			u.col(4) = c_4*mu*(zij(4,0)*(p.col(0)-p.col(4))+zij(4,2)*((p.col(2)-p.col(4))))-v_5*zk(2)*J.transpose()*((p.col(2)-p.col(4))/norm2(p.col(2)-p.col(4))-(p.col(0)-p.col(4))/norm2(p.col(0)-p.col(4)));
			
			//APF
			f.col(1) = APF(p.col(1),barrierpoint.col(0))+APF(p.col(1),barrierpoint.col(1))+APF(p.col(1),barrierpoint.col(2))+APF(p.col(1),barrierpoint.col(3))+APF(p.col(1),barrierpoint.col(4));
			f.col(2) = APF(p.col(2),barrierpoint.col(0))+APF(p.col(2),barrierpoint.col(1))+APF(p.col(2),barrierpoint.col(2))+APF(p.col(2),barrierpoint.col(3))+APF(p.col(2),barrierpoint.col(4));
			f.col(3) = APF(p.col(3),barrierpoint.col(0))+APF(p.col(3),barrierpoint.col(1))+APF(p.col(3),barrierpoint.col(2))+APF(p.col(3),barrierpoint.col(3))+APF(p.col(3),barrierpoint.col(4));
			f.col(4) = APF(p.col(4),barrierpoint.col(0))+APF(p.col(4),barrierpoint.col(1))+APF(p.col(4),barrierpoint.col(2))+APF(p.col(4),barrierpoint.col(3))+APF(p.col(4),barrierpoint.col(4));
			
			// f.col(1) = APF(p.col(1),barrierpoint.col(0))+APF(p.col(1),barrierpoint.col(1))+APF(p.col(1),barrierpoint.col(2));
			// f.col(2) = APF(p.col(2),barrierpoint.col(0))+APF(p.col(2),barrierpoint.col(1))+APF(p.col(2),barrierpoint.col(2));
			// f.col(3) = APF(p.col(3),barrierpoint.col(0))+APF(p.col(3),barrierpoint.col(1))+APF(p.col(3),barrierpoint.col(2));
			// f.col(4) = APF(p.col(4),barrierpoint.col(0))+APF(p.col(4),barrierpoint.col(1))+APF(p.col(4),barrierpoint.col(2));
			
			// for(int i = 1 ;i<5;i++)
			// {
			// 	for (int j = 0; j < barrier_number; j++)
			// 	{
			// 		/* code */
			// 		f.col(i) = f.col(i)+APF(p.col(i),barrierpoint.col(j));
			// 	}
			// }

			
			u.col(1) = u.col(1) + f.col(1);
			u.col(2) = u.col(2) + f.col(2);
			u.col(3) = u.col(3) + f.col(3);
			u.col(4) = u.col(4) + f.col(4);

			//debug
			cout << "速度1:"<< vel_msg0.linear.x
			<< ","<< vel_msg0.angular.z<<endl;
			cout << "速度2:"<< vel_msg1.linear.x
			<< ","<< vel_msg1.angular.z<<endl;
			cout << "速度3:"<< vel_msg2.linear.x
			<< ","<< vel_msg2.angular.z<<endl;
			cout << "速度4:"<< vel_msg3.linear.x
			<< ","<< vel_msg3.angular.z<<endl;
			cout << "速度5:"<< vel_msg4.linear.x
			<< ","<< vel_msg4.angular.z<<endl;
			cout << "距离和:"<< distance_sum<<endl;

			//速度发布
			StampedTransformToTwist_Angle_Distance(transformf1,u.col(1),vel_msg1);
			StampedTransformToTwist_Angle_Distance(transformf2,u.col(2),vel_msg2);
			StampedTransformToTwist_Angle_Distance(transformf3,u.col(3),vel_msg3);
			StampedTransformToTwist_Angle_Distance(transformf4,u.col(4),vel_msg4);




			//time 
			robot_msg0.header.stamp = ros::Time::now();
			robot_msg1.header.stamp = ros::Time::now();
			robot_msg2.header.stamp = ros::Time::now();
			robot_msg3.header.stamp = ros::Time::now();
			robot_msg4.header.stamp = ros::Time::now();
			//error
			//eij(i,j) 
			// angle_3-desire_angle_3<<endl;
			// cout << "erro_angle_4:"<< angle_4-desire_angle_4<<endl;
			// cout << "erro_angle_5:"<< angle_5-desire_angle_5<<endl;

			// all_desire_edge(1,0)=all_desire_edge(2,0)=sqrt(2*(edge_all+cos(pi/5)));
			// all_desire_edge(2,1)=all_desire_edge(3,0)=all_desire_edge(3,1)=all_desire_edge(4,0)=all_desire_edge(4,2)=sqrt(2*(edge_all-cos(2*pi/5)));	
			robot_msg0.linear_acceleration.x = eij(1,0) ;
			robot_msg0.linear_acceleration.y = eij(2,0) ;
			robot_msg1.linear_acceleration.x = eij(2,1) ;
			robot_msg1.linear_acceleration.y = eij(3,0) ;
			robot_msg2.linear_acceleration.x = eij(3,1) ;
			robot_msg2.linear_acceleration.y = eij(4,0) ;
			robot_msg3.linear_acceleration.x = eij(4,2) ;
			robot_msg3.linear_acceleration.y = angle_3-desire_angle_3;
			robot_msg4.linear_acceleration.x = angle_4-desire_angle_4;
			robot_msg4.linear_acceleration.y = angle_5-desire_angle_5;
			//point position
			robot_msg0.orientation.x = transformf0.getOrigin().x();
			robot_msg0.orientation.y = transformf0.getOrigin().y();
			robot_msg1.orientation.x = transformf1.getOrigin().x();
			robot_msg1.orientation.y = transformf1.getOrigin().y();
			robot_msg2.orientation.x = transformf2.getOrigin().x();
			robot_msg2.orientation.y = transformf2.getOrigin().y();
			robot_msg3.orientation.x = transformf3.getOrigin().x();
			robot_msg3.orientation.y = transformf3.getOrigin().y();
			robot_msg4.orientation.x = transformf4.getOrigin().x();
			robot_msg4.orientation.y = transformf4.getOrigin().y();

			//distance and angle
			robot_msg0.angular_velocity.x = edge(1,0);
			robot_msg0.angular_velocity.y = edge(2,0);
			robot_msg1.angular_velocity.x = edge(2,1);
			robot_msg1.angular_velocity.y = edge(3,0);
			robot_msg2.angular_velocity.x = edge(3,1);
			robot_msg2.angular_velocity.y = edge(4,0);
			robot_msg3.angular_velocity.x = edge(4,2);
			robot_msg3.angular_velocity.y = angle_3;
			robot_msg4.angular_velocity.x = angle_4;
			robot_msg4.angular_velocity.y = angle_5;
			
			//input
			robot_msg0.orientation.w = u(0,0);
			robot_msg0.orientation.z = u(1,0);
			robot_msg1.orientation.w = u(0,1);
			robot_msg1.orientation.z = u(1,1);
			robot_msg2.orientation.w = u(0,2);
			robot_msg2.orientation.z = u(1,2);
			robot_msg3.orientation.w = u(0,3);
			robot_msg3.orientation.z = u(1,3);
			robot_msg4.orientation.w = u(0,4);
			robot_msg4.orientation.z = u(1,4);
			

			tb3_0_robot.publish(robot_msg0);//发布出去
			tb3_1_robot.publish(robot_msg1);//发布出去
			tb3_2_robot.publish(robot_msg2);//发布出去
			tb3_3_robot.publish(robot_msg3);//发布出去
			tb3_4_robot.publish(robot_msg4);//发布出去


			tb3_1_vel.publish(vel_msg1);
			tb3_2_vel.publish(vel_msg2);
			tb3_3_vel.publish(vel_msg3);
			tb3_4_vel.publish(vel_msg4);
			
			rate.sleep();
			
		}
	
    ros::spin();
	return 0;
};


Eigen::Matrix<double, 2, 1> APF(Eigen::Matrix<double, 2, 1> robotpoint,Eigen::Matrix<double, 2, 1>  barrierpoint)
{
	//APF
	double r_min = 0.00;//最小安全距离
	double r_max = 0.42;//最大安全距离
	double alpha =0.088; //斥力系数

	// Eigen::Matrix<double, 5, 5> Zij;//距离
	// Eigen::Matrix<double, 5, 5> Xij_x;//相对坐标
	// Eigen::Matrix<double, 5, 5> Xij_y;//相对坐标
	// Eigen::Matrix<double, 5, 5> fij_x;//斥力
	// Eigen::Matrix<double, 5, 5> fij_y;//斥力
	// Eigen::Matrix<double, 2, 5> fi;//斥力之和


	double Zij;//距离
	double Xij_x;//相对坐标
	double Xij_y;//相对坐标
	Eigen::Matrix<double, 2, 1> fi;//斥力

		

	Xij_x = robotpoint(0)-barrierpoint(0);//获得和其他小车的相对坐标
	Xij_y = robotpoint(1)-barrierpoint(1);//获得和其他小车的相对坐标
	Zij = sqrt((robotpoint-barrierpoint).transpose()*(robotpoint-barrierpoint));//获得和其他小车的距离
	if(Zij>r_min&&Zij<r_max)
	{
		fi(0) = alpha*(r_max-Zij)*Xij_x/(r_min-Zij)/(r_min-Zij)/Zij;
		fi(1) = alpha*(r_max-Zij)*Xij_y/(r_min-Zij)/(r_min-Zij)/Zij;
	}
	else
	{
		fi(0) = 0;
		fi(1) = 0;
	}
	return fi;
}

/*
rosbag record /tb3_0/robot /tb3_1/robot /tb3_2/robot /tb3_3/robot  /tb3_4/robot -o ~/catkin_ws/src/mrs-tb3/demo/tb3_base_camera_control/rosbag/robot.bag
*/