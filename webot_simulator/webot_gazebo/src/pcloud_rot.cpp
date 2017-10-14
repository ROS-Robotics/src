/*
 * pcloud_rot.cpp
 *
 *  Created on: Jan 11, 2015
 *      Author: lab-118
 */


#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

	ros::Publisher rotated_cloud;
	Eigen::Matrix4f rotMatrixX ;
	Eigen::Matrix4f rotMatrixY ;
	Eigen::Matrix4f rotMatrixZ ;
	Eigen::Matrix4f rot_matrix ;

void rotCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
		sensor_msgs::PointCloud2 transformed_cloud;

		pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud ;

		pcl::fromROSMsg(*msg , in_pointcloud);
		pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
		pcl::transformPointCloud(in_pointcloud , out_pointcloud , rot_matrix );
		pcl::toROSMsg(out_pointcloud , transformed_cloud);
		rotated_cloud.publish(transformed_cloud);
}

int main (int argc , char** argv)
{
	ros::init(argc , argv , "cloud_rot");
	ros::NodeHandle n ;
	ros::Rate loop_rate(10) ;

	char robot[50] = {0} ;

	//sprintf(robot,"%s_%s",argv[2], argv[1]);
	
	//std::string pre(robot);
	//std::string name(argv[2]);
	
	rotated_cloud = n.advertise<sensor_msgs::PointCloud2>(std::string("Asus_Camera/depth/points"),1);
	ros::Subscriber cloud_sub = n.subscribe(std::string("Asus_Camera/depth/rotated_points"),1, rotCallback);

	double rotx = 0.0 ;
	double roty = M_PI_2 ;
	double rotz = -M_PI_2 ;

	rotMatrixX <<
			1.0, 0.0, 0.0, 0.0,
			0.0, cos(rotx), -sin(rotx), 0.0,
			0.0, sin(rotx), cos(rotx), 0.0,
			0.0, 0.0, 0.0, 1.0;

	rotMatrixY <<
			cos(roty), 0.0, sin(roty), 0.0,
			0.0, 1.0, 0.0, 0.0,
			-sin(roty), 0.0, cos(roty), 0.0,
			0.0, 0.0, 0.0, 1.0;

	rotMatrixZ <<
			cos(rotz), -sin(rotz), 0.0, 0.0,
			sin(rotz), cos(rotz), 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0;

	rot_matrix = (rotMatrixX * rotMatrixY)*rotMatrixZ;

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

