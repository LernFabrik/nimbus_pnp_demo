/**
 * MIT License
 * 
 * Copyright (c) 2020 IWT Wirtschaft und Technik GmbH
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file test.cpp
 * @author Vishnuprasad Prachandabhanu (vishnu.pbhat93@gmail.com)
 */

#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>

#include <box_detector/box_detector.hpp>

double distance_max, distance_min;

void updateParm(ros::NodeHandle nh)
{
    nh.getParam("distance_max", distance_max);
    nh.getParam("distance_min", distance_min);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("model_point", 5);
    nimbus::BoxDetector<pcl::PointXYZ> bDetector(nh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr blob (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

    Eigen::Vector4f box_param;
    float curvature;

    ros::Rate r(1);
    while (ros::ok())
    {
        pcl::io::loadPCDFile("/home/vishnu/ros_ws/test/box_modif2.pcd", *blob);
        updateParm(nh);
        bDetector.zAxisLimiter(blob, distance_max, distance_min, *cloud);
        cloud->is_dense = false;
        bDetector.computePointNormal(cloud, box_param, curvature);
        ROS_INFO("The Centroid a: %f, b: %f, c: %f, d: %f, curvature: %f", box_param[0], box_param[1], box_param[2], box_param[3], curvature);
        float yaw;
        bDetector.boxYaw(cloud, yaw);
        cloud->header.frame_id = "model";
        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        pub.publish(cloud);
        r.sleep();
    }
    
    return 0;
}