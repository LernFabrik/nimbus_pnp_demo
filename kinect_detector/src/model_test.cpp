#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "model_test_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("model_point", 5);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

    // Load Point for perticular location

    pcl::io::loadPCDFile("/home/vishnu/ros_ws/test/box.pcd", *cloud);

    double px = cloud->sensor_origin_[0];
    double py = cloud->sensor_origin_[1];
    double pz = cloud->sensor_origin_[2];
    double x = cloud->sensor_orientation_.x();
    double y = cloud->sensor_orientation_.y();
    double z = cloud->sensor_orientation_.z();
    double w = cloud->sensor_orientation_.w();

    cloud->header.frame_id = "model";
    ros::Rate r(1);

    while (ros::ok())
    {
        ROS_INFO("Printing cloud information");
        ROS_INFO("Sensor origin px:%d , py: %d, pz: %d", px, py, pz);
        ROS_INFO("Sensor orientation x: %d, y: %d, z: %d, w: %d", x, y, z, w);

        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        pub.publish(cloud);

        r.sleep();
    }
    
    return 0;
}