#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>


class ModelTraining
{
    protected:
        pcl::PointCloud<pcl::PointXYZI>::Ptr blob;
        bool save_point_cloud;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nimbus_model_training_node");
    ros::NodeHandle nh;
}