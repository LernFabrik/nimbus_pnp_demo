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

#include <nimbus_cloud/cloud_mean.h>
#include <nimbus_cloud/cloud_util.h>
#include <nimbus_cloud/cloudEditConfig.h>
#include <dynamic_reconfigure/server.h>

double remove_w, remove_h, z_max, z_min;
bool save = false;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud cloud_blob;
bool newCloud = false;
void callback(const PointCloud::ConstPtr& msg){
    pcl::copyPointCloud(*msg, cloud_blob);
    newCloud = true;
}

void saveCallback(const std_msgs::Bool::ConstPtr &msg)
{
    save = msg->data;
}

void dynamicCallback(nimbus_cloud::cloudEditConfig &config, uint32_t data)
{
    remove_w = config.per_width;
    remove_h = config.per_height;
    z_max = config.z_max;
    z_min = config.z_min;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "nimbus_driver_io_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 10, callback);
    ros::Subscriber subSave = nh.subscribe<std_msgs::Bool>("save_pointcloud", 10, saveCallback);
    ros::Publisher pub = nh.advertise<PointCloud>("pointcloud", 5);
    static tf2_ros::StaticTransformBroadcaster staticTrans;

    cloudMean<pcl::PointXYZ> cE(nh);
    nimbus::cloudEdit <pcl::PointXYZ> cloud_edit(nh);

    dynamic_reconfigure::Server<nimbus_cloud::cloudEditConfig> server;
    dynamic_reconfigure::Server<nimbus_cloud::cloudEditConfig>::CallbackType f;
    f = boost::bind(&dynamicCallback, _1, _2);
    server.setCallback(f);

    geometry_msgs::TransformStamped cameraPose;
    cameraPose.child_frame_id = "Mcamera";
    cameraPose.header.frame_id = "camera_base";
    cameraPose.transform.translation.x = 0;
    cameraPose.transform.translation.y = 0;
    cameraPose.transform.translation.z = 1;
    tf2::Quaternion q;
    q.setRPY(-1.57, 1.57, 0);
    cameraPose.transform.rotation.x = q.x();
    cameraPose.transform.rotation.y = q.y();
    cameraPose.transform.rotation.z = q.z();
    cameraPose.transform.rotation.w = q.w();
    
    while (ros::ok())
    {
        if(newCloud){
            
            PointCloud::Ptr cloud(new PointCloud());
            PointCloud::Ptr cloudE(new PointCloud());
            PointCloud::Ptr cloudZ(new PointCloud());
            pcl::copyPointCloud(cloud_blob, *cloud);
            cloud_edit.remover(cloud, cloud_blob.width, cloud_blob.height, remove_w, remove_h, *cloudE);
            float addZ = 0;
            float counter = 0;
            
            cloud_edit.zRemover(cloudE, z_max, z_min, *cloudZ);
            if(save == true){
                ROS_INFO("Saving");
                pcl::io::savePCDFile("model1.pcd", *cloudZ);
                ROS_INFO("Saved");
                save == false;
            }
            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            // pcl::RangeImage::Ptr range_image (new pcl::RangeImage);
            // cloud_edit._toRangeImage(cloudZ, *range_image);
            // ROS_INFO("Range Image size: %d", (int)range_image->points.size());
            // pcl::visualization::PCLVisualizer viewer("Range Image");
            // viewer.setBackgroundColor(1,1,1);
            // viewer.addCoordinateSystem(1.0f, "global");
            // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image, 150, 150, 150);
            // viewer.addPointCloud(range_image, range_image_color_handler, "range image");
            // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
            // viewer.spin();
            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            cloud->header.frame_id= "Mcamera";
            pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
            pub.publish(cloud);
            cloud.reset();
            // cloud_blob.points.clear()
            newCloud = false;
        }
        cameraPose.header.stamp = ros::Time::now();
        staticTrans.sendTransform(cameraPose);
        ros::spinOnce();
    }
    
    ros::spin();
}