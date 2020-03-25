#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#include <nimbus_cloud/cloud_mean.h>
#include <nimbus_cloud/cloud_edit.h>
#include <nimbus_cloud/cloudEditConfig.h>
#include <dynamic_reconfigure/server.h>

double remove_w, remove_h, z_max, z_min;
bool save = false;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
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
    ros::Subscriber sub = nh.subscribe<PointCloud>("/nimbus/pointcloud", 10, callback);
    ros::Subscriber subSave = nh.subscribe<std_msgs::Bool>("save_pointcloud", 10, saveCallback);
    ros::Publisher pub = nh.advertise<PointCloud>("pointcloud", 5);
    static tf2_ros::StaticTransformBroadcaster staticTrans;

    cloudMean<pcl::PointXYZI> cE(nh);
    nimbus::cloudEdit <pcl::PointXYZI> cloud_edit(nh);

    dynamic_reconfigure::Server<nimbus_cloud::cloudEditConfig> server;
    dynamic_reconfigure::Server<nimbus_cloud::cloudEditConfig>::CallbackType f;
    f = boost::bind(&dynamicCallback, _1, _2);
    server.setCallback(f);

    geometry_msgs::TransformStamped cameraPose;
    cameraPose.child_frame_id = "Mcamera";
    cameraPose.header.frame_id = "world";
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
            if(cE.cloudQueue.size() < 20){
                cE.cloudQueue.enqueue(cloud_blob);
                newCloud = false;
            }
            else{
                PointCloud::Ptr cloud(new PointCloud());
                PointCloud::Ptr cloudE(new PointCloud());
                PointCloud::Ptr cloudZ(new PointCloud());
                cE.meanFilter (*cloud, cloud_blob.width, cloud_blob.height);
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
                cloudZ->header.frame_id= "Mcamera";
                pcl_conversions::toPCL(ros::Time::now(), cloudZ->header.stamp);
                pub.publish(cloudZ);
                cloud.reset();
            }
            // cloud_blob.points.clear()
        }
        cameraPose.header.stamp = ros::Time::now();
        staticTrans.sendTransform(cameraPose);
        ros::spinOnce();
    }
    
    ros::spin();
}