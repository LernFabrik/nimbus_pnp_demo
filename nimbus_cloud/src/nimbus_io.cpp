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

#include <nimbus_cloud/cloud_mean.h>
#include <nimbus_cloud/cloud_edit.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
PointCloud cloud_blob;
bool newCloud = false;
void callback(const PointCloud::ConstPtr& msg){
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    pcl::PointXYZI points;
    // BOOST_FOREACH (const pcl::PointXYZI& pt, msg->points){
    //     points.x = tempC.point[i]x;
    //     points.y = tempC.point[i]y;
    //     points.z = tempC.point[i]z;
    //     points.intensity = tempC.point[i]intensity;
    // }
    cloud_blob.width = msg->width;
    cloud_blob.height = msg->height;
    cloud_blob.header = msg->header;
    cloud_blob.points = msg->points;
    newCloud = true;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "nimbus_driver_io_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/nimbus/pointcloud", 10, callback);
    ros::Publisher pub = nh.advertise<PointCloud>("pointcloud", 5);
    static tf2_ros::StaticTransformBroadcaster staticTrans;

    cloudMean<pcl::PointXYZI> cE(nh);
    nimbus::cloudEdit <pcl::PointXYZI> cloud_edit(nh);

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
    bool save = true;
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
                cloud_edit.remover(cloud, cloud_blob.width, cloud_blob.height, 0.75, 0.65, *cloudE);
                float addZ = 0;
                float counter = 0;
                /*
                BOOST_FOREACH (const pcl::PointXYZI& pt, cloudE->points){
                    ROS_INFO("Monitore Z: %f\n", pt.z);
                    //addZ += pt.z;
                    //counter ++;
                }
                //ROS_INFO("Mean Distance: %f\n", (float)(addZ/counter));*/
                cloud_edit.zRemover(cloudE, 0.85, 0.6, *cloudZ);
                if(save == true){
                    pcl::io::savePCDFile("lwCube.pcd", *cloudZ);
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