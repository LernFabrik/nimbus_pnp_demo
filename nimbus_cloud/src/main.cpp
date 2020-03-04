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
// #include <nimbus_cloud/cloud_edit.h>
#include <nimbus_cloud/cloud_features.h>
#include <nimbus_cloud/cloud_keypoints.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
PointCloud blob;
bool newCloud = false;
void callback(const PointCloud::ConstPtr& msg){
    pcl::PointXYZI points;
    blob.width = msg->width;
    blob.height = msg->height;
    blob.header = msg->header;
    blob.points = msg->points;
    newCloud = true;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "nimbus_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/nimbus/pointcloud", 10, callback);
    ros::Publisher pub = nh.advertise<PointCloud>("pointcloud", 5);
    static tf2_ros::StaticTransformBroadcaster staticTrans;

    // cloudMean<pcl::PointXYZI> cE(nh);
    //nimbus::cloudEdit <pcl::PointXYZI> cloud_edit(nh);
    nimbus::cloudFeatures<pcl::PointXYZI, pcl::Normal> cFeature(nh);
    nimbus::cloudKeypoints<pcl::PointXYZI> cKeypoints(nh);

    geometry_msgs::TransformStamped cameraPose;
    cameraPose.child_frame_id = "detection";
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
            pcl::PointCloud<pcl::Normal>::Ptr scene_norm (new pcl::PointCloud<pcl::Normal>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr scene (new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr scene_keypoint (new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptor (new pcl::PointCloud<pcl::SHOT352>());
            for(int i = 0; i < blob.height * blob.width; i++){
                pcl::PointXYZI temp;
                temp.x = blob.points[i].x;
                temp.y = blob.points[i].y;
                temp.z = blob.points[i].z;
                //temp.intensity = blob.points[i].intensity;
                scene->points.push_back(temp);
            }
            scene->height =  blob.height;
            scene->width = blob.width;
            scene->is_dense = false;
            cFeature.cloudNormalEstimationOMP(scene, *scene_norm);
            cKeypoints.cloudUniformSampling(scene, scene_keypoint);
            cFeature.cloudSHOTEstimationOMP(scene_keypoint, scene_norm, scene, scene_descriptor);
        }
        cameraPose.header.stamp = ros::Time::now();
        staticTrans.sendTransform(cameraPose);
        ros::spinOnce();
    }
    
    ros::spin();
}