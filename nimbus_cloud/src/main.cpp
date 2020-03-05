#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <pcl/correspondence.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include <nimbus_cloud/searchRadiusConfig.h>

#include <nimbus_cloud/cloud_mean.h>
#include <nimbus_cloud/cloud_edit.h>
#include <nimbus_cloud/cloud_features.h>
#include <nimbus_cloud/cloud_keypoints.h>
#include <nimbus_cloud/cloud_recognition.h>

double scene_ns_;
double model_ns_;
double scene_ks_;
double model_ks_;
double scene_ds_;
double model_ds_;
double rf_rad_;
double cg_size_;
double cg_thresh_;

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

void dynamicCallback(nimbus_cloud::searchRadiusConfig &config, uint32_t level){
    scene_ns_ = config.scene_ns_;
    model_ns_ = config.model_ns_;
    scene_ks_ = config.scene_ks_;
    model_ks_ = config.model_ks_;
    scene_ds_ = config.scene_ds_;
    model_ds_ = config.model_ds_;
    rf_rad_ = config.rf_rad_;
    cg_size_ = config.cg_size_;
    cg_thresh_ = config.cg_thresh_;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "nimbus_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/nimbus/pointcloud", 10, callback);
    ros::Publisher pub = nh.advertise<PointCloud>("pointcloud", 5);

    dynamic_reconfigure::Server<nimbus_cloud::searchRadiusConfig> dServer;
    dynamic_reconfigure::Server<nimbus_cloud::searchRadiusConfig>::CallbackType f;
    f = boost::bind(&dynamicCallback, _1, _2);
    dServer.setCallback(f);

    static tf2_ros::StaticTransformBroadcaster staticTrans;

    pcl::PointCloud<pcl::PointXYZI>::Ptr model(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile("/home/vishnu/ros_ws/catkin_nim_ws/src/nimbus_cloud/test/wCube.pcd", *model);

    cloudMean<pcl::PointXYZI> cMean(nh);
    nimbus::cloudFeatures<pcl::PointXYZI, pcl::Normal> cFeature(nh);
    nimbus::cloudKeypoints<pcl::PointXYZI> cKeypoints(nh);
    nimbus::cloudRecognition cRecognition(nh);

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

    pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoint (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::Normal>::Ptr model_norm (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptor (new pcl::PointCloud<pcl::SHOT352>());

    while (ros::ok())
    {
        pcl::PointCloud<pcl::Normal>::Ptr scene_norm (new pcl::PointCloud<pcl::Normal>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr scene (new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr scene_blob (new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr scene_keypoint (new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptor (new pcl::PointCloud<pcl::SHOT352>());
        if(newCloud && cMean.cloudQueue.size() < 2){
            cMean.cloudQueue.enqueue(blob);
            newCloud = false;
        }else{
            cMean.meanFilter(*scene_blob, blob.width, blob.height);
            cFeature.remover(scene_blob, blob.width, blob.height, 0.7, 0.6, *scene);
            scene->is_dense = false;
            model->is_dense = false;
            
            cFeature.cloudNormalEstimationOMP(scene, 0.01,*scene_norm);
            cFeature.cloudNormalEstimationOMP(model, 0.01, *model_norm);
            cKeypoints.cloudUniformSampling(scene, 0.01, scene_keypoint);
            cKeypoints.cloudUniformSampling(model, model_ks_, model_keypoint);
            cFeature.cloudSHOTEstimationOMP(scene_keypoint, scene_norm, scene, 0.01, scene_descriptor);
            cFeature.cloudSHOTEstimationOMP(model_keypoint, model_norm, model, 0.01, model_descriptor);

            pcl::CorrespondencesPtr model_scene_corr (new pcl::Correspondences());
            cRecognition.cloudCorrespondence(model_descriptor, scene_descriptor, model_scene_corr);
        }
        scene_keypoint->header.frame_id= "detection";
        pcl_conversions::toPCL(ros::Time::now(), scene_keypoint->header.stamp);
        pub.publish(scene_keypoint);
        cameraPose.header.stamp = ros::Time::now();
        staticTrans.sendTransform(cameraPose);
        ros::spinOnce();
    }
    
    ros::spin();
}