#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <pcl/surface/mls.h>
#include <pcl/correspondence.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <dynamic_reconfigure/server.h>
#include <nimbus_cloud/searchRadiusConfig.h>

#include <nimbus_cloud/cloud_mean.h>
#include <nimbus_cloud/cloud_recognition.h>

double _ns;
double _ks;
double _ds;
double rf_rad_;
double cg_size_;
double cg_thresh_;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
PointCloud blob;
bool newCloud = false;
void callback(const PointCloud::ConstPtr& msg){
    pcl::copyPointCloud(*msg, blob);
    newCloud = true;
}

void dynamicCallback(nimbus_cloud::searchRadiusConfig &config, uint32_t level){
    _ns = config.normal_sr_;
    _ks = config.keypoint_sr_;
    _ds = config.descriptor_sr_;
    rf_rad_ = config.rf_rad_;
    cg_size_ = config.cg_size_;
    cg_thresh_ = config.cg_thresh_;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "nimbus_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/nimbus/pointcloud", 10, callback);
    ros::Publisher pub = nh.advertise<PointCloud>("filtered_cloud", 5);
    ros::Publisher pubPose = nh.advertise<geometry_msgs::Transform>("pose_detection", 5);

    dynamic_reconfigure::Server<nimbus_cloud::searchRadiusConfig> server;
    dynamic_reconfigure::Server<nimbus_cloud::searchRadiusConfig>::CallbackType dF;
    dF = boost::bind(&dynamicCallback, _1, _2);
    server.setCallback(dF);

    static tf2_ros::StaticTransformBroadcaster staticTrans;

    pcl::PointCloud<pcl::PointXYZI>::Ptr model(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile("/home/vishnu/ros_ws/catkin_nim_ws/src/nimbus_cloud/test/juice_carton.pcd", *model);
    pcl::PointCloud<pcl::PointXYZI>::Ptr scene (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr scene_blob (new pcl::PointCloud<pcl::PointXYZI>());

    cloudMean<pcl::PointXYZI> cMean(nh);
    nimbus::cloudRecognition<pcl::PointXYZI, pcl::Normal> cRecog(nh);

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

    geometry_msgs::Transform pose;

    while (ros::ok())
    {
        cRecog.updateParm(_ns, _ks, _ds, rf_rad_, cg_size_, cg_thresh_);
        if(newCloud && cMean.cloudQueue.size() < 5){
            cMean.cloudQueue.enqueue(blob);
            newCloud = false;
        }else{
            cRecog.modelConstruct(model);
            
            cMean.meanFilter(*scene_blob, blob.width, blob.height);
            cRecog.remover(scene_blob, blob.width, blob.height, 0.5, 0.5, *scene);
            scene->is_dense = false;
            
            // Clustering
            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
            std::vector<pcl::Correspondences> clustered_corrs;
            cRecog.cloudHough3D(scene, rototranslations, clustered_corrs);
            std::cout << "Model instances found: " << rototranslations.size () << std::endl;
            for(std::size_t i = 0; i < rototranslations.size(); ++i){
                std::cout << "\n  Instance " << i+1 << ":" << std::endl;
                std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
                //std::cout << "        Scale: " << scale[i] << std::endl;

                Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0,0);
                Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
                tf2::Matrix3x3 mat(rotation (0,0), rotation (0,1), rotation (0,2),
                                   rotation (1,0), rotation (1,1), rotation (1,2),
                                   rotation (2,0), rotation (2,1), rotation (2,2));
                tf2Scalar roll, pitch, yaw;
                mat.getEulerYPR(yaw, pitch, roll);
                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);
                pose.translation.x = translation(0);
                pose.translation.y = translation(1);
                pose.translation.z = translation(2);
                pose.rotation = tf2::toMsg(q);
                pubPose.publish(pose);
            }
            scene->header.frame_id= "detection";
            pcl_conversions::toPCL(ros::Time::now(), scene->header.stamp);
            pub.publish(scene);
            scene.reset(new pcl::PointCloud<pcl::PointXYZI>());
            scene_blob.reset(new pcl::PointCloud<pcl::PointXYZI>());
        }
        cameraPose.header.stamp = ros::Time::now();
        staticTrans.sendTransform(cameraPose);
        ros::spinOnce();
    }
    
    ros::spin();
}