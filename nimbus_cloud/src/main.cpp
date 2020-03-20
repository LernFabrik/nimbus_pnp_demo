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
    pcl::io::loadPCDFile("/home/vishnu/ros_ws/catkin_nim_ws/src/nimbus_cloud/test/model1.pcd", *model);

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
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_ref (new pcl::PointCloud<pcl::ReferenceFrame>());

    model->is_dense = false;
    cFeature.cloudNormalEstimationOMP(model, 0.01, *model_norm);
    cKeypoints.cloudUniformSampling(model, model_ks_, model_keypoint);
    cFeature.cloudSHOTEstimationOMP(model_keypoint, model_norm, model, 0.01, model_descriptor);
    cFeature.cloudBoardLocalRefeFrame(model_keypoint, model_norm, model, 0.015, model_ref);

    while (ros::ok())
    {
        pcl::PointCloud<pcl::Normal>::Ptr scene_norm (new pcl::PointCloud<pcl::Normal>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr scene (new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr scene_blob (new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr scene_keypoint (new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptor (new pcl::PointCloud<pcl::SHOT352>());
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_ref (new pcl::PointCloud<pcl::ReferenceFrame>());
        pcl::CorrespondencesPtr model_scene_corr (new pcl::Correspondences());

        if(newCloud && cMean.cloudQueue.size() < 2){
            cMean.cloudQueue.enqueue(blob);
            newCloud = false;
        }else{
            cMean.meanFilter(*scene_blob, blob.width, blob.height);
            cFeature.remover(scene_blob, blob.width, blob.height, 0.7, 0.6, *scene);
            scene->is_dense = false;
            
            cFeature.cloudNormalEstimationOMP(scene, 0.01,*scene_norm);
            cKeypoints.cloudUniformSampling(scene, 0.01, scene_keypoint);
            cFeature.cloudSHOTEstimationOMP(scene_keypoint, scene_norm, scene, 0.01, scene_descriptor);

            cRecognition.cloudCorrespondence(model_descriptor, scene_descriptor, model_scene_corr);
            std::cout << "Model scene Correspondence found: " << model_scene_corr->size() << std::endl;

            // Clustering
            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
            std::vector<pcl::Correspondences> clustered_corrs;
            cFeature.cloudBoardLocalRefeFrame(scene_keypoint, scene_norm, scene, 0.015, scene_ref);

            typedef pcl::PointXYZI PointTp;
            typedef pcl::ReferenceFrame RefereFm;
            pcl::Hough3DGrouping< PointTp, PointTp, RefereFm, RefereFm> clusterer;
            clusterer.setHoughBinSize (0.017);
            clusterer.setHoughThreshold (3.5);
            clusterer.setUseInterpolation (true);
            clusterer.setUseDistanceWeight (false);

            clusterer.setInputCloud (model_keypoint);
            clusterer.setInputRf (model_ref);
            clusterer.setSceneCloud (scene_keypoint);
            clusterer.setSceneRf (scene_ref);
            clusterer.setModelSceneCorrespondences (model_scene_corr);
            //std::vector<double> scale = clusterer.getCharacteristicScales();

            clusterer.recognize (rototranslations, clustered_corrs);

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
                ROS_ERROR("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
                printf ("\n");
                printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
                printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
                printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
                printf ("\n");
                printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
            }
            model->header.frame_id= "detection";
            pcl_conversions::toPCL(ros::Time::now(), model->header.stamp);
            pub.publish(model);
        }
        cameraPose.header.stamp = ros::Time::now();
        staticTrans.sendTransform(cameraPose);
        ros::spinOnce();
    }
    
    ros::spin();
}