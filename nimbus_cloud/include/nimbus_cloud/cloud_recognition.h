#ifndef _CLOUD_RECOGNITION_H_
#define _CLOUD_RECOGNITION_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>

#include <nimbus_cloud/cloud_features.h>

/** 
 * http://www.pointclouds.org/documentation/tutorials/#recognition-tutorial
 * http://docs.pointclouds.org/trunk/group__recognition.html
*/
namespace nimbus{
    struct modelData{
        pcl::PointCloud<pcl::Normal>::Ptr normal;
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoint;
        pcl::PointCloud<pcl::SHOT352>::Ptr descriptor;
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr blRefence;
    };

    template <class PointType, class NormalType>
    class cloudRecognition : public cloudFeatures<PointType, NormalType>{
        private:
            ros::NodeHandle _nh;
        public:
            modelData mData;
            pcl::CorrespondencesPtr model_scene_corr;
            double cg_size_, cg_thresh_;
        public:
            cloudRecognition(ros::NodeHandle nh);
            ~cloudRecognition();
            //Functions
            /** Model Constructor 
             * @brief This will run neccessary function and store the\
             * model metadata in the structure
            */
            void modelConstruct(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob);
            /** Update Params
             * This will run per loop and update the param/
             * if Changed by dymanic param */
            void updateParm(double ns, double ks, double ds, double rs, double cs, double ct); 
            /** 
             * Correspondence Matching
             * @brief It is RECOMMENDED to run "modelConstruct" before. 
             * @param input downsampled cloud
             * @param input search radius
             * @param output normal cloud
            */
            void cloudCorrespondence(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob);
                                
            void cloudHough3D(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob,
                              std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations, 
                              std::vector<pcl::Correspondences> &clustered_corrs);

    };
}

// https://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
// To remove the undefined reference to the class
template class nimbus::cloudRecognition<pcl::PointXYZI, pcl::Normal>;

#endif