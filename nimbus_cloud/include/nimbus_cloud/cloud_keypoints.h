#ifndef _CLOUD_KEYPOINTS_H_
#define _CLOUD_KEYPOINTS_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/impl/iss_3d.hpp>
#include <pcl/keypoints/narf_keypoint.h>

#include <nimbus_cloud/cloud_util.h>

/** 
 * http://docs.pointclouds.org/trunk/group__keypoints.html
 * http://www.pointclouds.org/documentation/tutorials/#keypoints-tutorial
*/
namespace nimbus{
    template <class PointInType>
    class cloudKeypoints : public cloudEdit<PointInType>{
        private:
            ros::NodeHandle _nh;
            typedef pcl::PointCloud<PointInType> PointCloud;
            typedef boost::shared_ptr<PointCloud> PointCloudPtr;
            typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;
        protected:
            typename pcl::PointCloud<PointInType>::Ptr keypointOut;
            double keypoint_sr;
        public:
            cloudKeypoints(ros::NodeHandle nh);
            ~cloudKeypoints();
            //Functions
            /** Uniform sampling Keypoints extraction (Detectors)
             * @brief http://docs.pointclouds.org/trunk/classpcl_1_1_uniform_sampling.html
             * @param Input Cloud
             * @param Output keypoint cloud
            */
           void cloudUniformSampling(const PointCloudConstPtr blob);

           /** ISSKeypoint3D 
            * Detects Intrinstic Shape Sigmature keypoint for a given point cloud
            * Reconmmended to implement Filter before
           */
          template <typename NormalType>
          void cloudISSKeypoints(const PointCloudConstPtr blob);

          /** NARF (Normal Aligned Radial Feature) */
          void cloudNarf(const PointCloudConstPtr blob);
            
    };
}
template <class PointInType>
nimbus::cloudKeypoints<PointInType>::cloudKeypoints(ros::NodeHandle nh):cloudEdit<PointInType>(nh), _nh(nh){}
template <class PointInType>
nimbus::cloudKeypoints<PointInType>::~cloudKeypoints(){}

template <class PointInType>
void nimbus::cloudKeypoints<PointInType>::cloudUniformSampling(const PointCloudConstPtr blob){
    pcl::UniformSampling<PointInType> uniSampling;
    double searchRadius = this->computeCloudResolution(blob);
    // this->keypoint_sr *= searchRadius;
    uniSampling.setInputCloud(blob);
    uniSampling.setRadiusSearch(keypoint_sr);
    keypointOut.reset(new pcl::PointCloud<PointInType>());
    uniSampling.filter(*keypointOut);
}

template <class PointInType>
template <typename NormalType>
void nimbus::cloudKeypoints<PointInType>::cloudISSKeypoints(const PointCloudConstPtr blob)
{
    double resolution = this->computeCloudResolution(blob);
    tree.reset(new pcl::search::KdTree<pcl::PointXYZI> ());
    pcl::ISSKeypoint3D<PointInType, PointInType, NormalType> iss;
    iss.setSalientRadius(6 * resolution);
    iss.setNonMaxRadius(4 * resolution);
    iss.setMinNeighbors(5);
    iss.setThreshold21(0.975);
    iss.setThreshold32(0.975);
    iss.setNumberOfThreads(4);
    iss.setInputCloud(blob);
    iss.setSearchMethod(tree);
    keypointOut.reset(new pcl::PointCloud<PointInType>());
    iss.compute(*keypointOut);
}

template <class PointInType>
void nimbus::cloudKeypoints<PointInType>::cloudNarf(const PointCloudConstPtr blob)
{
    pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
    this->_toRangeImage(blob, *range_image_ptr);
    pcl::RangeImage &range_image = *range_image_ptr;

    pcl::RangeImageBorderExtractor range_border_extractor;
    pcl::NarfKeypoint narf_keypoint(&range_border_extractor);
    narf_keypoint.getParameters().support_size = 0.01;
    narf_keypoint.getParameters().add_points_on_straight_edges = tree;
    narf_keypoint.getParameters().distance_for_additional_points = 0.5;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint.compute(keypoint_indices);

    keypointOut.reset(new pcl::PointCloud<PointInType>());
    pcl::PointCloud<pcl::PointXYZI> &keypoints = *keypointOut;
    keypoints.points.resize(keypoint_indices.points.size());
    for(std::size_t i = 0; i < keypoint_indices.points.size(); ++i)
        keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
}

#endif