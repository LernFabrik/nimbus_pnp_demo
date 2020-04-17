#ifndef _FEATURES_H_
#define _FEATURES_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>

/** 
 * http://www.pointclouds.org/documentation/tutorials/#features-tutorial
 * http://www.pointclouds.org/documentation/tutorials/how_features_work.php#rusudissertation
*/
namespace nimbus{
    /**
     * @brief 
     * 
     * @tparam PointType 
     * @tparam NormalType 
     * @tparam DescriptorType 
     */
    template <class PointType, class NormalType, class DescriptorType>
    class Features{
        private:
            ros::NodeHandle _nh;
            typedef pcl::PointCloud<PointType> PointCloudType;
            typedef typename PointCloudType::Ptr PointCloudTypePtr;
            typedef typename PointCloudType::ConstPtr PointCloudTypeConstPtr;
        
        protected:
            typename pcl::PointCloud<NormalType>::Ptr _normals;
            typename pcl::PointCloud<DescriptorType>::Ptr _descriptor;
        
        public:
            Features(ros::NodeHandle nh);
            ~Features();
            // Variables
            double _norm_sr;
            double _desc_sr;
            //Functions
            /**
             * @brief Normal Estimation with Kd Tree Search method
             * @param blob 
             */
            void cloudNormalEstimationOMP(const PointCloudTypeConstPtr blob);
            /**
             * @brief Fast Point Feature Histogram(FPFH) descriptor
             * http://www.pointclouds.org/documentation/tutorials/fpfh_estimation.php#fpfh-estimation
             * @param blob 
             */
            void cloudFPFHEstimation(const PointCloudTypeConstPtr blob);
            
    };
}
template <class PointType, class NormalType, class DescriptorType>
nimbus::Features<PointType, NormalType, DescriptorType>::Features(ros::NodeHandle nh): _nh(nh){}
template <class PointType, class NormalType, class DescriptorType>
nimbus::Features<PointType, NormalType, DescriptorType>::~Features(){}

template <class PointType, class NormalType, class DescriptorType>
void nimbus::Features<PointType, NormalType, DescriptorType>::cloudNormalEstimationOMP(const PointCloudTypeConstPtr blob)
{
    pcl::NormalEstimationOMP<PointType, NormalType> ne;
    ne.setInputCloud(blob);
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(_norm_sr);
    _normals.reset(new pcl::PointCloud<NormalType>());
    ne.compute(*_normals);
}

template <class PointType, class NormalType, class DescriptorType>
void nimbus::Features<PointType, NormalType, DescriptorType>::cloudFPFHEstimation(const PointCloudTypeConstPtr blob)
{
    pcl::FPFHEstimation<PointType, NormalType, DescriptorType> fpfh;
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    fpfh.setSearchSurface(blob);
    //ToDo: determine the Keypoints
    fpfh.setInpoutCloud(blob);
    this->cloudNormalEstimationOMP(blob);
    fpfh.setInputNormals(_normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(_desc_sr);
    _descriptor.reset(new pcl::PointCloud<DescriptorType>());
    fpfh.compute(*_descriptor)
}

#endif