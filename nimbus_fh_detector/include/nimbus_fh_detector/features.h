#ifndef _FEATURES_H_
#define _FEATURES_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>

#include <nimbus_fh_detector/filters.h>

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

            // Variables
            double _keypoint_sr;
            double _norm_sr;
            double _desc_sr;
        
        public:
            Features(ros::NodeHandle nh, double normal_sr, double descriptor_sr, double keypoint_sr);
            ~Features();
            // Variables
            typename pcl::PointCloud<PointType>::Ptr keypoints;
            typename pcl::PointCloud<NormalType>::Ptr normals;
            typename pcl::PointCloud<DescriptorType>::Ptr descriptor;
            //Functions
            void keypointUniformSampling(const PointCloudTypeConstPtr blob, PointCloudType &res);
            /**
             * @brief Normal Estimation with Kd Tree Search method
             * @param blob 
             */
            void cloudNormalEstimationOMP(const PointCloudTypeConstPtr blob, PointCloudType &res);
            /**
             * @brief Fast Point Feature Histogram(FPFH) descriptor
             * http://www.pointclouds.org/documentation/tutorials/fpfh_estimation.php#fpfh-estimation
             * @param blob 
             */
            void cloudFPFHEstimation(const PointCloudTypeConstPtr blob, pcl::PointCloud<DescriptorType> &descriptor);
            void extraction(const PointCloudTypeConstPtr blob);
            
    };
}
template <class PointType, class NormalType, class DescriptorType>
nimbus::Features<PointType, NormalType, DescriptorType>::Features(ros::NodeHandle nh, 
                                                                  double normal_sr, 
                                                                  double descriptor_sr, 
                                                                  double keypoint_sr): _nh(nh),
                                                                                       _norm_sr(normal_sr),
                                                                                       _desc_sr(descriptor_sr),
                                                                                       _keypoint_sr(keypoint_sr){}
template <class PointType, class NormalType, class DescriptorType>
nimbus::Features<PointType, NormalType, DescriptorType>::~Features(){}

template <class PointType, class NormalType, class DescriptorType>
void nimbus::Features<PointType, NormalType, DescriptorType>::keypointUniformSampling(const PointCloudTypeConstPtr blob, PointCloudType &res)
{
    pcl::UniformSampling<PointType> uniform;
    uniform.setInputCloud(blob);
    uniform.setRadiusSearch(_keypoint_sr);
    typename pcl::PointCloud<PointType>::Ptr keypoint (new pcl::PointCloud<PointType>());
    uniform.filter(*keypoint);
    pcl::copyPointCloud(*keypoint, res);
}

template <class PointType, class NormalType, class DescriptorType>
void nimbus::Features<PointType, NormalType, DescriptorType>::cloudNormalEstimationOMP(const PointCloudTypeConstPtr blob, PointCloudType &res)
{
    pcl::NormalEstimationOMP<PointType, NormalType> ne;
    ne.setInputCloud(blob);
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(_norm_sr);
    typename pcl::PointCloud<NormalType>::Ptr _normals (new pcl::PointCloud<NormalType>());
    ne.compute(*_normals);
    pcl::copyPointCloud(*_normals, res);
}

template <class PointType, class NormalType, class DescriptorType>
void nimbus::Features<PointType, NormalType, DescriptorType>::cloudFPFHEstimation(const PointCloudTypeConstPtr blob, pcl::PointCloud<DescriptorType> &descriptor)
{
    keypoints.reset(new pcl::PointCloud<PointType>());
    normals.reset (new pcl::PointCloud<NormalType>());
    keypointUniformSampling(blob, *keypoints);
    cloudNormalEstimationOMP(blob, *normals);

    pcl::FPFHEstimation<PointType, NormalType, DescriptorType> fpfh;
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    fpfh.setSearchSurface(blob);
    fpfh.setInpoutCloud(keypoints);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(_desc_sr);
    typename pcl::PointCloud<DescriptorType>::Ptr _descriptor (new pcl::PointCloud<DescriptorType>());
    fpfh.compute(*_descriptor);
    pcl::copyPointCloud(*_descriptor, descriptor);
}

template <class PointType, class NormalType, class DescriptorType>
void nimbus::Features<PointType, NormalType, DescriptorType>::extraction(const PointCloudTypeConstPtr blob)
{
    typename PointCloudType::Ptr res (new PointCloudType());
    Filters<PointType> filters(_nh);
    filters.movingLeastSquare<PointType, PointType>(blob, res);
    descriptor.reset(new pcl::PointCloud<DescriptorType>());
    cloudFPFHEstimation(res, *descriptor);
}

#endif