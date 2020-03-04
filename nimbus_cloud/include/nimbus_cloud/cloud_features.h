#ifndef _CLOUD_FEATURES_H_
#define _CLOUD_FEATURES_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/shot_omp.h>

#include <nimbus_cloud/cloud_edit.h>

/** 
 * http://www.pointclouds.org/documentation/tutorials/#features-tutorial
 * http://www.pointclouds.org/documentation/tutorials/how_features_work.php#rusudissertation
*/
namespace nimbus{
    template <class PointType, class NormalType>
    class cloudFeatures : public cloudEdit<PointType> {
        private:
            ros::NodeHandle _nh;
            typedef pcl::PointCloud<PointType> PointCloud;
            typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
            typedef pcl::PointCloud<NormalType> NormalCloud;
            typedef boost::shared_ptr<NormalCloud> NormalCloudPtr;
            typedef boost::shared_ptr<const NormalCloud> NormalCloudConstPtr;

        public:
            cloudFeatures(ros::NodeHandle nh);
            ~cloudFeatures();
            //Functions
            /** 
             * @brief Kd Tree Search method
             * @param input downsampled cloud
             * @param input search radius
             * @param output normal cloud
            */
            void cloudNormalEstimation(const PointCloudConstPtr blob,
                                        NormalCloudPtr res);
            /** 
             * @brief Kd Tree Search method
             * @param input downsampled cloud
             * @param input search radius
             * @param output normal cloud
            */
            void cloudNormalEstimationOMP(const PointCloudConstPtr blob,
                                        pcl::PointCloud<NormalType> &res);
            
            /** 
             * @brief SHOT Estimation
             * @param input KeyPoints
             * @param input Normals
             * @param input Raw cloud for surface detection
             * @param output Discriptor of type pcl::SHOT352
            */
            void cloudSHOTEstimationOMP(const PointCloudConstPtr keyPoints,
                                        const NormalCloudConstPtr normalPoints, 
                                        const PointCloudConstPtr blob,
                                        pcl::PointCloud<pcl::SHOT352>::Ptr res);

            /** ToDo:
             * 1. Normal estimation with cloud indices.
             * 2. Implememt different feature extraction algorithms.
             * 3. Change the search radius method.
            */
    };
}
template <class PointType, class NormalType>
nimbus::cloudFeatures<PointType, NormalType>::cloudFeatures(ros::NodeHandle nh):cloudEdit<PointType>(nh), _nh(nh){}
template <class PointType, class NormalType>
nimbus::cloudFeatures<PointType, NormalType>::~cloudFeatures(){}

template <class PointType, class NormalType>
void nimbus::cloudFeatures<PointType, NormalType>::cloudNormalEstimation(const PointCloudConstPtr blob,
                                        NormalCloudPtr res)
{
    pcl::NormalEstimation<PointType, NormalType> ne;
    ne.setInputCloud(blob);
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);
    double searchRadius = this->computeCloudResolution(blob);
    ne.setRadiusSearch(searchRadius);
    ne.compute(*res);
}

template <class PointType, class NormalType>
void nimbus::cloudFeatures<PointType, NormalType>::cloudNormalEstimationOMP(const PointCloudConstPtr blob,
                                        pcl::PointCloud<NormalType> &res)
{
    pcl::NormalEstimationOMP<PointType, NormalType> ne;
    ne.setInputCloud(blob);
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);
    double searchRadius = this->computeCloudResolution(blob);
    ne.setRadiusSearch(searchRadius);
    ne.compute(res);
}

template <class PointType, class NormalType>
void nimbus::cloudFeatures<PointType, NormalType>::cloudSHOTEstimationOMP(const PointCloudConstPtr keyPoints,
                                        const NormalCloudConstPtr normalPoints, 
                                        const PointCloudConstPtr blob,
                                        pcl::PointCloud<pcl::SHOT352>::Ptr res)
{
    pcl::SHOTEstimationOMP<PointType, NormalType, pcl::SHOT352> descriptor;
    // double searchRadius = cEdit.computeCloudResolution(blob);
    descriptor.setNumberOfThreads(4);
    double searchRadius = this->computeCloudResolution(blob);
    descriptor.setRadiusSearch(searchRadius);
    descriptor.setInputCloud(keyPoints);
    descriptor.setInputNormals(normalPoints);
    descriptor.setSearchSurface(blob);
    descriptor.compute(*res);
}

#endif