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
#include <pcl/features/board.h>

#include <nimbus_cloud/cloud_keypoints.h>

/** 
 * http://www.pointclouds.org/documentation/tutorials/#features-tutorial
 * http://www.pointclouds.org/documentation/tutorials/how_features_work.php#rusudissertation
*/
namespace nimbus{
    template <class PointType, class NormalType>
    class cloudFeatures : public cloudKeypoints<PointType> {
        private:
            ros::NodeHandle _nh;
            typedef pcl::PointCloud<PointType> PointCloud;
            typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
            typedef pcl::PointCloud<NormalType> NormalCloud;
            typedef boost::shared_ptr<NormalCloud> NormalCloudPtr;
            typedef boost::shared_ptr<const NormalCloud> NormalCloudConstPtr;
        
        protected:
            typename pcl::PointCloud<NormalType>::Ptr normalOut;
            double normal_sr;
            typename pcl::PointCloud<pcl::SHOT352>::Ptr shotOut;
            double shot_sr;
            typename pcl::PointCloud<pcl::ReferenceFrame>::Ptr referenceOut;
            double reference_sr;

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
            void cloudNormalEstimationOMP(const PointCloudConstPtr blob);
            
            /** 
             * @brief SHOT Estimation
             * @param input KeyPoints
             * @param input Normals
             * @param input Raw cloud for surface detection
             * @param output Discriptor of type pcl::SHOT352
            */
            void cloudSHOTEstimationOMP(const PointCloudConstPtr blob);
            
            void cloudBoardLocalRefeFrame(const PointCloudConstPtr blob);

            /** ToDo:
             * 1. Normal estimation with cloud indices.
             * 2. Implememt different feature extraction algorithms.
             * 3. Change the search radius method.
            */
    };
}
template <class PointType, class NormalType>
nimbus::cloudFeatures<PointType, NormalType>::cloudFeatures(ros::NodeHandle nh):cloudKeypoints<PointType>(nh), _nh(nh){}
template <class PointType, class NormalType>
nimbus::cloudFeatures<PointType, NormalType>::~cloudFeatures(){}

template <class PointType, class NormalType>
void nimbus::cloudFeatures<PointType, NormalType>::cloudNormalEstimationOMP(const PointCloudConstPtr blob)
{
    pcl::NormalEstimationOMP<PointType, NormalType> ne;
    ne.setInputCloud(blob);
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);
    // double searchRadius = this->computeCloudResolution(blob);
    // normal_sr *= searchRadius;
    ne.setRadiusSearch(normal_sr);
    normalOut.reset(new pcl::PointCloud<NormalType>());
    ne.compute(*normalOut);
}

template <class PointType, class NormalType>
void nimbus::cloudFeatures<PointType, NormalType>::cloudSHOTEstimationOMP(const PointCloudConstPtr blob)
{
    pcl::SHOTEstimationOMP<PointType, NormalType, pcl::SHOT352> descriptor;
    descriptor.setNumberOfThreads(4);
    // double searchRadius = this->computeCloudResolution(blob);
    // shot_sr *= searchRadius;
    descriptor.setRadiusSearch(shot_sr);
    descriptor.setInputCloud(this->keypointOut);
    descriptor.setInputNormals(normalOut);
    descriptor.setSearchSurface(blob);
    shotOut.reset(new pcl::PointCloud<pcl::SHOT352>());
    descriptor.compute(*shotOut);
}

template <class PointType, class NormalType>
void nimbus::cloudFeatures<PointType, NormalType>::cloudBoardLocalRefeFrame(const PointCloudConstPtr blob)
{
    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, pcl::ReferenceFrame> rf_est;
    rf_est.setFindHoles(true);
    rf_est.setRadiusSearch(reference_sr);
    rf_est.setInputCloud(this->keypointOut);
    rf_est.setInputNormals(normalOut);
    rf_est.setSearchSurface(blob);
    referenceOut.reset(new pcl::PointCloud<pcl::ReferenceFrame>());
    rf_est.compute(*referenceOut);
}

#endif