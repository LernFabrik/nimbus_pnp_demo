#ifndef _FILTERS_H_
#define _FILTERS_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>


/** 
 * http://www.pointclouds.org/documentation/tutorials/#features-tutorial
 * http://www.pointclouds.org/documentation/tutorials/how_features_work.php#rusudissertation
*/
namespace nimbus{
    /**
     * @brief 
     * 
     * @tparam PointType 
     */
    template <class PointType>
    class Filters{
        private:
            ros::NodeHandle _nh;
            typedef pcl::PointCloud<PointType> PointCloudType;
            typedef typename PointCloudType::Ptr PointCloudTypePtr;
            typedef typename PointCloudType::ConstPtr PointCloudTypeConstPtr;
        // Variables
        public:
            typename pcl::PointCloud<PointType>::Ptr _filCloud;
            typename pcl::search::KdTree<PointType>::Ptr kdTree;
        
        public:
            Filters(ros::NodeHandle nh);
            ~Filters();
            //Functions
            /**
             * @brief Voxel grid filter
             * @param blob Input cloud
             * @result is stored in: this->_filCloud
             * @todo change the fixed parameter to dymanic so that it can be change during the run time.
             */
            void voxelGrid(const PointCloudTypeConstPtr blob);
            //Functions
            /**
             * @brief Moving Least Square filter with Upsampling
             * This method is falls under surface class of pcl
             * @param blob Input cloud
             * @result is stored in: this->_filCloud
             * @todo change the fixed parameter to dymanic so that it can be change during the run time.
             * Current system do not initializred with PointXYZI
             */
            template <typename PointIn, typename PointOut>
            void movingLeastSquare(const PointCloudTypePtr &blob, pcl::PointCloud<pcl::PointNormal> &res);
            
    };
}
template <class PointType>
nimbus::Filters<PointType>::Filters(ros::NodeHandle nh): _nh(nh){}
template <class PointType>
nimbus::Filters<PointType>::~Filters(){}

template <class PointType>
void nimbus::Filters<PointType>::voxelGrid(const PointCloudTypeConstPtr blob)
{
    pcl::VoxelGrid<PointType> vox;
    vox.setInputCloud(blob);
    // Leaf Size is 3cm
    vox.setLeafSize(0.03f, 0.03f, 0.03f);
    _filCloud.reset(new pcl::PointCloud<PointType>());
    vox.filter(*_filCloud);
}

template <class PointType>
template <typename PointIn, typename PointOut>
void nimbus::Filters<PointType>::movingLeastSquare(const PointCloudTypePtr &blob, pcl::PointCloud<pcl::PointNormal> &res)
{
    typename pcl::search::KdTree<PointIn>::Ptr tree (new pcl::search::KdTree<PointIn>);
    typename pcl::PointCloud<PointIn>::Ptr cloud (new pcl::PointCloud<PointIn>());
    std::vector<int> indices; 
    pcl::removeNaNFromPointCloud(*blob, *cloud, indices);
    pcl::MovingLeastSquares<PointIn, PointOut> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(0.01);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.005);
    mls.setUpsamplingStepSize(0.003);
    pcl::PointCloud<pcl::PointNormal>::Ptr filCloud (new pcl::PointCloud<pcl::PointNormal>());
    mls.process(*filCloud);
    pcl::copyPointCloud(*filCloud, res);
}

#endif