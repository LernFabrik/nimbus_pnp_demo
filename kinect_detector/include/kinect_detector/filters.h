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
        public:
            Filters();
            ~Filters();
            //Functions
            /**
             * @brief Voxel grid filter
             * @param blob Input cloud
             * @result is stored in: this->_filCloud
             * @todo change the fixed parameter to dymanic so that it can be change during the run time.
             */
            void voxelGrid(const boost::shared_ptr<const pcl::PointCloud<PointType>> &blob, pcl::PointCloud<PointType> &res);
            //Functions
            /**
             * @brief Moving Least Square filter with Upsampling
             * This method is falls under surface class of pcl
             * @param blob Input cloud
             * @result is stored in: this->_filCloud
             * @todo change the fixed parameter to dymanic so that it can be change during the run time.
             * Current system do not initialized with PointXYZI
             */
            void movingLeastSquare(const boost::shared_ptr<const pcl::PointCloud<PointType>> &blob, pcl::PointCloud<PointType> &res);
            
    };
}
template <class PointType>
nimbus::Filters<PointType>::Filters(){}
template <class PointType>
nimbus::Filters<PointType>::~Filters(){}

template <class PointType>
void nimbus::Filters<PointType>::voxelGrid(const boost::shared_ptr<const pcl::PointCloud<PointType>> &blob, pcl::PointCloud<PointType> &res)
{
    pcl::VoxelGrid<PointType> vox;
    vox.setInputCloud(blob);
    // Leaf Size is 3cm
    vox.setLeafSize(0.01f, 0.01f, 0.01f);
    typename pcl::PointCloud<PointType>::Ptr filCloud (new pcl::PointCloud<PointType>());
    vox.filter(*filCloud);
    pcl::copyPointCloud(*filCloud, res);
}

template <class PointType>
void nimbus::Filters<PointType>::movingLeastSquare(const boost::shared_ptr<const pcl::PointCloud<PointType>>  &blob, pcl::PointCloud<PointType> &res)
{
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    typename pcl::PointCloud<PointType>::Ptr tempCloud (new pcl::PointCloud<PointType>());
    typename pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>());
    std::vector<int> indices;
    this->voxelGrid(blob, *tempCloud);
    pcl::removeNaNFromPointCloud(*tempCloud, *cloud, indices);
    pcl::MovingLeastSquares<PointType, PointType> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(0.03);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointType, PointType>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.01);
    mls.setUpsamplingStepSize(0.005);
    typename pcl::PointCloud<PointType>::Ptr filCloud (new pcl::PointCloud<PointType>());
    mls.process(*filCloud);
    pcl::copyPointCloud(*filCloud, res);
}

#endif