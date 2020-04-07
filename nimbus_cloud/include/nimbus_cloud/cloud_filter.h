#ifndef _CLOUD_FILTER_H_
#define _CLOUD_FILTER_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>


namespace nimbus
{
    /**
     * @brief Implemnetation for different Filter Algorithms
     * @tparam PointInType Input Point Cloud type
     */
    template <class PointInType>
    class cloudFilter
    {
        private:
            ros::NodeHandle _nh;
            typedef pcl::PointCloud<PointInType> PointCloudType;
            typedef typename PointCloudType::Ptr PoinCloudtPtr;
            typedef typename PointCloudType::ConstPtr PointConstPtr;
        public:
            cloudFilter(ros::NodeHandle nh);
            ~cloudFilter();

            /**
             * @brief Pass through Filter
             * @param blob input point clouds
             * @param res Filteres Point cloud.
             */
            void voxelGrid(const PointCloudType blob, PointCloudType &res);
    };
}

template <class PointInType>
nimbus::cloudFilter<PointInType>::cloudFilter(ros::NodeHandle nh): _nh(nh){}

template <class PointInType>
nimbus::cloudFilter<PointInType>::~cloudFilter(){}

template <class PointInType>
void nimbus::cloudFilter<PointInType>::voxelGrid(const PointCloudType blob, PointCloudType &res)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(blob, *temp);
    pcl::VoxelGrid<pcl::PointXYZI> vox;
    vox.setInputCloud(temp);
    vox.setLeafSize(0.01f, 0.01f, 0.01f);
    vox.filter(res);
}

#endif