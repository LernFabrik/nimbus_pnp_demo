#ifndef _CLOUD_UTIL_H
#define _CLOUD_UTIL_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/range_image/range_image.h>

namespace nimbus{
    template <class T>
    class cloudEdit
    {
    private:
        ros::NodeHandle _nh;
        typedef pcl::PointCloud<T> PointCloud;
        typedef boost::shared_ptr<const PointCloud > PointCloudConstPtr;
    public:
        cloudEdit(ros::NodeHandle nh);
        ~cloudEdit();
        void remover(const PointCloudConstPtr blob, 
                    int width, int height, float perW, float perH,
                    PointCloud &res);
        void zRemover(const PointCloudConstPtr blob,
                    float maxDis, float minDis,
                    PointCloud &res);
        double computeCloudResolution(const PointCloudConstPtr cloud);
        void _toRangeImage(const PointCloudConstPtr cloud, pcl::RangeImage &range);
    };
}

// https://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
// To remove the undefined reference to the class
template class nimbus::cloudEdit<pcl::PointXYZI>;

#endif // !cloudEdit_H_
