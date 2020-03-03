#ifndef _CLOUD_EDIT_H
#define _CLOUD_EDIT_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

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
                    pcl::PointCloud<T> &res);
        void zRemover(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob,
                    float maxDis, float minDis,
                    pcl::PointCloud<pcl::PointXYZI> &res);
    };
}

// https://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
// To remove the undefined reference to the class
template class nimbus::cloudEdit<pcl::PointXYZI>;

#endif // !cloudEdit_H_
