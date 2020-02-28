#ifndef CLOUD_MEAN_H
#define CLOUD_MEAN_H

#include <pcl/io/impl/synchronized_queue.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

template <class T>
class cloudMean
{
private:
    typedef pcl::PointCloud<T> PointCloud;
    ros::NodeHandle _nh;
public:
    cloudMean(ros::NodeHandle nh);
    ~cloudMean();

    void meanFilter(pcl::PointCloud<T> &res, int width, int height);
    // Variables
    pcl::SynchronizedQueue<PointCloud> cloudQueue;

};

#endif