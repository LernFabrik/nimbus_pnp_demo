#ifndef _CLOUD_KEYPOINTS_H_
#define _CLOUD_KEYPOINTS_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/uniform_sampling.h>

#include <nimbus_cloud/cloud_edit.h>

/** 
 * http://docs.pointclouds.org/trunk/group__keypoints.html
 * http://www.pointclouds.org/documentation/tutorials/#keypoints-tutorial
*/
namespace nimbus{
    template <class PointInType>
    class cloudKeypoints : public cloudEdit<PointInType>{
        private:
            ros::NodeHandle _nh;
            typedef pcl::PointCloud<PointInType> PointCloud;
            typedef boost::shared_ptr<PointCloud> PointCloudPtr;
            typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
        protected:
            typename pcl::PointCloud<PointInType>::Ptr keypointOut;
            double keypoint_sr;
        public:
            cloudKeypoints(ros::NodeHandle nh);
            ~cloudKeypoints();
            //Functions
            /** Uniform sampling Keypoints extraction (Detectors)
             * @brief http://docs.pointclouds.org/trunk/classpcl_1_1_uniform_sampling.html
             * @param Input Cloud
             * @param Output keypoint cloud
            */
           void cloudUniformSampling(const PointCloudConstPtr blob);
            
    };
}
template <class PointInType>
nimbus::cloudKeypoints<PointInType>::cloudKeypoints(ros::NodeHandle nh):cloudEdit<PointInType>(nh), _nh(nh){}
template <class PointInType>
nimbus::cloudKeypoints<PointInType>::~cloudKeypoints(){}

template <class PointInType>
void nimbus::cloudKeypoints<PointInType>::cloudUniformSampling(const PointCloudConstPtr blob){
    pcl::UniformSampling<PointInType> uniSampling;
    // double searchRadius = this->computeCloudResolution(blob);
    // this->keypoint_sr *= searchRadius;
    uniSampling.setInputCloud(blob);
    uniSampling.setRadiusSearch(keypoint_sr);
    keypointOut.reset(new pcl::PointCloud<PointInType>());
    uniSampling.filter(*keypointOut);
}

#endif