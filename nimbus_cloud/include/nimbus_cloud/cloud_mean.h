#ifndef CLOUD_MEAN_H
#define CLOUD_MEAN_H

#include <pcl/io/impl/synchronized_queue.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

#include <nimbus_cloud/cloud_filter.h>

template <class T>
class cloudMean : public nimbus::cloudFilter<T>
{
private:
    typedef pcl::PointCloud<T> PointCloud;
    ros::NodeHandle _nh;
public:
    typedef nimbus::cloudFilter<T> cFilter; 
public:
    cloudMean(ros::NodeHandle nh);
    ~cloudMean();

    /**
     * @brief This will take mean of individual points
     * @param res 
     * @param width 
     * @param height 
     * ToDo: Implementation of voxel grid filter
     */
    void meanFilter(pcl::PointCloud<T> &res, int width, int height);
    // Variables
    pcl::SynchronizedQueue<PointCloud> cloudQueue;

};

template <class T>
cloudMean<T>::cloudMean(ros::NodeHandle nh): cFilter(nh), _nh(nh){}
template <class T>
cloudMean<T>::~cloudMean(){}

template <class T>
void cloudMean<T>::meanFilter(pcl::PointCloud<T> &res, int width, int height){
    if(cloudQueue.isEmpty()) return;
    std::vector<float> conf(width*height, 0);
    std::vector<float> mnCounter(width*height, 0);
    std::vector<float> addX(width*height, 0);
    std::vector<float> addY(width*height, 0);
    std::vector<float> addZ(width*height, 0);
    //std::vector<float> ampt(width*height, 0);
    //Point_Cloud sum;
    PointCloud tempToFilter;
    PointCloud tempC;
    res.width = width;
    res.height = height;
    while (!cloudQueue.isEmpty()){
        cloudQueue.dequeue(tempC);
        //Filter Implementation
        //this->voxelGrid(tempToFilter, tempC);
        int i = 0;
        for(int i = 0; i < tempC.points.size(); i++){
            if(!std::isnan(tempC.points[i].x)){
                addX[i] += tempC.points[i].x;
                addY[i] += tempC.points[i].y;
                addZ[i] += tempC.points[i].z;
                //ampt[i] += tempC.points[i].intensity;
                mnCounter[i] +=1;
                // ROS_INFO("X: %f, Y: %f, Z:%f AMP: %f, Mean Counter: %d", addX[i], addY[i], addZ[i], ampt[i], mnCounter[i]);
            }else{
                conf[i] = 1;
                mnCounter[i] = 0;
            }
        }
    }
    for(int i = 0; i < mnCounter.size(); i++){
        pcl::PointXYZ temPoint;
        if(mnCounter[i] == 0){
            temPoint.x = NAN;
            temPoint.y = NAN;
            temPoint.z = NAN;
            //temPoint.intensity = NAN;
        }else{
            temPoint.x = addX[i] / mnCounter[i];
            temPoint.y = addY[i] / mnCounter[i];
            temPoint.z = addZ[i] / mnCounter[i];
            //temPoint.intensity = ampt[i] / mnCounter[i];
        }
        res.points.push_back(temPoint);
    }
}

#endif