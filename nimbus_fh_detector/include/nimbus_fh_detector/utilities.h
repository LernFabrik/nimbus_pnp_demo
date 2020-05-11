#ifndef UTILITIES_H
#define UTILITIES_H

#include <pcl/io/impl/synchronized_queue.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

template <class PointType>
class cloudUtilities 
{
    public:
        cloudUtilities();
        ~cloudUtilities();

        /**
         * @brief This will take mean of individual points
         * @param res 
         * @param width 
         * @param height 
         * ToDo: Implementation of voxel grid filter
         */
        void meanFilter(pcl::SynchronizedQueue<pcl::PointCloud<pcl::PointXYZI>> queue, int width, int height, pcl::PointCloud<PointType> &res);

};

template <class PointType>
cloudUtilities<PointType>::cloudUtilities(){}
template <class PointType>
cloudUtilities<PointType>::~cloudUtilities(){}

template <class PointType>
void cloudUtilities<PointType>::meanFilter(pcl::SynchronizedQueue<pcl::PointCloud<PointType>> queue, int width, int height, pcl::PointCloud<PointType> &res){
    if(queue.isEmpty()) return;
    std::vector<float> conf(width*height, 0);
    std::vector<float> mnCounter(width*height, 0);
    std::vector<float> addX(width*height, 0);
    std::vector<float> addY(width*height, 0);
    std::vector<float> addZ(width*height, 0);
    std::vector<float> ampt(width*height, 0);
    //Point_Cloud sum;
    typedef pcl::PointCloud<PointType> tempC;
    res.width = width;
    res.height = height;
    while (!queue.isEmpty()){
        queue.dequeue(tempC);
        //Filter Implementation
        //this->voxelGrid(tempToFilter, tempC);
        int i = 0;
        for(int i = 0; i < tempC.points.size(); i++){
            if(!std::isnan(tempC.points[i].x)){
                addX[i] += tempC.points[i].x;
                addY[i] += tempC.points[i].y;
                addZ[i] += tempC.points[i].z;
                ampt[i] += tempC.points[i].intensity;
                mnCounter[i] +=1;
            }else{
                conf[i] = 1;
                mnCounter[i] = 0;
            }
        }
    }
    for(int i = 0; i < mnCounter.size(); i++){
        PointType temPoint;
        if(mnCounter[i] == 0){
            temPoint.x = NAN;
            temPoint.y = NAN;
            temPoint.z = NAN;
            temPoint.intensity = NAN;
        }else{
            temPoint.x = addX[i] / mnCounter[i];
            temPoint.y = addY[i] / mnCounter[i];
            temPoint.z = addZ[i] / mnCounter[i];
            temPoint.intensity = ampt[i] / mnCounter[i];
        }
        res.points.push_back(temPoint);
    }
}

#endif  //UTILITIES_H