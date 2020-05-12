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
        pcl::SynchronizedQueue<pcl::PointCloud<PointType>> _queue;
    public:
        cloudUtilities();
        ~cloudUtilities();

        /**
         * @brief 
         * 
         * @param res 
         */
        void meanFilter(pcl::PointCloud<PointType> &res);
        /**
         * @brief 
         * 
         * @param blob 
         * @param width 
         * @param height 
         * @param perW 
         * @param perH 
         * @param res 
         */
        void outlineRemover(const boost::shared_ptr< const pcl::PointCloud<PointType>> blob, 
                    int width, int height, float perW, float perH,
                    pcl::PointCloud<PointType> &res);
        /**
         * @brief 
         * 
         * @param groud 
         * @param raw 
         * @param tolerence 
         * @param model 
         */
        void modelFromGroudtruth(const boost::shared_ptr< const pcl::PointCloud<PointType>> groud, 
                                 const boost::shared_ptr< const pcl::PointCloud<PointType>> raw,
                                 double tolerence,
                                 pcl::PointCloud<PointType> &model);

};

template <class PointType>
cloudUtilities<PointType>::cloudUtilities(){}
template <class PointType>
cloudUtilities<PointType>::~cloudUtilities(){}

template <class PointType>
void cloudUtilities<PointType>::meanFilter(pcl::PointCloud<PointType> &res){
    if(_queue.isEmpty()) return;
    typename pcl::PointCloud<PointType>::Ptr tempC (new pcl::PointCloud<PointType>());
    _queue.dequeue(*tempC);
    int width = tempC->width;
    int height = tempC->height;
    std::vector<float> conf(width*height, 0);
    std::vector<float> mnCounter(width*height, 0);
    std::vector<float> addX(width*height, 0);
    std::vector<float> addY(width*height, 0);
    std::vector<float> addZ(width*height, 0);
    std::vector<float> ampt(width*height, 0);
    //Point_Cloud sum;
    
    res.width = width;
    res.height = height;
    while (!_queue.isEmpty()){
        int i = 0;
        for(int i = 0; i < tempC->points.size(); i++){
            if(!std::isnan(tempC->points[i].z)){
                addX[i] += tempC->points[i].x;
                addY[i] += tempC->points[i].y;
                addZ[i] += tempC->points[i].z;
                ampt[i] += tempC->points[i].intensity;
                mnCounter[i] +=1;
            }else{
                conf[i] = 1;
                mnCounter[i] = 0;
            }
        }
        _queue.dequeue(*tempC);
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

template <class PointType>
void cloudUtilities<PointType>::outlineRemover(const boost::shared_ptr< const pcl::PointCloud<PointType>> blob, 
                    int width, int height, float perW, float perH,
                    pcl::PointCloud<PointType> &res)
{
    int hLower = (height*perH)/2;
    int hUpper = (height - hLower);
    int wLower = (width*perW)/2;
    int wUpper = width - wLower;
    int pCounter = 0;
    //res.width = width-wLower-wUpper;
    //res.height = height-hLower-hUpper;
    for(int i = 0; i < height; i++){
        std::vector<float> tempX;
        std::vector<float> tempY;
        std::vector<float> tempZ;
        std::vector<float> tempA;
        for(int j = 0; j < width; j++){
            if((i > hLower & i < hUpper) & (j > wLower & j < wUpper)){
                tempX.push_back(blob->points[pCounter].x);
                tempY.push_back(blob->points[pCounter].y);
                tempZ.push_back(blob->points[pCounter].z);
                tempA.push_back(blob->points[pCounter].intensity);
                pCounter ++;
            }else pCounter ++;
        }
        if((i > hLower & i < hUpper)){
            for(int i = 0; i < tempX.size(); i++){
                PointType temP;
                temP.x = tempX[i];
                temP.y = tempY[i];
                temP.z = tempZ[i];
                temP.intensity = tempA[i];
                res.points.push_back(temP);
            }
        }
    }
    res.width = res.points.size();
    res.height = 1;
}

template <class PointType>
void cloudUtilities<PointType>::modelFromGroudtruth(const boost::shared_ptr< const pcl::PointCloud<PointType>> groud, 
                                 const boost::shared_ptr< const pcl::PointCloud<PointType>> raw,
                                 double tolerence,
                                 pcl::PointCloud<PointType> &model)
{
    // if(groud->points.size() ! = raw->points.size())){
    //     ROS_ERROR ("Size of ground truth and raw point cloud is not matching");
    //     return;
    // }
    // typename pcl::PointCloud<pcl::PointXYZI>::Ptr edit_cloud (new pcl::PointCloud<pcl::PointXYZI>());
    // // Store the difference of z axis values;
    // std::vector<float> absZ;
    // for (int i = 0; i < groud->points.size(); i ++){
    //     if(!std::isnan(tempC->points[i].z))
    //         absZ.push_back(std::abs(groud->points[i].z - raw->points[i].z));
    //     else
    //         absZ.push_back(NAN);
    // }

    // for (int i = 0; i < absZ.size(); i ++){
    //     if(!std::isnan(absZ[i]) && absZ[i] > tolerence)
    //     {
            
    //     }
    //     else
    //         absZ.push_back(NAN);
    // }

    
    
}

#endif  //UTILITIES_H