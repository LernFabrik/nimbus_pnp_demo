/**
 * MIT License
 * 
 * Copyright (c) 2020 IWT Wirtschaft und Technik GmbH
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file box_detector.cpp
 * @author Vishnuprasad Prachandabhanu (vishnu.pbhat93@gmail.com)
 */

#include <ros/ros.h>
#include <box_detector/box_detector.hpp>

template class nimbus::BoxDetector<pcl::PointXYZ>;

template <class PointType>
nimbus::BoxDetector<PointType>::BoxDetector(ros::NodeHandle nh): _nh(nh){}
template <class PointType>
nimbus::BoxDetector<PointType>::~BoxDetector(){}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class PointType>
void 
nimbus::BoxDetector<PointType>::zAxisLimiter(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                             double max, double min,
                                             pcl::PointCloud<pcl::PointXYZ> &res)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*blob, *cloud); 
    // To Carry other info
    pcl::copyPointCloud(*blob, *result);

    // Check the size of input points
    if(cloud->points.empty())
    {
        ROS_ERROR("Input Point Cloud is empty");
        return;
    }

    for (size_t i; i < cloud->points.size(); ++i)
    {
        if((max >= cloud->points[i].z) && (min <= cloud->points[i].z))
        {
            pcl::copyPoint(cloud->points[i], result->points[i]);
        }else{
            result->points[i].x = NAN;
            result->points[i].y = NAN;
            result->points[i].y = NAN;
        }
    }

    // Copy to result
    pcl::copyPointCloud(*result, res);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class PointType>
void 
nimbus::BoxDetector<PointType>::box3DCentroid(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                              Eigen::Matrix<double, 4, 1> &centroid)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::copyPointCloud(*blob, cloud);
    if(cloud.points.empty())
    {
        ROS_ERROR("Input Point Cloud is empty");
        return;
    }

    // Initialize centroid to zero
    centroid.setZero();
    // Cloud Dense is false and contain NAN points
    unsigned cp = 0;
    for(const auto& point: cloud)
    {
        if(!pcl::isFinite(point))
            continue;
        
        centroid[0] += point.x;
        centroid[1] += point.y;
        centroid[2] += point.z;
        ++cp;
    }
    centroid /= static_cast<double>(cp);
    centroid[3] = 1;
    ROS_DEBUG_NAMED("Computed 3D Centroid","Calculated Points: %f", static_cast<double>(cp));
}