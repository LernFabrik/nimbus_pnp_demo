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

template <class PointType>
void 
nimbus::BoxDetector<PointType>::zAxisLimiter(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                             double max, double min,
                                             pcl::PointCloud<pcl::PointXYZ> &res)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr result (new pcl::PointCloud<pcl::PointXYZI>());
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