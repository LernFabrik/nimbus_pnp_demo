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
 * @file box_detector_node.cpp
 * @author Vishnuprasad Prachandabhanu (vishnu.pbhat93@gmail.com)
 */
#include <iostream>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

#include <box_detector/box_detector.hpp>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class Detector{
    private:
        ros::NodeHandle _nh; 
        ros::Subscriber _sub;
        ros::Publisher _pub;
        PointCloud::Ptr _cloud;
        bool _newCloud = false;
        std::mutex cloud_lock;

        nimbus::BoxDetector<pcl::PointXYZ> * boxDectect;
        
    public:
        Detector(ros::NodeHandle nh): _nh(nh)
        {
            _sub = _nh.subscribe<sensor_msgs::PointCloud2>("/nimbus/pointcloud", 10, boost::bind(&Detector::callback, this, _1));
            _pub = _nh.advertise<PointCloud>("filtered_cloud", 5);

            this->boxDectect = new nimbus::BoxDetector<pcl::PointXYZ>(_nh);
        }

        ~Detector(){}

        void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            std::lock_guard<std::mutex> lock(cloud_lock);
            _cloud.reset(new PointCloud());
            pcl::PointCloud<pcl::PointXYZI>::Ptr blob (new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, *blob);
            _cloud->is_dense = false;
            pcl::copyPointCloud(*blob, *_cloud);
            _newCloud = true;        
        }

        void run()
        {   
            ros::spinOnce();
            ros::Rate rate(5);
            while(ros::ok())
            {  
                if(_newCloud)
                {
                    _newCloud = false;
                    PointCloud::Ptr blob (new PointCloud());

                    std::unique_lock<std::mutex> lock(cloud_lock);
                    pcl::copyPointCloud(*_cloud, *blob);
                    lock.unlock();

                    //// Core Operation ////
                    
                    ////////////////////////

                    blob->header.frame_id = "camera";
                    pcl_conversions::toPCL(ros::Time::now(), blob->header.stamp);
                    _pub.publish(blob);
                    ros::spinOnce();
                }else{
                    ros::spinOnce();
                }
                rate.sleep();
            }
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "box_detector_node");
    ros::NodeHandle nh;
    Detector detector(nh);
    try{
        detector.run();
    }catch(ros::Exception e){
        ROS_ERROR("Runtime error %s", e.what());
    }
    return 0;
}