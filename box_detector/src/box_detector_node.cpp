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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/impl/synchronized_queue.hpp>
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
        pcl::SynchronizedQueue<pcl::PointCloud<pcl::PointXYZ>> _queue;

        bool _newCloud = false;
        std::mutex cloud_lock;
        double distance_max, distance_min, per_width, per_height;

        nimbus::BoxDetector<pcl::PointXYZ> * boxDectect;

        Eigen::Matrix<float, 4, 1> centroid, param_norm;
        float yaw, curvature;
        
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

        void updateParm(ros::NodeHandle nh)
        {
            nh.getParam("distance_max", distance_max);
            nh.getParam("distance_min", distance_min);
            nh.getParam("per_width", per_width);
            nh.getParam("per_height", per_height);
        }


        void run()
        {   
            ros::spinOnce();
            ros::Rate rate(5);
            while(ros::ok())
            {  
                updateParm(this->_nh);
                if(_newCloud)
                {
                    _newCloud = false;
                    PointCloud::Ptr blob (new PointCloud());
                    PointCloud::Ptr rCloud (new PointCloud());
                    PointCloud::Ptr meanCloud (new PointCloud());
                    PointCloud::Ptr cloud (new PointCloud());

                    std::unique_lock<std::mutex> lock(cloud_lock);
                    pcl::copyPointCloud(*_cloud, *blob);
                    lock.unlock();

                    _queue.enqueue(*blob);
                    // Queue sheild
                    if(_queue.size() < 5) continue;
                    boxDectect->meanFilter(_queue, *meanCloud);
                    ROS_WARN("Queue size: %d", _queue.size());

                    //// Core Operation ////
                    boxDectect->outlineRemover(meanCloud, blob->width, blob->height, per_width, per_height, *rCloud);
                    boxDectect->zAxisLimiter(rCloud, distance_max, distance_min, *cloud);
                    
                    boxDectect->box3DCentroid(cloud, centroid);
                    boxDectect->boxYaw(cloud, yaw);
                    boxDectect->computePointNormal(cloud, param_norm, curvature);
                    ////////////////////////

                    /////    Result   ///////
                    ROS_INFO("Centroid x: %f, y:%f, z:%f", centroid[0], centroid[1], centroid[2]);
                    ROS_INFO("Yaw :%f", yaw);
                    ROS_INFO("Normal vector a:%f, b:%f, c:%f, d:%f", param_norm[0], param_norm[1], param_norm[2], param_norm[3]);
                    ROS_INFO("Curvature: %f", curvature);

                    float theta = atan(sqrt((param_norm[2] * param_norm[2]) + (param_norm[1] * param_norm[1]))/param_norm[0]);
                    float phi = atan(param_norm[2]/param_norm[1]);
                    ROS_WARN("Theta: %f and phi: %f", theta, phi);
                    /////////////////////////

                    cloud->header.frame_id = "camera";
                    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
                    _pub.publish(cloud);
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
    ros::NodeHandle nh("~");
    Detector detector(nh);
    try{
        detector.run();
    }catch(ros::Exception e){
        ROS_ERROR("Runtime error %s", e.what());
    }
    return 0;
}