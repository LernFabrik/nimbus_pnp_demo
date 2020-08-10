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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <box_detector/box_detector.hpp>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class Detector{
    private:
        ros::NodeHandle _nh; 
        ros::Subscriber _sub;
        ros::Publisher _pub;
        ros::Publisher _pubPose;
        PointCloud::Ptr _cloud;
        tf2_ros::Buffer buffer;
        pcl::SynchronizedQueue<pcl::PointCloud<pcl::PointXYZ>> _queue;

        bool _newCloud = false;
        std::mutex cloud_lock;
        double distance_max, distance_min, per_width, per_height, width, length, height;

        nimbus::BoxDetector<pcl::PointXYZ> * boxDectect;

        Eigen::Matrix<float, 4, 1> centroid, param_norm;
        float yaw = 0;

        tf2_ros::StaticTransformBroadcaster broadCaster;
        geometry_msgs::TransformStamped pose;

        unsigned int yawCounter;
        
    public:
        Detector(ros::NodeHandle nh): _nh(nh)
        {
            _sub = _nh.subscribe<sensor_msgs::PointCloud2>("/nimbus/pointcloud", 10, boost::bind(&Detector::callback, this, _1));
            _pub = _nh.advertise<PointCloud>("filtered_cloud", 5);
            _pubPose = _nh.advertise<geometry_msgs::TransformStamped>("detected_pose", 10);
            tf2_ros::TransformListener listener(buffer);


            this->boxDectect = new nimbus::BoxDetector<pcl::PointXYZ>(_nh);

            pose.header.frame_id = "camera";
            pose.child_frame_id = "box";
            yawCounter = 0;
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
            nh.getParam("box_width", width);
            nh.getParam("box_length", length);
            nh.getParam("box_height", height);
        }

        bool groudTruth(const boost::shared_ptr< const pcl::PointCloud<pcl::PointXYZ>> blob, pcl::PointCloud<pcl::PointXYZ> &res)
        {
            std::string model_name = "/grount_truth";
            std::string extention = ".pcd";
            std::string saved_groudtruth;
            int counter = 0;
            std::stringstream dir;
            std::string _working_dir = getenv("HOME");
            dir << _working_dir << "/ros_ws/ground";
            boost::filesystem::path test_dir = dir.str();
            if(!boost::filesystem::exists(test_dir) && !boost::filesystem::is_directory(test_dir))
            {
                ROS_INFO("------------------ Creating folder ---------------------");
                boost::filesystem::create_directory(test_dir);
                ros::Duration(1.0).sleep();
            }
            std::stringstream file;
            file << test_dir.c_str() << model_name << extention;
            if(!boost::filesystem::exists(file.str()))
            {
                ROS_INFO("------------------ For the first time scan the empty table ---------------------");
                ROS_INFO("------------------ If the table is not empty then please empty the table and RESTART ---------------------");
                ros::Duration(1.0).sleep();
                ROS_INFO("                   Saving ground truth.....");
                pcl::io::savePCDFile(file.str(), *blob);
                ROS_INFO("                   Place the box");
                return false;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
            typename pcl::PointCloud<pcl::PointXYZ>::Ptr ground (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::io::loadPCDFile(file.str(), *ground);
            bool model = boxDectect->getBaseModel(ground, blob, height - 0.02, file.str(), *cloud);
            if(!model) return false;
            pcl::copyPointCloud(*cloud, res);
            return true;
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
                    
                    boxDectect->outlineRemover(blob, blob->width, blob->height, per_width, per_height, *rCloud);
                    
                    _queue.enqueue(*rCloud);
                    // Queue sheild
                    if(_queue.size() < 2) continue;
                    
                    boxDectect->meanFilter(_queue, *meanCloud);
                    
                    bool model = groudTruth(meanCloud, *cloud);
                    if(!model) continue;
                    //// Core Operation ////
                    boxDectect->box3DCentroid(cloud, centroid);
                    if(std::isnan(centroid[0])){
                        ROS_ERROR ("Can not find the centroid");
                        continue;
                    }
                    bool calYaw = boxDectect->boxYaw(cloud, width, length, centroid, yaw);
                    ////////////////////////
                    if(calYaw)
                    {
                        // Dump first few values of yaw make it stable
                        // ToDo replace this loop burner
                        // if(yawCounter <= 5)
                        // {
                        //     yawCounter += 1;
                        //     continue;
                        // }
                        // yawCounter = 0;

                        if((yaw * 180)/M_PI > 90) yaw = yaw - M_PI;
                        if((yaw * 180)/M_PI < -90) yaw = yaw + M_PI;
                        ROS_INFO("Yaw :%f", (yaw * 180)/M_PI );
                        pose.header.stamp = ros::Time::now();
                        pose.transform.translation.x = centroid[0];
                        pose.transform.translation.y = centroid[1];
                        pose.transform.translation.z = centroid[2];
                        tf2::Quaternion q;
                        // ToDo Orientation correction instead of -ve in x and y
                        q.setRPY(0,0,yaw);
                        pose.transform.rotation = tf2::toMsg(q);
                        _pubPose.publish(pose);
                        broadCaster.sendTransform(pose);
                    }

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