#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#include <nimbus_fh_detector/utilities.h>


class ModelTraining
{
    protected:
        pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud;
        bool save_point_cloud;
        ros::NodeHandle _nh;
        ros::Subscriber _sub;
        ros::Publisher _pub;
        pcl::SynchronizedQueue<pcl::PointCloud<pcl::PointXYZI>> _queue;
        cloudUtilities cloudUtil;

    public: 
        ModelTraining(ros::NodeHandle nh): _nh(nh), 
                                           save_point_cloud(false), 
                                           _cloud(new pcl::PointCloud<pcl::PointXYZI>()),
                                           cloudUtil()
        {
            _sub = _nh.advertise<sensor_msgs::PointCloud2>("/nimbus/pointcloud", ModelTraining::Callback, this);
        }

        void Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            pcl::PointCloud<pcl::PointXYZI> blob;
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, blob);
            _queue.enqueue(*blob);
            _cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
            if(_queue.size() > 10) cloudUtil.meanFilter(_queue, blob.width, blob.height, *_cloud);
            while(!_queue.isEmpty()) _queue.dequeue(blob);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nimbus_model_training_node");
    ros::NodeHandle nh;
}