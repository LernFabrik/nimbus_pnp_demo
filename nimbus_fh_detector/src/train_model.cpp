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


class ModelTraining : public cloudUtilities<pcl::PointXYZI>
{
    protected:
        typename pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud;
        bool save_point_cloud;
        ros::NodeHandle _nh;
        ros::Subscriber _sub;
        ros::Publisher _pub;
        tf2_ros::StaticTransformBroadcaster _staticTrans;
        geometry_msgs::TransformStamped _cameraPose;

    public: 
        ModelTraining(ros::NodeHandle nh): _nh(nh), 
                                           save_point_cloud(false), 
                                           cloudUtilities<pcl::PointXYZI>()
        {
            _sub = _nh.subscribe<sensor_msgs::PointCloud2>("/nimbus/pointcloud", 10 , boost::bind(&ModelTraining::Callback, this, _1));
            _pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("filtered_cloud", 5);
            _cameraPose.child_frame_id = "detector";
            _cameraPose.header.frame_id = "world";
            _cameraPose.transform.translation.x = 0;
            _cameraPose.transform.translation.y = 0;
            _cameraPose.transform.translation.z = 1;
            tf2::Quaternion q;
            q.setRPY(-1.57, 1.57, 0);
            _cameraPose.transform.rotation.x = q.x();
            _cameraPose.transform.rotation.y = q.y();
            _cameraPose.transform.rotation.z = q.z();
            _cameraPose.transform.rotation.w = q.w();
        }

        void Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            pcl::PointCloud<pcl::PointXYZI> blob;
            pcl::PointCloud<pcl::PointXYZI>::Ptr blob_removed (new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, blob);
            this->outlineRemover(blob, blob.width, blob.height, 0.82, 0.8, *blob_removed);
            this->_queue.enqueue(*blob_removed);
        }

        void run()
        {
            while(ros::ok())
            {
                if(this->_queue.size() > 10)
                {
                    _cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
                    this->meanFilter(*_cloud);
                }
                _staticTrans.sendTransform(_cameraPose);
                ros::spinOnce();
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nimbus_model_training_node");
    ros::NodeHandle nh;

    ModelTraining training(nh);

    try{
        training.run();
    }catch(ros::Exception e){
        ROS_ERROR("Runtime error %s", e.what());
    }
    return 0;
}