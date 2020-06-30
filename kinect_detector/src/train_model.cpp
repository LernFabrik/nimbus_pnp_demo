#include <iostream>
#include <thread>
#include <boost/filesystem.hpp>

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

#include <kinect_detector/utilities.h>
#include <boost/filesystem.hpp>

class ModelTraining : public cloudUtilities<pcl::PointXYZ>
{
    protected:
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr _ground;
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr _model;
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr blob;
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr blob_removed;
        bool save_point_cloud;
        ros::NodeHandle _nh;
        ros::Subscriber _sub;
        ros::Subscriber _sub_save;
        ros::Publisher _pub;
        tf2_ros::StaticTransformBroadcaster _staticTrans;
        geometry_msgs::TransformStamped _cameraPose;
        std::string _working_dir;

    public: 
        ModelTraining(ros::NodeHandle nh, std::string work_dir): _nh(nh), 
                                           save_point_cloud(false), 
                                           cloudUtilities<pcl::PointXYZ>()
        {
            _sub = _nh.subscribe<sensor_msgs::PointCloud2>("points2", 10 , boost::bind(&ModelTraining::Callback, this, _1));
            _pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("filtered_cloud", 5);
            _sub_save = nh.subscribe<std_msgs::Bool>("save_pointcloud", 10, boost::bind(&ModelTraining::saveCallback, this, _1));
            _cameraPose.child_frame_id = "detector";
            _cameraPose.header.frame_id = "camera_base";
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
            blob.reset(new pcl::PointCloud<pcl::PointXYZ>());
            blob_removed.reset(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, *blob);
            // this->outlineRemover(blob, blob->width, blob->height, 0.67, 0.67, *blob_removed); // because the point size is define in the width and height = 1 
            blob->is_dense = false;
            this->_queue.enqueue(*blob);
        }

        void saveCallback(const std_msgs::Bool::ConstPtr &msg)
        {
            save_point_cloud = msg->data;
        }

        void run()
        {
            std::string model_name = "/box_";
            std::string extention = ".pcd";
            std::string saved_groudtruth;
            int counter = 0;
            std::stringstream dir;
            _working_dir = boost::filesystem::current_path().c_str();
            dir << _working_dir << "/ros_ws/test";
            boost::filesystem::path test_dir = dir.str();
            if(!boost::filesystem::exists(test_dir) && !boost::filesystem::is_directory(test_dir))
                boost::filesystem::create_directory(test_dir);
            while(ros::ok())
            {
                unsigned int queue_size = this->_queue.size();
                if(queue_size > 2)
                {
                    _cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
                    _ground.reset(new pcl::PointCloud<pcl::PointXYZ>());
                    _model.reset(new pcl::PointCloud<pcl::PointXYZ>());
                    this->meanFilter(*_cloud);

                    ROS_INFO("Ready to save");

                    if(save_point_cloud){
                        save_point_cloud = false;
                        std::stringstream ss;
                        ss << test_dir.c_str() << model_name;
                        if (counter == 0){
                            ss << std::to_string(counter) << extention;
                            saved_groudtruth = ss.str();
                            pcl::io::savePCDFileASCII(ss.str(), *_cloud);
                            ROS_WARN("Saved PCD file path: %s", saved_groudtruth.c_str());
                            counter += 1;
                        }else{
                            ss << std::to_string(counter) << extention;
                            pcl::io::loadPCDFile(saved_groudtruth, *_ground);
                            this->modelFromGroudtruth(_ground, _cloud, 0.05, *_model);
                            pcl::io::savePCDFileASCII(ss.str(), *_model);
                            std::string saved_model_path = ss.str();
                            ROS_WARN("Saved PCD file path: %s", saved_model_path.c_str());
                            counter += 1;
                        }
                    }

                    _cloud->header.frame_id = "detector";
                    pcl_conversions::toPCL(ros::Time::now(), _cloud->header.stamp);
                    _pub.publish(_cloud);
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

    ModelTraining training(nh, argv[0]);

    try{
        training.run();
    }catch(ros::Exception e){
        ROS_ERROR("Runtime error %s", e.what());
    }
    return 0;
}