#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <dynamic_reconfigure/server.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <pcl/filters/filter.h>
#include <pcl/io/impl/synchronized_queue.hpp>

#include <nimbus_fh_detector/filters.h>
#include <nimbus_fh_detector/utilities.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class Detector : public cloudUtilities<pcl::PointXYZI>
{
    private:
        ros::NodeHandle _nh; 
        ros::Publisher _pub;
        tf2_ros::StaticTransformBroadcaster _staticTrans;
        geometry_msgs::TransformStamped _cameraPose;
        PointCloud::Ptr _cloud;
        bool _newCloud = false;
        
        pcl::SynchronizedQueue<PointCloud::Ptr> _queue_cloud;
        
    public:
        Detector(ros::NodeHandle nh): _nh(nh), cloudUtilities<pcl::PointXYZI>() 
        {
            _pub = _nh.advertise<PointCloud>("filtered_cloud", 5);

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

        ~Detector(){}


        void run()
        {
            while(ros::ok())
            {
                PointCloud::Ptr cloud (new PointCloud());
                pcl::PointCloud<pcl::PointXYZI>::Ptr ground (new pcl::PointCloud<pcl::PointXYZI>());
                pcl::PointCloud<pcl::PointXYZI>::Ptr model (new pcl::PointCloud<pcl::PointXYZI>());
                pcl::io::loadPCDFile("/home/vishnu/ros_ws/catkin_nimbus_work/src/nimbus_cloud/test/box_0.pcd", *cloud);
                pcl::io::loadPCDFile("/home/vishnu/ros_ws/catkin_nimbus_work/src/nimbus_cloud/test/box_1.pcd", *ground);
                this->modelFromGroudtruth(ground, cloud, 0.03, *model);
                model->header.frame_id = "detector";
                pcl_conversions::toPCL(ros::Time::now(), model->header.stamp);
                _pub.publish(model);
                _staticTrans.sendTransform(_cameraPose);
            }
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "nimbus_detector_node");
    ros::NodeHandle nh;
    Detector detector(nh);
    try{
        detector.run();
    }catch(ros::Exception e){
        ROS_ERROR("Runtime error %s", e.what());
    }
    return 0;
}