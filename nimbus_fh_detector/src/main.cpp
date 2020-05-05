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
#include <nimbus_fh_detector/features.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class Detector : public nimbus::Filters<pcl::PointXYZ>{
    private:
        ros::NodeHandle _nh; 
        ros::Subscriber _sub;
        ros::Publisher _pub;
        tf2_ros::StaticTransformBroadcaster _staticTrans;
        geometry_msgs::TransformStamped _cameraPose;
        PointCloud::Ptr _cloud;
        bool _newCloud = false;
        
        pcl::SynchronizedQueue<PointCloud::Ptr> _queue_cloud;
        nimbus::Features<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> _feature;
        
    public:
        Detector(ros::NodeHandle nh): _nh(nh), nimbus::Filters<pcl::PointXYZ>(), _feature(nh, 0.01, 0.01, 0.01) 
        {
            _sub = _nh.subscribe<sensor_msgs::PointCloud2>("/nimbus/pointcloud", 10, boost::bind(&Detector::callback, this, _1));
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

        void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            _cloud.reset(new PointCloud());
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, *_cloud);
            _cloud->is_dense = false;
            _queue_cloud.enqueue(_cloud);          
        }

        void run()
        {
            while(ros::ok())
            {
                if(!_queue_cloud.isEmpty())
                {
                    PointCloud::Ptr cloud (new PointCloud());
                    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>());
                    pcl::PointCloud<pcl::PointXYZ>::Ptr filCloud (new pcl::PointCloud<pcl::PointXYZ>());
                    while(_queue_cloud.size() > 5) _queue_cloud.dequeue(cloud);
                    _queue_cloud.dequeue(cloud);
                    pcl::copyPointCloud(*cloud, *tempCloud);
                    this->movingLeastSquare(tempCloud, *filCloud);
                    //_feature.extraction(filCloud);

                    filCloud->header.frame_id = "detector";
                    pcl_conversions::toPCL(ros::Time::now(), filCloud->header.stamp);

                    _pub.publish(filCloud);
                    ros::spinOnce();
                }else{
                    ros::spinOnce();
                }
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