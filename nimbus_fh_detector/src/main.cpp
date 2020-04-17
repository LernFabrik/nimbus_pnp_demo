#include <iostream>
#include <thread>

#include <ros/ros.h>
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

#include <nimbus_fh_detector/filters.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class Detector : public nimbus::Filters<PointType>{
    private:
        ros::NodeHandle _nh; 
        ros::Subscriber _sub;
        ros::Publisher _pub;
        tf2_ros::StaticTransformBroadcaster _staticTrans;
        geometry_msgs::TransformStamped _cameraPose;
        PointCloud::Ptr _cloud;
        bool _newCloud = false;

    public:
        Detector(ros::NodeHandle nh): _nh(nh), nimbus::Filters<PointType>(nh)
        {
            _sub = _nh.subscribe<pcl::PointCloud<PointType>>("/nimbus/pointcloud", 10, boost::bind(&Detector::callback, this, _1));
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

        void callback(const PointCloud::ConstPtr& msg)
        {
            _cloud.reset(new PointCloud());
            pcl::copyPointCloud(*msg, *_cloud);
            _newCloud = true;
        }

        void run()
        {
            while(ros::ok())
            {
                if(_newCloud && !_cloud->empty())
                {
                    this->movingLeastSquare(_cloud);
                    this->_filCloud->header.frame_id = "detector";
                    pcl_conversions::toPCL(ros::Time::now(), this->_filCloud->header.stamp);
                    _pub.publish(this->_filCloud);
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