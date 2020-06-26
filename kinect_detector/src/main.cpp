#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <pcl/filters/filter.h>
#include <pcl/io/impl/synchronized_queue.hpp>

#include <nimbus_fh_detector/utilities.h>
#include <nimbus_fh_detector/recognition.hpp>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class Detector : public nimbus::Recognition{
    private:
        ros::NodeHandle _nh; 
        ros::Subscriber _sub;
        ros::Publisher _pub;
        PointCloud::Ptr _cloud;
        bool _newCloud = false;

        cloudUtilities<pcl::PointXYZI> _util;
        tf2_ros::StaticTransformBroadcaster staticTF;
        geometry_msgs::TransformStamped camera;
        
    public:
        Detector(ros::NodeHandle nh): _nh(nh),
                                      _util(),
                                      nimbus::Recognition(nh, "/home/vishnu/ros_ws/test")
        {
            _sub = _nh.subscribe<sensor_msgs::PointCloud2>("/nimbus/pointcloud", 10, boost::bind(&Detector::callback, this, _1));
            _pub = _nh.advertise<PointCloud>("filtered_cloud", 5);
            camera.header.frame_id = "iiwa_link_0";
            camera.child_frame_id = "camera";
            camera.transform.translation.x = 0.9;
            camera.transform.translation.y = 0.1;
            camera.transform.translation.z = 0.79;
            camera.transform.rotation.x = -0.6977124;
            camera.transform.rotation.y = -0.6977124;
            camera.transform.rotation.z = 0.1148801;
            camera.transform.rotation.w = 0.1148801;
        }

        ~Detector(){}

        void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            _cloud.reset(new PointCloud());
            pcl::PointCloud<pcl::PointXYZI>::Ptr blob (new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, *blob);
            _cloud->is_dense = false;
            _util.outlineRemover(blob, blob->width, blob->height, 0.65, 0.65, *_cloud);
            _util._queue.enqueue(*_cloud);
            _newCloud = true;        
        }

        void run()
        {   
            this->constructModelParam();
            ros::spinOnce();
            while(ros::ok())
            {  
                if(_newCloud)
                {
                    _newCloud = false;
                    PointCloud::Ptr blob (new PointCloud());

                    int queue_size = _util._queue.size();
                    while (!(queue_size > 5) ){
                        queue_size = _util._queue.size();
                        ros::spinOnce();
                    }
                    _util.meanFilter(*blob);
                    
                    this->cloudHough3D(blob);

                    ros::Duration(2).sleep();
                    blob->header.frame_id = "camera";
                    pcl_conversions::toPCL(ros::Time::now(), blob->header.stamp);
                    _pub.publish(blob);
                    ros::spinOnce();
                    camera.header.stamp = ros::Time::now();
                    staticTF.sendTransform(camera);
                }else{
                    camera.header.stamp = ros::Time::now();
                    staticTF.sendTransform(camera);
                    ros::spinOnce();
                }
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