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

#include <nimbus_fh_detector/utilities.h>
#include <nimbus_fh_detector/recognition.hpp>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

class Detector : public nimbus::Recognition{
    private:
        ros::NodeHandle _nh; 
        ros::Subscriber _sub;
        ros::Publisher _pub;
        tf2_ros::StaticTransformBroadcaster _staticTrans;
        geometry_msgs::TransformStamped _cameraPose;
        PointCloud::Ptr _cloud;
        bool _newCloud = false;

        cloudUtilities<pcl::PointXYZI> _util;
        geometry_msgs::Transform pose;
        ros::Publisher pubPose;
        
    public:
        Detector(ros::NodeHandle nh): _nh(nh),
                                      _util(),
                                      nimbus::Recognition(nh, "/home/vishnu/ros_ws/test")
        {
            _sub = _nh.subscribe<sensor_msgs::PointCloud2>("/nimbus/pointcloud", 10, boost::bind(&Detector::callback, this, _1));
            _pub = _nh.advertise<PointCloud>("filtered_cloud", 5);
            pubPose = nh.advertise<geometry_msgs::Transform>("detected_pose", 5);

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
            pcl::PointCloud<pcl::PointXYZI>::Ptr blob (new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, *blob);
            _cloud->is_dense = false;
            _util.outlineRemover(blob, blob->width, blob->height, 0.55, 0.5, *_cloud);
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
                    PointCloud::Ptr cloud (new PointCloud());
                    pcl::copyPointCloud(*_cloud, *cloud);

                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
                    std::vector<pcl::Correspondences> clustered_corrs;

                    this->cloudHough3D(cloud, rototranslations, clustered_corrs);

                    std::cout << "Model instances found: " << rototranslations.size () << std::endl;

                    // Pub data
                    for(std::size_t i = 0; i < rototranslations.size(); ++i){
                        std::cout << "\n  Instance " << i+1 << ":" << std::endl;
                        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
                        //std::cout << "        Scale: " << scale[i] << std::endl;

                        Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0,0);
                        Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
                        tf2::Matrix3x3 mat(rotation (0,0), rotation (0,1), rotation (0,2),
                                        rotation (1,0), rotation (1,1), rotation (1,2),
                                        rotation (2,0), rotation (2,1), rotation (2,2));
                        tf2Scalar roll, pitch, yaw;
                        mat.getEulerYPR(yaw, pitch, roll);
                        tf2::Quaternion q;
                        q.setRPY(roll, pitch, yaw);
                        pose.translation.x = translation(0);
                        pose.translation.y = translation(1);
                        pose.translation.z = translation(2);
                        pose.rotation = tf2::toMsg(q);
                        pubPose.publish(pose);
                    }
                    
                    cloud->header.frame_id = "detector";
                    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);

                    _pub.publish(cloud);
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