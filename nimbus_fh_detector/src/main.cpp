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
        geometry_msgs::TransformStamped pose;
        geometry_msgs::TransformStamped camera;
        tf2_ros::TransformBroadcaster tfb;
        tf2_ros::StaticTransformBroadcaster staticTF;
        ros::Publisher pubPose;
        
    public:
        Detector(ros::NodeHandle nh): _nh(nh),
                                      _util(),
                                      nimbus::Recognition(nh, "/home/vishnu/ros_ws/test")
        {
            _sub = _nh.subscribe<sensor_msgs::PointCloud2>("/nimbus/pointcloud", 10, boost::bind(&Detector::callback, this, _1));
            _pub = _nh.advertise<PointCloud>("filtered_cloud", 5);
            pubPose = nh.advertise<geometry_msgs::TransformStamped>("detected_pose", 5);
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
            _util.outlineRemover(blob, blob->width, blob->height, 0.67, 0.67, *_cloud);
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
                    
                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
                    std::vector<pcl::Correspondences> clustered_corrs;

                    this->cloudHough3D(blob, rototranslations, clustered_corrs);

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
                         ROS_WARN("Position X: %f, Y: %f Z: %f", (float)translation(0), (float)translation(1), (float)translation(2));
                        ROS_WARN("Rotation Roll: %f, Pitch: %f yaw: %f", (float)roll * (180 / M_PI), (float)pitch * (180 / M_PI), (float)yaw * (180 / M_PI));
                        pose.header.frame_id = "camera";
                        pose.child_frame_id = "object";
                        pose.header.stamp = ros::Time::now();
                        pose.transform.translation.x = translation(0);
                        pose.transform.translation.y = translation(1);
                        pose.transform.translation.z = translation(2);
                        pose.transform.rotation = tf2::toMsg(q);
                        pubPose.publish(pose);
                        tfb.sendTransform(pose);
                        camera.header.stamp = ros::Time::now();
                        staticTF.sendTransform(camera);
                        // ros::Duration(10).sleep();
                    }
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