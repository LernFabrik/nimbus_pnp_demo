#ifndef RECOGNITION_HPP
#define RECOGNITION_HPP

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
#include <pcl/features/shot.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h> 
#include <pcl/console/parse.h>

#include <kinect_detector/features.h>
#include <kinect_detector/filters.h>

namespace nimbus{
    class Recognition
    {
        private:
            std::string _path;
            ros::NodeHandle _nh;
            geometry_msgs::TransformStamped pose;
            
            tf2_ros::TransformBroadcaster tfb;
            
            ros::Publisher pubPose;
            ros::Publisher _pub;

        protected:
            pcl::PointCloud<pcl::PointXYZ>::Ptr _model;
            pcl::PointCloud<pcl::Normal>::Ptr _model_normals;
            pcl::PointCloud<pcl::PointXYZ>::Ptr _model_keypoints;
            pcl::PointCloud<pcl::SHOT352>::Ptr _model_description;
            pcl::PointCloud<pcl::ReferenceFrame>::Ptr _model_board;

            nimbus::Features<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> _features;
            pcl::CorrespondencesPtr model_scene_corr;
        public:
            Recognition(ros::NodeHandle nh, const std::string path);
            ~Recognition();
            void constructModelParam();
            void correspondences(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr blob);
            void cloudHough3D(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr blob);
            void registrationICP (const std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> instances,
                                  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene,
                                  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations, 
                                  std::vector<pcl::Correspondences> clustered_corrs);
            void hypothesisVerification(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr blob,
                                        std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> registered_instances,
                                        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations);
            void publishPose(std::vector<bool> mask, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations);
            void visualization (const int num, 
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr  scene,
                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations,
                    std::vector<pcl::Correspondences> clustered_corrs);
    };
}
#endif  //UTILITIES_H