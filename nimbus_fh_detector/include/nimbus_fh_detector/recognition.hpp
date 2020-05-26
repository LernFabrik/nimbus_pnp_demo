#ifndef RECOGNITION_HPP
#define RECOGNITION_HPP

    #ifdef __cplusplus
    extern "C"
    {
    #endif

    #include <ros/ros.h>
    #include <pcl_ros/point_cloud.h>
    #include <pcl_conversions/pcl_conversions.h>
    #include <pcl/io/pcd_io.h>
    #include <boost/foreach.hpp>

    #include <pcl/kdtree/kdtree_flann.h>
    #include <pcl/kdtree/impl/kdtree_flann.hpp>
    #include <pcl/correspondence.h>
    #include <pcl/features/shot.h>
    
    #include <nimbus_fh_detector/features.h>
    #include <nimbus_fh_detector/filters.h>
    
    namespace nimbus{
        class Recognition
        {
            private:
                std::string _path;
                ros::NodeHandle _nh;
            protected:
                std::vector<pcl::PointCloud<pcl::Normal>::Ptr> _model_normals;
                std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _model_keypoints;
                std::vector<pcl::PointCloud<pcl::SHOT352>::Ptr> _model_description;
                nimbus::Features<pcl::PointXYZI, pcl::Normal, pcl::SHOT352> _features;
            public:
                Recognition(ros::NodeHandle nh, const std::string path);
                ~Recognition();
                void constructModelParam();
                void correspondences(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob);
                void cloudHough3D(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob,
                                  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations, 
                                  std::vector<pcl::Correspondences> &clustered_corrs);
        };
    }

    #ifdef __cplusplus
    } // extern "C"
   #endif
#endif  //UTILITIES_H