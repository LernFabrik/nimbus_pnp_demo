#ifndef _CLOUD_RECOGNITION_H_
#define _CLOUD_RECOGNITION_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>



/** 
 * http://www.pointclouds.org/documentation/tutorials/#recognition-tutorial
 * http://docs.pointclouds.org/trunk/group__recognition.html
*/
namespace nimbus{
    class cloudRecognition{
        private:
            ros::NodeHandle _nh;
            typedef pcl::CorrespondencesPtr correspondencesPtr;

        public:
            cloudRecognition(ros::NodeHandle nh);
            ~cloudRecognition();
            //Functions
            /** 
             * Correspondence Matching
             * @param input downsampled cloud
             * @param input search radius
             * @param output normal cloud
            */
            void cloudCorrespondence(const pcl::PointCloud<pcl::SHOT352>::ConstPtr model_descriptor, 
                                    const pcl::PointCloud<pcl::SHOT352>::ConstPtr scene_descriptor, 
                                    correspondencesPtr corr);
                                
            void cloudHough3D();

    };
}


#endif