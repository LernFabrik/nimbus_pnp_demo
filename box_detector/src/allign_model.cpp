/**
 * MIT License
 * 
 * Copyright (c) 2020 IWT Wirtschaft und Technik GmbH
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file alligne_model.cpp
 * @author Vishnuprasad Prachandabhanu (vishnu.pbhat93@gmail.com)
 */

#include <iostream>
#include <thread>
#include <pthread.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

#include <pcl/io/vtk_io.h>
#include <vtkPolyDataMapper.h>
#include <pcl/apps/render_views_tesselated_sphere.h>

double size_x, size_y, size_z, px, py, pz, roll, pitch, yaw;
static volatile bool save_cloud = false;

static void* user_thread(void*)
{
    while (!save_cloud && ros::ok())
    {
        if(std::cin.get() == 'q') 
        {
            save_cloud = true;
        }
        
    }    
}

double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}

void updateParm(ros::NodeHandle nh)
{
    nh.getParam("size_x", size_x);
    nh.getParam("size_y", size_y);
    nh.getParam("size_z", size_z);
    nh.getParam("px", px);
    nh.getParam("py", py);
    nh.getParam("pz", pz);
    nh.getParam("roll", roll);
    nh.getParam("pitch", pitch);
    nh.getParam("yaw", yaw);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "model_test_node");
    ros::NodeHandle nh("~");

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("model_point", 5);
    pcl::PointCloud<pcl::PointXYZ>::Ptr blob (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr blob_trasform (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr blob_orien (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

    // Load Point for perticular location

    

    ros::Rate r(1);

    // pthread_t key_thread;
    // (void) pthread_create(&key_thread, 0, user_thread, 0);

    while (ros::ok() && !save_cloud)
    {
        updateParm(nh);
        pcl::io::loadPCDFile("/home/vishnu/ros_ws/test/box.pcd", *blob);
        pcl::transformPointCloud(*blob, *blob_trasform, Eigen::Vector3f(-size_x/2, -size_y/2, -size_z/2), Eigen::Quaternionf(1, 0, 0, 0));
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pcl::transformPointCloud(*blob_trasform, *blob_orien, Eigen::Vector3f(0 , 0, 0), Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z()));
        pcl::transformPointCloud(*blob_orien, *cloud, Eigen::Vector3f(px, py, pz), Eigen::Quaternionf(1, 0, 0, 0));

        cloud->header.frame_id = "model";
        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        pub.publish(cloud);

        // pcl::apps::RenderViewsTesselatedSphere rendered_view
        r.sleep();
        //TODO Save upon key stroke
        ROS_INFO("Saving Point Cloud");
        pcl::io::savePCDFile("/home/vishnu/ros_ws/test/box_modif2.pcd", *cloud);
    }
    // (void) pthread_join(key_thread, NULL);
    // ROS_INFO("Saving Point Cloud");
    // pcl::io::savePCDFile("/home/vishnu/ros_ws/test/box_modif.pcd", *cloud);
    return 0;
}