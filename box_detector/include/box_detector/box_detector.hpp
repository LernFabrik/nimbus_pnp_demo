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
 * @file box_detector.hpp
 * @author Vishnuprasad Prachandabhanu (vishnu.pbhat93@gmail.com)
 */

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <pcl/io/impl/synchronized_queue.hpp>

#include <pcl/search/search.h>
#include <pcl/common/common.h>

namespace nimbus
{
    template <class PointType>
    class BoxDetector
    {
        private:
            visualization_msgs::Marker _marker;
            uint32_t _shape = visualization_msgs::Marker::CUBE;
        protected:
            ros::NodeHandle _nh;
            ros::Publisher _pub_marker;
            EIGEN_ALIGN16 Eigen::Matrix3f _covariance_matrix;
            Eigen::Vector4f _centroid;
        public:
            BoxDetector(ros::NodeHandle nh);
            ~BoxDetector();
            /**
             * @brief Remove the points on the basis of Z Axis distace
             * @param blob Input cloud
             * @param max Maximum distance
             * @param min Minimum distance
             * @param res Resulting Cloud
             */
            void zAxisLimiter(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                              double max, double min,
                              pcl::PointCloud<pcl::PointXYZ> &res);

            void outlineRemover(const boost::shared_ptr< const pcl::PointCloud<PointType>> blob, 
                                int width, int height, float perW, float perH,
                                pcl::PointCloud<PointType> &res);

            void meanFilter(pcl::SynchronizedQueue<pcl::PointCloud<pcl::PointXYZ>> &queue, pcl::PointCloud<pcl::PointXYZ> &res);
            /**
             * @brief Compute the Least-Squares plane fit for a given set of points, using their indices,
             * and return the estimated plane parameters together with the surface curvature. 
             * @param blob the input point cloud
             * @param plane_parameters the plane parameters as: a, b, c, d (ax + by + cz + d = 0)
             * @param curvature the estimated surface curvature as a measure of f[ lambda_0 / (lambda_0 + lambda_1 + lambda_2)]
             */
            bool computePointNormal(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                    Eigen::Vector4f &plane_parameters, float &curvature);

            void box3DCentroid(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                               Eigen::Matrix<float, 4, 1> &centroid);

            unsigned int boxCovarianceMatrix (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                          const Eigen::Matrix<float, 4, 1> &centroid,
                                          Eigen::Matrix<float, 3, 3> &covariance_matrix);

            void boxCovarianceMatrixNormalized(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                               const Eigen::Matrix<float, 4, 1> &centroid,
                                               Eigen::Matrix<float, 3, 3> &covariance_matrix);
            
            unsigned int boxMeanAndCovarianceMatrix(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                                    Eigen::Matrix<float, 3, 3> &covariance_matrix,
                                                    Eigen::Matrix<float, 4, 1> &centroid);
            
            void solveBoxParameters (const Eigen::Matrix3f &covariance_matrix,
                                       const Eigen::Vector4f &centroid,
                                       Eigen::Vector4f &plane_param, float &curvature);

            void solveBoxParameters (const Eigen::Matrix3f &covariance_matrix,
                                       float &nx, float &ny, float &nz, float &curvature);
            
            void boxYaw(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob, 
                        const Eigen::Vector4f &centroid,
                        float &yaw);
    };

} // namespace nimbus