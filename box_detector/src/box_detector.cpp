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
 * @file box_detector.cpp
 * @author Vishnuprasad Prachandabhanu (vishnu.pbhat93@gmail.com)
 */

#include <ros/ros.h>
#include <pcl/common/eigen.h>
#include <box_detector/box_detector.hpp>

template class nimbus::BoxDetector<pcl::PointXYZ>;

template <class PointType>
nimbus::BoxDetector<PointType>::BoxDetector(ros::NodeHandle nh): _nh(nh){}
template <class PointType>
nimbus::BoxDetector<PointType>::~BoxDetector(){}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class PointType>
void 
nimbus::BoxDetector<PointType>::zAxisLimiter(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                             double max, double min,
                                             pcl::PointCloud<pcl::PointXYZ> &res)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*blob, *cloud); 
    // To Carry other info
    pcl::copyPointCloud(*blob, *result);

    // Check the size of input points
    if(cloud->points.empty())
    {
        ROS_ERROR("Input Point Cloud is empty");
        return;
    }

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if((max >= cloud->points[i].z) && (min <= cloud->points[i].z))
        {
            pcl::copyPoint(cloud->points[i], result->points[i]);
        }else{
            result->points[i].x = NAN;
            result->points[i].y = NAN;
            result->points[i].y = NAN;
        }
    }

    // Copy to result
    pcl::copyPointCloud(*result, res);
}

template <class PointType>
void nimbus::BoxDetector<PointType>::outlineRemover(const boost::shared_ptr< const pcl::PointCloud<PointType>> blob, 
                    int width, int height, float perW, float perH,
                    pcl::PointCloud<PointType> &res)
{
    int hLower = (height*perH)/2;
    int hUpper = (height - hLower);
    int wLower = (width*perW)/2;
    int wUpper = width - wLower;
    int pCounter = 0;
    //res.width = width-wLower-wUpper;
    //res.height = height-hLower-hUpper;
    for(int i = 0; i < height; i++){
        std::vector<float> tempX;
        std::vector<float> tempY;
        std::vector<float> tempZ;
        for(int j = 0; j < width; j++){
            if((i > hLower & i < hUpper) & (j > wLower & j < wUpper)){
                tempX.push_back(blob->points[pCounter].x);
                tempY.push_back(blob->points[pCounter].y);
                tempZ.push_back(blob->points[pCounter].z);
                pCounter ++;
            }else pCounter ++;
        }
        if((i > hLower & i < hUpper)){
            for(int i = 0; i < tempX.size(); i++){
                PointType temP;
                temP.x = tempX[i];
                temP.y = tempY[i];
                temP.z = tempZ[i];
                res.points.push_back(temP);
            }
        }
    }
    res.header   = blob->header;
    res.is_dense = blob->is_dense;
    res.sensor_orientation_ = blob->sensor_orientation_;
    res.sensor_origin_ = blob->sensor_origin_;
}


template <class PointType>
bool 
nimbus::BoxDetector<PointType>::computePointNormal(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                                   Eigen::Vector4f &plane_parameters, float &curvature)
{
    if(blob->points.size() < 3 ||
       boxMeanAndCovarianceMatrix(blob, _covariance_matrix, _centroid) == 0)
    {
        plane_parameters.setConstant(std::numeric_limits<float>::quiet_NaN());
        curvature = std::numeric_limits<float>::quiet_NaN();
        return false;
    }
    // Get the plane normal and surface curvature
    solveBoxParameters(_covariance_matrix, _centroid, plane_parameters, curvature);
    return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class PointType>
void 
nimbus::BoxDetector<PointType>::box3DCentroid(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                              Eigen::Matrix<float, 4, 1> &centroid)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::copyPointCloud(*blob, cloud);
    if(cloud.points.empty())
    {
        ROS_ERROR("Input Point Cloud is empty");
        return;
    }
    if(cloud.is_dense)
    {
        ROS_ERROR_NAMED("Covariance Matrix", "!Dense Cloud is Not Acceptable!");
        return;
    }

    // Initialize centroid to zero
    centroid.setZero();
    // Cloud Dense is false and contain NAN points
    unsigned cp = 0;
    for(const auto& point: cloud)
    {
        if(!pcl::isFinite(point))
            continue;
        
        centroid[0] += point.x;
        centroid[1] += point.y;
        centroid[2] += point.z;
        ++cp;
    }
    centroid /= static_cast<float>(cp);
    centroid[3] = 1;
    ROS_DEBUG_NAMED("Computed 3D Centroid","Calculated Points: %f", static_cast<float>(cp));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class PointType>
inline unsigned int 
nimbus::BoxDetector<PointType>::boxCovarianceMatrix (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                          const Eigen::Matrix<float, 4, 1> &centroid,
                                          Eigen::Matrix<float, 3, 3> &covariance_matrix)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::copyPointCloud(*blob, cloud);
    if(cloud.points.empty())
    {
        ROS_ERROR("Input Point Cloud is empty");
        return 0;
    }
    if(cloud.is_dense)
    {
        ROS_ERROR_NAMED("Covariance Matrix", "!Dense Cloud is Not Acceptable!");
        return 0;
    }

    // Initialize centroid to zero
    covariance_matrix.setZero();

    // Cloud Dense is false and contain NAN points
    unsigned cp = 0;
    for (const auto& point: cloud)
    {
        if(!pcl::isFinite(point))
            continue;
        
        Eigen::Matrix<float, 4, 1> pt;
        pt[0] = point.x - centroid[0];
        pt[1] = point.y - centroid[1];
        pt[2] = point.z - centroid[2];

        covariance_matrix (1, 1) += pt.y() * pt.y();
        covariance_matrix (1, 2) += pt.y() * pt.z();
        covariance_matrix (2, 2) += pt.z() * pt.z();
        pt *= pt.x();
        covariance_matrix (0, 0) += pt.x();
        covariance_matrix (0, 1) += pt.y();
        covariance_matrix (0, 2) += pt.z();
        ++ cp;
    }
    covariance_matrix (1, 0) = covariance_matrix (0, 1);
    covariance_matrix (2, 0) = covariance_matrix (0, 2);
    covariance_matrix (2, 1) = covariance_matrix (1, 2);

    ROS_DEBUG_NAMED("Computed Covariance Matrix","Calculated Points: %f", static_cast<float>(cp));
    return (cp);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class PointType>
void 
nimbus::BoxDetector<PointType>::boxCovarianceMatrixNormalized(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                               const Eigen::Matrix<float, 4, 1> &centroid,
                                               Eigen::Matrix<float, 3, 3> &covariance_matrix)
{
    unsigned point_count = this->boxCovarianceMatrix(blob, centroid, covariance_matrix);
    if(point_count != 0)
        covariance_matrix /= static_cast<float>(point_count);
    
    ROS_DEBUG_NAMED("Computed Covariance Matrix Normalized","Calculated Points: %f", static_cast<float>(point_count));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class PointType>
inline unsigned int 
nimbus::BoxDetector<PointType>::boxMeanAndCovarianceMatrix (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob,
                                                            Eigen::Matrix<float, 3, 3> &covariance_matrix,
                                                            Eigen::Matrix<float, 4, 1> &centroid)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::copyPointCloud(*blob, cloud);
    if(cloud.points.empty())
    {
        ROS_ERROR("Input Point Cloud is empty");
        return 0;
    }
    if(cloud.is_dense)
    {
        ROS_ERROR_NAMED("Covariance Matrix", "!Dense Cloud is Not Acceptable!");
        return 0;
    }

    Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero();
    std::size_t point_count = 0;
    
    for(const auto& point: cloud)
    {
        if(!pcl::isFinite(point))
            continue;
        
        accu[0] += point.x * point.x;
        accu[1] += point.x * point.y;
        accu[2] += point.x * point.z;
        accu[3] += point.y * point.y;
        accu[4] += point.y * point.z;
        accu[5] += point.z * point.z;
        accu[6] += point.x;
        accu[7] += point.y;
        accu[8] += point.z;
        ++ point_count;
    } 

    accu /= static_cast<float>(point_count);
    if(point_count != 0)
    {
        centroid[0] = accu[6];
        centroid[1] = accu[7];
        centroid[2] = accu[8];
        centroid[3] = 1;

        covariance_matrix.coeffRef(0) = accu[0] - accu[6] * accu[6];
        covariance_matrix.coeffRef(1) = accu[1] - accu[6] * accu[7];
        covariance_matrix.coeffRef(2) = accu[2] - accu[6] * accu[8];
        covariance_matrix.coeffRef(4) = accu[3] - accu[7] * accu[7];
        covariance_matrix.coeffRef(5) = accu[4] - accu[7] * accu[8];
        covariance_matrix.coeffRef(8) = accu[5] - accu[8] * accu[8];
        covariance_matrix.coeffRef(3) = covariance_matrix.coeff(1);
        covariance_matrix.coeffRef(6) = covariance_matrix.coeff(2);
        covariance_matrix.coeffRef(7) = covariance_matrix.coeff(5);
    }
    ROS_DEBUG_NAMED("Computed Mean Covariance Matrix","Calculated Points: %f", static_cast<float>(point_count));
    return (point_count);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class PointType>
void
nimbus::BoxDetector<PointType>::solveBoxParameters (const Eigen::Matrix3f &covariance_matrix,
                                                    const Eigen::Vector4f &centroid,
                                                    Eigen::Vector4f &plane_param, float &curvature)
{
    solveBoxParameters(covariance_matrix, plane_param[0], plane_param[1], plane_param[2], curvature);

    plane_param[3] = 0;
    // Hessian form (D = nc . p_plane (centroid here) + p)
    plane_param[3] = -1 * plane_param.dot(centroid);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class PointType>
void 
nimbus::BoxDetector<PointType>::solveBoxParameters (const Eigen::Matrix3f &covariance_matrix,
                                       float &nx, float &ny, float &nz, float &curvature)
{
    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
    pcl::eigen33(covariance_matrix, eigen_value, eigen_vector);

    nx = eigen_vector[0];
    ny = eigen_vector[1];
    nz = eigen_vector[2];

    // Compute the surface curvature
    float eigen_sum = covariance_matrix.coeff(0) + covariance_matrix.coeff(4) + covariance_matrix.coeff(8);
    if(eigen_sum != 0)
        curvature = std::abs(eigen_value / eigen_sum);
    else
        curvature = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class PointType>
void 
nimbus::BoxDetector<PointType>::boxYaw(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &blob, 
                                       float &yaw)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::copyPointCloud(*blob, cloud);

    //unsigned int Xmin_indice = 0, Xmax_indice = 0, Ymin_indice = 0, Ymax_indice = 0;
    float Xmax, Xmin, X_yMax, X_yMin;
    float Ymax, Ymin, Y_xMax, Y_xMin;

    float raw_yaw = 0;

    size_t i = 0;
    Xmax = Xmin = cloud.points[i].x;
    Ymax = Ymin = cloud.points[i].y;
    i += 1;
    while (std::isnan(Xmax))
    {
        Xmax = Xmin = cloud.points[i].x;
        Ymax = Ymin = cloud.points[i].y;
        i += 1;
    }

    // Calculate Maximum and Minimum values in X and Y axis
    for(size_t j = i; j < cloud.points.size(); ++j)
    {
        pcl::PointXYZ point = cloud.points[j];
        if(!pcl::isFinite(point)) continue;

        if(cloud.points[j].x < Xmin)
        {
            Xmin = cloud.points[j].x;
            // Xmin_indice = static_cast<int>(i);
            Y_xMin = cloud.points[j].y;
        }
        if(cloud.points[j].x > Xmax)
        {
            Xmax = cloud.points[j].x;
            // Xmax_indice = static_cast<int>(i);
            Y_xMax =cloud.points[j].y;
        }
        if(cloud.points[j].y < Ymin)
        {
            Ymin = cloud.points[j].y;
            // Ymin_indice = static_cast<int>(i);
            X_yMin = cloud.points[j].x;
        }
        if(cloud.points[j].y > Ymax)
        {
            Ymax = cloud.points[j].y;
            // Ymax_indice = static_cast<int>(i);
            X_yMax = cloud.points[j].x;
        }
    }

    raw_yaw = atan((Ymax - Y_xMin) / (X_yMax - Xmin));

    yaw = raw_yaw;  
}