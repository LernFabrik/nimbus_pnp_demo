#include <nimbus_cloud/cloud_util.h>
#include <ros/ros.h>

template <class T>
nimbus::cloudEdit<T>::cloudEdit(ros::NodeHandle nh) : _nh(nh){}
template <class T>
nimbus::cloudEdit<T>::~cloudEdit(){}

template <class T>
void nimbus::cloudEdit<T>::remover(const PointCloudConstPtr blob, 
                    int width, int height, float perW, float perH,
                    PointCloud &res){
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
        std::vector<float> tempA;
        for(int j = 0; j < width; j++){
            if((i > hLower & i < hUpper) & (j > wLower & j < wUpper)){
                tempX.push_back(blob->points[pCounter].x);
                tempY.push_back(blob->points[pCounter].y);
                tempZ.push_back(blob->points[pCounter].z);
                tempA.push_back(blob->points[pCounter].intensity);
                pCounter ++;
            }else pCounter ++;
        }
        if((i > hLower & i < hUpper)){
            for(int i = 0; i < tempX.size(); i++){
                T temP;
                temP.x = tempX[i];
                temP.y = tempY[i];
                temP.z = tempZ[i];
                temP.intensity = tempA[i];
                res.points.push_back(temP);
            }
        }
    }
    res.width = res.points.size();
    res.height = 1;
}

template <class T>
void nimbus::cloudEdit<T>::zRemover(const PointCloudConstPtr blob,
                    float maxDis, float minDis,
                    PointCloud &res){
    if(blob->points.empty())
        return;
    pcl::PointXYZI tempPoints;
    for(int i = 0; i < blob->points.size(); i++){
        if(maxDis > blob->points[i].z > minDis){
            tempPoints.x = blob->points[i].x;
            tempPoints.y = blob->points[i].y;
            tempPoints.z = blob->points[i].z;
            tempPoints.intensity = blob->points[i].intensity;
            res.points.push_back(tempPoints);
        } 
    }
    res.width = res.points.size();
    res.height = 1;
}

template <class T>
double nimbus::cloudEdit<T>::computeCloudResolution(const PointCloudConstPtr cloud){
    double res = 0.0;
    int _points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distance(10);
    pcl::search::KdTree<T> tree;
    tree.setInputCloud(cloud);
    
    for(std::size_t i = 0; i < cloud->points.size(); ++i){
        if(! std::isfinite(cloud->points[i].x)){
            continue;
        }
        // Search for 10 neighbors and take the max square distance
        nres = tree.nearestKSearch(i, 10, indices, sqr_distance);
        if(nres == 2 && sqr_distance[9] > 0.0001){
            res += sqrt(sqr_distance[9]);
            ++_points;
        }
    }
    if(_points != 0){
        res /= _points;
    }
    return res;
}

template <class T>
void nimbus::cloudEdit<T>::_toRangeImage(const PointCloudConstPtr cloud, pcl::RangeImage &temp_range_image)
{
    float noise_level = 0.0;
    float min_range = 0.0f;
    int boder_size = 1;
    float angular_resolution = pcl::deg2rad(1.0);
    float max_angle_width = pcl::deg2rad(66.0);
    float max_angle_height = pcl::deg2rad(54.0);
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(cloud->sensor_origin_[0],
                                                                       cloud->sensor_origin_[1],
                                                                       cloud->sensor_origin_[2]);
    pcl::RangeImage::CoordinateFrame coordinateFrame = pcl::RangeImage::CAMERA_FRAME;

    temp_range_image.createFromPointCloud(*cloud, angular_resolution, max_angle_width, 
                                          max_angle_height, sensorPose, coordinateFrame, 
                                          noise_level, min_range, boder_size);
    temp_range_image.setUnseenToMaxRange();

}