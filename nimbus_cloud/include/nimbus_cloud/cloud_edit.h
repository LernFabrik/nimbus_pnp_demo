#ifndef cloudEdit_H_
#define cloudEdit_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

template <class T>
class cloudEdit
{
private:
    ros::NodeHandle _nh;
public:
    cloudEdit(ros::NodeHandle nh);
    ~cloudEdit();
    void remover(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob,
                int width,
                int height,
                pcl::PointCloud<pcl::PointXYZI> &res);
};

template <class T>
cloudEdit<T>::cloudEdit(ros::NodeHandle nh) : _nh(nh){}
template <class T>
cloudEdit<T>::~cloudEdit(){}


#endif // !cloudEdit_H_
