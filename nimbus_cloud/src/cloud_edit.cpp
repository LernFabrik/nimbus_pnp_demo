#include <nimbus_cloud/cloud_edit.h>

template <class T>
void cloudEdit<T>::remover(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob,
                int width,
                int height,
                pcl::PointCloud<pcl::PointXYZI> &res){
    ROS_INFO("Input Widht: %d, Height: %d", width, height);
}