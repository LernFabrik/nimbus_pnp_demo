#include <nimbus_cloud/cloud_edit.h>
#include <ros/ros.h>

template <class T>
nimbus::cloudEdit<T>::cloudEdit(ros::NodeHandle nh) : _nh(nh){}
template <class T>
nimbus::cloudEdit<T>::~cloudEdit(){}

template <class T>
void nimbus::cloudEdit<T>::remover(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr blob, 
                    int width, int height, float perW, float perH,
                    pcl::PointCloud<pcl::PointXYZI> &res){
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
                pcl::PointXYZI temP;
                temP.x = tempX[i];
                temP.y = tempY[i];
                temP.z = tempZ[i];
                temP.intensity = tempA[i];
                res.points.push_back(temP);
            }
        }
    }
}