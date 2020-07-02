#include <iostream>
#include <thread>
#include <pthread.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

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

void disable_canonical(){
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
}
void enable_canonical(){
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
}
bool key_pressed(char c){
    struct pollfd fds[1];
    fds[0].fd = 0;
    fds[0].events = POLLIN;
    fds[0].revents = 0;
    int r = poll(fds, 1, 1);
    if(r > 0){
        if(fds[0].revents & POLLIN || fds[0].revents & POLLRDBAND || fds[0].revents & POLLRDNORM){
            char buffer[1024];
            memset(buffer, 0, sizeof(buffer));
            int n = read(0, buffer, sizeof(buffer) -1);
            for(int i = 0; i < n; ++i){
                if(buffer[i] == c){
                    return true;
                }
            }
        }
    }
    return false;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "model_test_node");
    ros::NodeHandle nh("~");

    nh.getParam("size_x", size_x);
    nh.getParam("size_y", size_y);
    nh.getParam("size_z", size_z);
    nh.getParam("px", px);
    nh.getParam("py", py);
    nh.getParam("pz", pz);
    nh.getParam("roll", roll);
    nh.getParam("pitch", pitch);
    nh.getParam("yaw", yaw);

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("model_point", 5);
    pcl::PointCloud<pcl::PointXYZ>::Ptr blob (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr blob_trasform (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr blob_orien (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

    // Load Point for perticular location

    pcl::io::loadPCDFile("/home/vishnu/ros_ws/test/box.pcd", *blob);

    pcl::transformPointCloud(*blob, *blob_trasform, Eigen::Vector3f(-size_x/2, -size_y/2, -size_z/2), Eigen::Quaternionf(1, 0, 0, 0));
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    pcl::transformPointCloud(*blob_trasform, *blob_orien, Eigen::Vector3f(0 , 0, 0), Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z()));
    pcl::transformPointCloud(*blob_orien, *cloud, Eigen::Vector3f(px, py, pz), Eigen::Quaternionf(1, 0, 0, 0));

    cloud->header.frame_id = "model";
    ros::Rate r(1);

    // pthread_t key_thread;
    // (void) pthread_create(&key_thread, 0, user_thread, 0);

    while (ros::ok() && !save_cloud)
    {
        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        pub.publish(cloud);
        r.sleep();
        //TODO Save upon key stroke
        if(key_pressed('x'))
        {
            ROS_INFO("Saving Point Cloud");
            pcl::io::savePCDFile("/home/vishnu/ros_ws/test/box_modif.pcd", *cloud);
        }
    }

    // (void) pthread_join(key_thread, NULL);
    // ROS_INFO("Saving Point Cloud");
    // pcl::io::savePCDFile("/home/vishnu/ros_ws/test/box_modif.pcd", *cloud);
    return 0;
}