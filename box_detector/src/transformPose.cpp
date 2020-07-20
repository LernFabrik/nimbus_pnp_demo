#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "detection_tranformation_node");
    ros::NodeHandle node;
    ros::Publisher turtle_vel = node.advertise<geometry_msgs::TransformStamped>("detected_goal", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("iiwa_link_0", "camera",
                                        ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::TransformStamped pose;
        pose.child_frame_id = "box";
        pose.header.frame_id = "iiwa_link_0";
        pose.header.stamp = ros::Time::now();
        pose.transform.translation.x = transformStamped.transform.translation.x;
        pose.transform.translation.y = transformStamped.transform.translation.y;
        pose.transform.translation.z = transformStamped.transform.translation.z;
        pose.transform.rotation.x = transformStamped.transform.rotation.x;
        pose.transform.rotation.y = transformStamped.transform.rotation.y;
        pose.transform.rotation.z = transformStamped.transform.rotation.z;
        pose.transform.rotation.w = transformStamped.transform.rotation.w;

        turtle_vel.publish(pose);
        rate.sleep();
    }
    return 0;
};