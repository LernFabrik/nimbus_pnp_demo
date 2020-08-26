#ifndef _IIWA_MANIPULATION_H
#define _IIWA_MANIPULATION_H

#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#include <kuka_control/schunk_gripper.h>

#include <iwtros_msgs/plcControl.h>
#include <iwtros_msgs/kukaControl.h>

namespace iwtros{
    class iiwaMove : public schunkGripper
    {
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _sub;
        ros::Subscriber _accpSub;
        ros::Publisher _plcPub;
        ros::Subscriber _plcSub;
        bool _initialized = false;
        moveit::planning_interface::MoveGroupInterface move_group;
        tf2_ros::Buffer buffer;
        // moveit parameters
        /** ToDo:
         * Currently parameters are hard coded.
         * change this in the future
         */
        std::string PLANNER_ID;
        std::string PLANNING_GROUP;
        std::string REFERENCE_FRAME;
        std::string EE_FRAME;
        double velocityScalling;
        double accelerationScalling;
        geometry_msgs::Transform detected_pose;
        geometry_msgs::PoseStamped pick_pose;
        bool ready_pick_pose, _accept_pose;
        iwtros_msgs::plcControl _plcSubscriberControl; 
        iwtros_msgs::kukaControl _plcKUKA; 

    public:
        iiwaMove(ros::NodeHandle nh, const std::string planning_group);
        ~iiwaMove();
        void _loadParam();
        void init(ros::NodeHandle nh);

         /** PLC Control Callback*/ 
        void plcCallback(const iwtros_msgs::plcControl::ConstPtr& data);

        /** Detected Pose Callback*/ 
        void callback(const geometry_msgs::TransformStamped::ConstPtr& data);
        void acceptCallback(const std_msgs::Bool::ConstPtr &data);

        /** Return geometry pose from given poisition values*/
        void generateMeanPose(const geometry_msgs::Transform detection);
        void generatePickPose(const geometry_msgs::Transform detection, std::string base_link);

         /** Return geometry pose from given poisition values*/
        geometry_msgs::PoseStamped generatePose(double x, double y, double z,
                                                double roll, double pitch, double yaw,
                                                std::string base_link);

        /** Genarate Motion contraints for Pilz industrial motion*/
        void motionContraints(const geometry_msgs::PoseStamped pose);
        
        /** Motion execution pipe line */
        void motionExecution(const geometry_msgs::PoseStamped pose);
        
        /** Pick and Place Pipeline */
        void pnpPipeLine(geometry_msgs::PoseStamped pick,
                        geometry_msgs::PoseStamped place,
                        const double offset);

        /** Rviz visual marker*/
        void visualMarkers(const geometry_msgs::PoseStamped target_pose,
                            moveit::planning_interface::MoveGroupInterface::Plan plan);

        /** Main Execution */
        void run();
        void _ctrl_loop();
        void _tf_listner_loop();

    };
}


#endif // !_IIWA_MANIPULATION_H
