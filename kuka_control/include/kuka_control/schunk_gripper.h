#ifndef _SCHUNK_GRIPPER_H
#define _SCHUNK_GRIPPER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>

/** ToDo: 
 * 1. Remove hard coded gripper move position
 * 2. Detect the object that gripped or not 
 *   - By reading the gripper positions
 *   - Or by calculating the effort from the gripper fingers
*/

namespace iwtros{
    class schunkGripper
    {
    protected:
        ros::NodeHandle _nh;
        actionlib::SimpleActionClient<control_msgs::GripperCommandAction> _client;
        control_msgs::GripperCommandGoal _goal;
    public:
        schunkGripper(ros::NodeHandle nh);
        ~schunkGripper();
        void closeGripper();
        void openGripper();
    };
    
    schunkGripper::schunkGripper(ros::NodeHandle nh) : _client("/iiwa/wsg50_gripper_action", true), _nh(nh){
        bool serverS = _client.waitForServer(ros::Duration(5.0));
        if(serverS) ROS_INFO("Gripper connection is established");
        else ROS_ERROR("Failed establish connection to gripper action server");
        
    }
    schunkGripper::~schunkGripper(){}

    void schunkGripper::closeGripper(){
        _goal.command.position = 0.0;
        _client.sendGoal(_goal);
        bool reached = _client.waitForResult(ros::Duration(10.0));
        if(reached) ROS_INFO("Closed Finger");
        else ROS_WARN("Failed Closed Finger");
    }

    void schunkGripper::openGripper(){
        _goal.command.position = 0.055;
        _client.sendGoal(_goal);
        bool reached = _client.waitForResult(ros::Duration(10.0));
        if(reached) ROS_INFO("Opened Finger");
        else ROS_WARN("Failed open Finger");
    }
}



#endif // !_SCHUNK_GRIPPER_H
