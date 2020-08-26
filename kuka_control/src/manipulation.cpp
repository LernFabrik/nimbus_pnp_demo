#include <kuka_control/iiwa_manipulation.h>
#include <thread>
#include <iostream>

#include <moveit_msgs/Constraints.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Transform.h>

iwtros::iiwaMove::iiwaMove(ros::NodeHandle nh, const std::string planning_group) : schunkGripper(nh), _nh(nh), move_group(planning_group){
        // Initialize the move_group
        // joint model group
        // visual markers
        PLANNING_GROUP = planning_group;
        init(_nh);
        tf2_ros::TransformListener listener(buffer);
}

iwtros::iiwaMove::~iiwaMove(){}

void iwtros::iiwaMove::init(ros::NodeHandle nh){
        _loadParam();
        _sub = nh.subscribe<geometry_msgs::TransformStamped>("/detected_goal", 10, boost::bind(&iiwaMove::callback, this, _1));
        _accpSub = nh.subscribe<std_msgs::Bool>("accept_pose", 10, boost::bind(&iiwaMove::acceptCallback, this, _1));
        _initialized = true;
        ready_pick_pose = false;
        _accept_pose = false;
}

void iwtros::iiwaMove::_loadParam(){
        PLANNER_ID = "PTP";
        REFERENCE_FRAME = "iiwa_link_0";
        EE_FRAME = "iiwa_link_ee";
        velocityScalling = 0.3;
        accelerationScalling = 0.3;
        // ToDo: input array param goals
}

geometry_msgs::PoseStamped iwtros::iiwaMove::generatePose(double x, double y, double z,
                                                double roll, double pitch, double yaw,
                                                std::string base_link){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = base_link.c_str();
        pose.header.stamp = ros::Time::now() + ros::Duration(2.1);
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        return pose;
}


void iwtros::iiwaMove::callback(const geometry_msgs::TransformStamped::ConstPtr& data){
    tf2Scalar roll, pitch, yaw;
    tf2::Quaternion q;
    tf2::fromMsg(data->transform.rotation, q);
    tf2::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    this->pick_pose = generatePose(data->transform.translation.x, data->transform.translation.y, 
                                   1.125 + data->transform.translation.z, M_PI, 0, yaw + M_PI/4, "iiwa_link_0");
    this->ready_pick_pose = true;
}

void iwtros::iiwaMove::acceptCallback(const std_msgs::Bool::ConstPtr &data)
{
    this->_accept_pose = data->data;
}

void iwtros::iiwaMove::run(){
        if(!_initialized){
                ROS_ERROR("IIWA Motion initialization is failed");
                return;
        }

        move_group.setPlannerId(PLANNER_ID);
        move_group.setMaxVelocityScalingFactor(velocityScalling);
        move_group.setMaxAccelerationScalingFactor(accelerationScalling);
        move_group.setPoseReferenceFrame(REFERENCE_FRAME);
        move_group.setEndEffectorLink(EE_FRAME);
        move_group.allowReplanning(true);

        std::thread t1(&iiwaMove::_ctrl_loop, this);
        // std::thread t2(&iiwaMove::_tf_listner_loop, this);
        t1.join();
        // t2.join();
        ros::shutdown();
}

void iwtros::iiwaMove::_ctrl_loop(){
        static ros::Rate r(1);
        // ToDo: Check asynchronous spinner is required
        ros::spinOnce();
        bool home_position = true;
        while(ros::ok()){
                geometry_msgs::PoseStamped place_pose = generatePose(0.228, -0.428, 1.2215, M_PI, 0 , M_PI/4 + M_PI/2, "iiwa_link_0");
                geometry_msgs::PoseStamped home_pose = generatePose(0.228, -0.428, 1.3, M_PI, 0 , M_PI/4, "iiwa_link_0");
                geometry_msgs::PoseStamped test_pose = generatePose(0.6, 0.09, 1.12, M_PI, 0 , M_PI/4 + M_PI/2, "iiwa_link_0");
                // this->pick_pose =  generatePose(0.694, 0.19, 1.14, M_PI, 0 , M_PI + M_PI/4, "iiwa_link_0");
                // ready_pick_pose = true;
                if(ready_pick_pose && _accept_pose){
                        ready_pick_pose = false;
                        _accept_pose = false;
                        _sub.shutdown();
                        ROS_WARN("Moving to Pick");
                        pnpPipeLine(this->pick_pose, place_pose, 0.15);
                        home_position = true;
                        _sub = _nh.subscribe<geometry_msgs::TransformStamped>("/detected_goal", 10, boost::bind(&iiwaMove::callback, this, _1)); 
                }if(home_position){
                        home_position = false;
                        ROS_WARN("Home Pose");
                        motionExecution(home_pose);
                }
                if(ready_pick_pose) ROS_INFO("Ready to accept the goal");
                else{
                        ROS_WARN("Doing Nothing and I am HAPPY!");
                }
                ros::spinOnce();        
                r.sleep();
        }
}

void iwtros::iiwaMove::pnpPipeLine(geometry_msgs::PoseStamped pick,
                        geometry_msgs::PoseStamped place,
                        const double offset){
        // Go to Pick prepose (PTP)
        pick.pose.position.z += offset;
        motionExecution(pick);
        this->openGripper();
        // Go to Pick pose, ToDo: Set LIN motion
        pick.pose.position.z -= offset;
        motionExecution(pick);
        this->closeGripper();
        ros::Duration(1.0).sleep();
        // Go to Pick Postpose, ToDo: Set LIN motion
        pick.pose.position.z += offset;
        motionExecution(pick);
        // Go to Place Prepose (PTP)
        place.pose.position.z += offset;
        motionExecution(place);
        // Go to Place pose, ToDo: Set LIN motion
        place.pose.position.z -= offset;
        motionExecution(place);
        this->openGripper();
        ros::Duration(0.5).sleep();
        // Go to Place Postpose, ToDo: Set LIN motion
        place.pose.position.z += offset;
        motionExecution(place);
        this->closeGripper();
        this->ackGripper();
}

void iwtros::iiwaMove::motionExecution(const geometry_msgs::PoseStamped pose){
        motionContraints(pose);
        move_group.setPoseTarget(pose);
        // ToDo: Valide the IK solution
        moveit::planning_interface::MoveGroupInterface::Plan mPlan;
        bool eCode = (move_group.plan(mPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_ERROR_STREAM_NAMED("PLAN","Motion planning is: " << eCode?"Success":"Failed");
        visualMarkers(pose, mPlan);
        if(eCode) move_group.execute(mPlan);
        move_group.clearTrajectoryConstraints();
        move_group.clearPoseTarget();
}

void iwtros::iiwaMove::motionContraints(const geometry_msgs::PoseStamped pose){
        // Orientation contraints
        moveit_msgs::OrientationConstraint oCon;
        oCon.header.frame_id = REFERENCE_FRAME;
        oCon.link_name = EE_FRAME;
        oCon.orientation = pose.pose.orientation;
        oCon.absolute_x_axis_tolerance = 0.01;
        oCon.absolute_y_axis_tolerance = 0.01;
        oCon.absolute_z_axis_tolerance = 0.01;
        oCon.weight = 1.0;
        // ToDO: planning Contraints
        // Trajectory Contraints
        moveit_msgs::TrajectoryConstraints tCon;
        tCon.constraints.resize(1);
        tCon.constraints[0].orientation_constraints.push_back(oCon);
        tCon.constraints[0].position_constraints.resize(1);
        tCon.constraints[0].position_constraints[0].header.frame_id = REFERENCE_FRAME;
        tCon.constraints[0].position_constraints[0].link_name = EE_FRAME;
        tCon.constraints[0].position_constraints[0].constraint_region.primitive_poses.resize(1);
        tCon.constraints[0].position_constraints[0].constraint_region.primitive_poses[0] = pose.pose;
        tCon.constraints[0].position_constraints[0].constraint_region.primitives.resize(1);
        tCon.constraints[0].position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
        tCon.constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions.push_back(2e-3);
        move_group.setTrajectoryConstraints(tCon);
        // ToDo: Goal Constraints
}

void iwtros::iiwaMove::visualMarkers(const geometry_msgs::PoseStamped target_pose,
                                        moveit::planning_interface::MoveGroupInterface::Plan plan){
        const robot_state::JointModelGroup * joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        moveit_visual_tools::MoveItVisualTools visual_tool(REFERENCE_FRAME);
        visual_tool.deleteAllMarkers();
        visual_tool.loadRemoteControl();
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 0.0;
        visual_tool.publishText(text_pose, "PnP Execution", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        // visual_tool.trigger();
        // Visualize Trajectory
        visual_tool.publishAxisLabeled(target_pose.pose, "PnP");
        visual_tool.publishTrajectoryLine(plan.trajectory_, joint_model_group);
        visual_tool.trigger();
}