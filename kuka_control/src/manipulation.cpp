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
}

iwtros::iiwaMove::~iiwaMove(){}

void iwtros::iiwaMove::init(ros::NodeHandle nh){
        _loadParam();
        _sub = nh.subscribe<geometry_msgs::Transform>("detected_pose", 10, boost::bind(&iiwaMove::callback, this, _1));
        _initialized = true;
        ready_pick_pose = false;
}

void iwtros::iiwaMove::_loadParam(){
        PLANNER_ID = "PTP";
        REFERENCE_FRAME = "iiwa_link_0";
        EE_FRAME = "iiwa_link_ee";
        velocityScalling = 0.4;
        accelerationScalling = 0.4;
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

void iwtros::iiwaMove::generateMeanPose(const geometry_msgs::Transform detection)
{
        double x = 0, y = 0, yaw = 0;
        int counter = 0;
        geometry_msgs::Transform new_pose;
        for (int i = 0; i < poseY.size(); i ++)
        {
                x += poseX.front();
                y += poseY.front();
                yaw += _yaw.front();
                counter += 1;
        }
        new_pose = detected_pose;
        new_pose.translation.x = x / counter;
        new_pose.translation.y = y / counter;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw/counter);
        new_pose.rotation  = tf2::toMsg(q);
        while(poseY.size() > 0 && poseX.size() > 0)
        {
                poseY.pop();
                poseX.pop();
        }
        this->generatePickPose(new_pose, "iiwa_link_0");
}

void iwtros::iiwaMove::generatePickPose(const geometry_msgs::Transform detection, std::string base_link){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = base_link.c_str();
        pose.header.stamp = ros::Time::now() + ros::Duration(2.1);
        ROS_INFO("Detected pose x: %f", detection.translation.x);
        if(detection.translation.x > 0.20 || detection.translation.x < -0.20 || detection.translation.y > 0.20 || detection.translation.y < -0.20){
                ROS_ERROR("False detection");
                return;
        }

        pose.pose.position.x = 0.6 + detection.translation.x;
        pose.pose.position.y = 0.09 - detection.translation.y;
        pose.pose.position.z = 1.12;                     // Fixed Pose for the robot
        tf2::Quaternion quad, q;
        tf2Scalar roll, pitch, yaw;
        tf2::fromMsg(detection.rotation, quad);
        tf2::Matrix3x3 mat(quad);
        mat.getEulerYPR(yaw, pitch, roll);
        /// Yaw Correction. 
        tf2Scalar fixed_yaw = (3*M_PI)/4;
        double new_yaw = fixed_yaw - yaw;
        q.setRPY(M_PI, 0, new_yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        this->pick_pose = pose;
        this->ready_pick_pose = true;
}

void iwtros::iiwaMove::callback(const geometry_msgs::Transform::ConstPtr& data){
        this->detected_pose.translation.x = data->translation.x;
        this->detected_pose.translation.y = data->translation.y;
        this->detected_pose.translation.z = data->translation.z;
        this->detected_pose.rotation.x = data->rotation.x;
        this->detected_pose.rotation.y = data->rotation.y;
        this->detected_pose.rotation.z = data->rotation.z;
        this->detected_pose.rotation.w = data->rotation.w;

        this->generatePickPose(this->detected_pose, "iiwa_link_0");
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
        t1.join();
        ros::shutdown();
}

void iwtros::iiwaMove::_ctrl_loop(){
        static ros::Rate r(1);
        // ToDo: Check asynchronous spinner is required
        ros::spinOnce();
        while(ros::ok()){
                geometry_msgs::PoseStamped place_pose = generatePose(0.228, -0.428, 1.2215, M_PI, 0 , M_PI/4, "iiwa_link_0");
                geometry_msgs::PoseStamped home_pose = generatePose(0.228, -0.428, 1.2215, M_PI, 0 , M_PI/4, "iiwa_link_0");

                if(ready_pick_pose){
                        ready_pick_pose = false;
                        pnpPipeLine(this->pick_pose, place_pose, 0.12);
                }else{
                        motionExecution(home_pose);
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