/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:12:38
 * @modify date 2019-11-28 10:12:38
 * @desc [description]
 */
#include <arm_perception_utilities/control/RobotController.h>

/**
 * @brief Construct a new Robot Controller:: Robot Controller object
 *
 */
RobotController::RobotController(/* args */) {
    // ros node handle pointer
    node_handle_ptr_ = new ros::NodeHandle();

    // fake command publisher for gazebo gripper controlller, note; ONLY USED for gazebo simulation NOT(yet) real robot
    finger1_command_pub_ =
        node_handle_ptr_->advertise<std_msgs::Float64>("Gripper_Finger1_position_controller/command", 1);
    finger2_command_pub_ =
        node_handle_ptr_->advertise<std_msgs::Float64>("Gripper_Finger2_position_controller/command", 1);
    finger3_command_pub_ =
        node_handle_ptr_->advertise<std_msgs::Float64>("Gripper_Finger3_position_controller/command", 1);

    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link", "/moveit_visual_markers"));

}

/**
 * @brief Destroy the Robot Controller:: Robot Controller object
 *
 */
RobotController::~RobotController() {}

/**
 * @brief moves robot tcp in joint space, a straight line following is not guarenteed
 *
 * @param robot_tcp_goal_in_joint_space
 * @param move_group_ptr_
 */
void RobotController::moveEndEffectortoGoalinJointSpace(
    geometry_msgs::Pose robot_tcp_goal_in_joint_space,
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_) {
    // set pose target for tcp
    move_group_ptr_->setPoseTarget(robot_tcp_goal_in_joint_space);

    // Now, we call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // chechk wheter a sucessfull plan was found
    bool success = (move_group_ptr_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    publishRobotTrajectory(move_group_ptr_, my_plan.trajectory_);


    // stream some info about this sucess state
    ROS_INFO_NAMED("Motion Plan for End-Effector %s", success ? "SUCCESSED" : "FAILED");

    // Move the manipulator according to Updated End-Effector goal pose
    move_group_ptr_->execute(my_plan);
}

/**
 * @brief moves robot tcp in joint space, a straight line following is  guarenteed
 *
 * @param robot_tcp_goal_in_cartesian_space
 * @param move_group_ptr_
 */
void RobotController::moveEndEffectortoGoalinCartesianSpace(
    geometry_msgs::Pose robot_tcp_goal_in_cartesian_space,
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_) {
    // get current pose ob tcp, because we will calculate a linear path, which needs current and target poses
    geometry_msgs::Pose current_pose = move_group_ptr_->getCurrentPose().pose;

    // We have two way point to calculate a linear path in between
    std::vector<geometry_msgs::Pose> waypoints;

    // Push current pose and target pose to way points
    //waypoints.push_back(current_pose);
    waypoints.push_back(robot_tcp_goal_in_cartesian_space);

    // compute a cartesian(linear) line between current and target pose
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_ptr_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // We dont actually plan, but we need to set the trajectory of plan to calculated cartesian path
    moveit::planning_interface::MoveGroupInterface::Plan myplan;
    myplan.trajectory_ = trajectory;

    publishRobotTrajectory(move_group_ptr_, myplan.trajectory_);


    // Move the manipulator according to calculated cartesian(linear) path
    move_group_ptr_->execute(myplan);
}

/**
 * @brief moves robot along End-effector(TOOL) link frames
 *
 * @param robot_tcp_goal_in_tool_space
 * @param move_group_ptr_
 * @param listener_
 */
void RobotController::moveEndEffectortoGoalinToolSpace(geometry_msgs::Pose robot_tcp_goal_in_tool_space,
                                                       moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                                                       tf::TransformListener *listener_) {
    /******************************** READ FUNCTION DESCRIPTION BELOW   ***********************************/
    /*
    This function moves robot tcp in tool link according to given amount of distance by user/program
    The given goal is in tool_link, this goal is transformed to base_link and then a cartesian/linear path is calculated
     */

    // get given goal postion(in tool_link frame)
    geometry_msgs::Pose robot_goal_in_tool_link_msg;
    robot_goal_in_tool_link_msg.position = robot_tcp_goal_in_tool_space.position;

    // we will trasnform the given goal from tool_link to base_link
    tf::Transform robot_goal_in_tool_link_tf;
    tf::poseMsgToTF(robot_goal_in_tool_link_msg, robot_goal_in_tool_link_tf);

    // get transform between tool_link and base_link
    tf::StampedTransform tool_to_base_link_transform;
    // lookup transform (this should be cached, since it’s probably static)
    listener_->lookupTransform("base_link", "tool0", ros::Time(0.0f), tool_to_base_link_transform);

    // do the transform by multipliying transfrom matrice with goal pose in tool_link
    tf::Transform robot_tcp_in_base_link_tf;
    robot_tcp_in_base_link_tf = tool_to_base_link_transform * robot_goal_in_tool_link_tf;

    // MOVEIT uses geometry_msgs type , so convert transformed goal pose to geometry_msgs
    geometry_msgs::Pose robot_goal_in_base_link_msg;
    tf::poseTFToMsg(robot_tcp_in_base_link_tf, robot_goal_in_base_link_msg);

    // we will do a linear plan between current pose and given target pose, so push the current pose to waypoint vector
    std::vector<geometry_msgs::Pose> waypoints;
    //waypoints.push_back(move_group_ptr_->getCurrentPose().pose);

    // Since we are moving along tool_link , the orintation of robot tcp shall not change
    robot_goal_in_base_link_msg.orientation = move_group_ptr_->getCurrentPose().pose.orientation;
    waypoints.push_back(robot_goal_in_base_link_msg);

    // compute a cartesian path between target pose(in base_link) and current pose
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_ptr_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // stream how much percent of this cartesian path can be achieved
    ROS_INFO("Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // We dont actually plan, but we need to set the trajectory of plan to calculated cartesian path
    moveit::planning_interface::MoveGroupInterface::Plan myplan;
    myplan.trajectory_ = trajectory;

    publishRobotTrajectory(move_group_ptr_, myplan.trajectory_);


    // Move the manipulator according to calculated cartesian(linear) path
    move_group_ptr_->execute(myplan);
}

/**
 * @brief moves each individual joint to given target position
 *
 * @param robot_joint_states
 * @param move_group_ptr_
 */
void RobotController::moveJointstoTargetPositions(std::vector<double> robot_joint_states,
                                                  moveit::planning_interface::MoveGroupInterface *move_group_ptr_) {
    // Set new joint states with modified Vector
    move_group_ptr_->setJointValueTarget(robot_joint_states);

    // Now, we call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan move_joint_plan;

    // check if a valid plan was found for this joint targets
    bool success = (move_group_ptr_->plan(move_joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Motion Plan for Joint %s", success ? "SUCCESSED" : "FAILED");

    publishRobotTrajectory(move_group_ptr_, move_joint_plan.trajectory_);


    // Move the manipulator according to Updated Joint States
    bool result = (move_group_ptr_->execute(move_joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

/**
 * @brief not implemented yet, but will open the robot gripper
 *
 */
void RobotController::openGripper() {
    /*** THIS FUNCTION IS USED FOR SIMULATOR ONLY ***/
    /*** THIS FUNCTION WILL BE MODIFIED ****/
    std_msgs::Float64 finger1_position, finger2_position, finger3_position;
    finger1_position.data = 1.0;
    finger2_position.data = 1.0;
    finger3_position.data = -1.0;
    finger1_command_pub_.publish(finger1_position);
    finger2_command_pub_.publish(finger2_position);
    finger3_command_pub_.publish(finger3_position);
}

/**
 * @brief not implemneted yet , but will close robots hand
 *
 */
void RobotController::closeGripper() {
    /*** THIS FUNCTION IS USED FOR SIMULATOR ONLY ***/
    /*** THIS FUNCTION WILL BE MODIFIED ****/
    std_msgs::Float64 finger1_position, finger2_position, finger3_position;
    finger1_position.data = -1.0;
    finger2_position.data = -1.0;
    finger3_position.data = 1.0;
    finger1_command_pub_.publish(finger1_position);
    finger2_command_pub_.publish(finger2_position);
    finger3_command_pub_.publish(finger3_position);
}

/**
 * @brief stops current execution of any trajectory
 *
 * @param move_group_ptr_
 */
void RobotController::stopRobotTrajectrory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_) {
    // Stops current path executions , if any
    move_group_ptr_->stop();
}

/**
 * @brief stops current execution of any trajectory but keeps memory of goal, so the execution can be continued
 *
 * @param move_group_ptr_
 */
void RobotController::pauseRobotTrajectrory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_) {
    // Pauses current path executions , if any
    move_group_ptr_->stop();
}

/**
 * @brief publishes the planned path of robot, and visualizes it in RVIZ
 *
 */
void RobotController::publishRobotTrajectory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                                             moveit_msgs::RobotTrajectory trajectory) {
    // Whicj joint model group's trajectory you would like to visualize ? , in our case "manipulator"
    joint_model_group = move_group_ptr_->getCurrentState()->getJointModelGroup("manipulator");

    // Delete any marker publushed by visual_tools_
    visual_tools_->deleteAllMarkers();

    // Convert trejectory in joint states to EEF poses
    robot_trajectory::RobotTrajectoryPtr robot_trajectory(
        new robot_trajectory::RobotTrajectory(move_group_ptr_->getRobotModel(), joint_model_group->getName()));

    // set the trajectory
    robot_trajectory->setRobotTrajectoryMsg(*move_group_ptr_->getCurrentState(), trajectory);

    // the path points will be pushed into Eigen container
    EigenSTL::vector_Vector3d path;
    // go through each way point of our trajectory
    for (std::size_t i = 0; i < robot_trajectory->getWayPointCount(); ++i) {
        // tip pose W.R.T its parent link
        const Eigen::Isometry3d &tip_pose = robot_trajectory->getWayPoint(i).getGlobalLinkTransform("link_6");

        // Error Check
        if (tip_pose.translation().x() != tip_pose.translation().x()) {
            ROS_INFO("NAN DETECTED AT TRAJECTORY POINT i=");
            return;
        }

        // push this point into path , which krrps waypoints inside
        path.push_back(tip_pose.translation());

        // put a sphere on this way point
        visual_tools_->publishSphere(tip_pose, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
    }

    // radius of path
    const double radius = 0.006;

    // publish the path
    visual_tools_->publishPath(path, rviz_visual_tools::RED, radius);

    // trigger action for moveit_tools_ markers
    visual_tools_->trigger();
}
