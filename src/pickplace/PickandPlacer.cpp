/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:13:16
 * @modify date 2019-11-28 10:13:16
 * @desc [description]
 */
#include <arm_perception_utilities/pickplace/PickandPlacer.h>

/**
 * @brief Construct a new Pickand Placer:: Pickand Placer object
 *
 */
PickandPlacer::PickandPlacer(/* args */) {
    // robot_controller_ptr_ = new RobotController();

    listener_ = new tf::TransformListener();
    robot_controller_ptr_ = new RobotController();
}

/**
 * @brief Destroy the Pickand Placer:: Pickand Placer object
 *
 */
PickandPlacer::~PickandPlacer() {}

/**
 * @brief
 *
 * @param move_group
 */
void PickandPlacer::pick(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                         geometry_msgs::Point pre_grasp_point, geometry_msgs::Point grasp_point)

{
    robot_controller_ptr_->openGripper();
    geometry_msgs::Pose pre_grasp_pose;
    pre_grasp_pose.position.x = pre_grasp_point.x;
    pre_grasp_pose.position.y = pre_grasp_point.y;
    pre_grasp_pose.position.z = pre_grasp_point.z;

    float rot_x = std::atan2(pre_grasp_point.z - grasp_point.z, pre_grasp_point.x - grasp_point.x);
    float rot_y = std::atan2(pre_grasp_point.z - grasp_point.z, pre_grasp_point.y - grasp_point.y);
    float rot_z = std::atan2(pre_grasp_point.x - grasp_point.x, pre_grasp_point.y - grasp_point.y);
    tf::quaternionTFToMsg(tf::Quaternion(-rot_x + 1.57, rot_y + 1.57, 0), pre_grasp_pose.orientation);

    robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(pre_grasp_pose, move_group_ptr_);

    geometry_msgs::Pose pre_pick_path;
    pre_pick_path.position.x = 0;
    pre_pick_path.position.y = 0;
    pre_pick_path.position.z = pre_grasp_point.z - grasp_point.z;
    tf::quaternionTFToMsg(tf::Quaternion(-rot_x + 1.57, rot_y + 1.57, 0), pre_grasp_pose.orientation);

    robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(pre_pick_path, move_group_ptr_, listener_);
    robot_controller_ptr_->closeGripper();

    geometry_msgs::Pose post_pick_path;
    post_pick_path.position.x = 0;
    post_pick_path.position.y = 0;
    post_pick_path.position.z = grasp_point.z - pre_grasp_point.z - 0.2;
    tf::quaternionTFToMsg(tf::Quaternion(-rot_x + 1.57, rot_y + 1.57, 0), pre_grasp_pose.orientation);

    robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(post_pick_path, move_group_ptr_, listener_);
}

/**
 * @brief
 *
 * @param move_group_ptr_
 * @param object_pose
 */
void PickandPlacer::pick(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                         geometry_msgs::Pose object_pose)

{
    robot_controller_ptr_->openGripper();
    geometry_msgs::Pose grasp_pose = object_pose;
    grasp_pose.position.z += 0.3;
    robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(grasp_pose, move_group_ptr_);

    geometry_msgs::Pose pre_pick_path;
    pre_pick_path.position.z = 0.05;
    robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(pre_pick_path, move_group_ptr_, listener_);
    robot_controller_ptr_->closeGripper();

    geometry_msgs::Pose post_pick_path;
    post_pick_path.position.z = -0.35;
    robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(post_pick_path, move_group_ptr_, listener_);
}

/**
 * @brief
 *
 * @param move_group
 */
void PickandPlacer::place(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                          geometry_msgs::Pose home_pose) {
    geometry_msgs::Pose place_pose = home_pose;
    place_pose.position.z += 0.05;
    robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(place_pose, move_group_ptr_);

    geometry_msgs::Pose pre_place_path;
    pre_place_path.position.z = 0.05;
    robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(pre_place_path, move_group_ptr_, listener_);
    robot_controller_ptr_->openGripper();
    geometry_msgs::Pose post_place_path;
    post_place_path.position.z = -0.35;
    robot_controller_ptr_->moveEndEffectortoGoalinToolSpace(post_place_path, move_group_ptr_, listener_);
    robot_controller_ptr_->closeGripper();
}

/**
 * @brief
 *
 * @param move_group_ptr_
 * @param scan_objects_pose
 */
void PickandPlacer::goto_scanning_pose(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                                       std::vector<double> joint_values) {
    robot_controller_ptr_->moveJointstoTargetPositions(joint_values, move_group_ptr_);
}

/**
 * @brief
 *
 * @param move_group_ptr_
 * @param scan_objects_pose
 */
void PickandPlacer::goto_scanning_pose(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                                       geometry_msgs::Pose scan_objects_pose) {
    robot_controller_ptr_->moveEndEffectortoGoalinCartesianSpace(scan_objects_pose, move_group_ptr_);
}
