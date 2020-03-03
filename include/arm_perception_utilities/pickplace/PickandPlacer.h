/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:04:35
 * @modify date 2019-11-28 10:04:35
 * @desc [description]
 */
#ifndef pick_placer_H
#define pick_placer_H
#include <arm_perception_utilities/control/RobotController.h>
#include <arm_perception_utilities/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace utils;
/**
 * @brief
 * A class to execute Pick and Place
 */
class PickandPlacer {
   private:
    // Transform listener , knows continous transforms between each link
    tf::TransformListener *listener_;

    // Controller to execute motion commands
    RobotController *robot_controller_ptr_;

    // we feed Radians NOT degrees, so conversion between them are normal routine
    double const kDEG2RAD = M_PI / 180.0;

   public:
    /**
     * @brief Construct a new Pickand Placer object
     *
     */
    PickandPlacer(/* args */);

    /**
     * @brief Destroy the Pickand Placer object
     *
     */
    ~PickandPlacer();

    /**
     * @brief given an object pose , approaches to object , closes gripper, pulls the object back and then place()
     * function takes action
     *
     * @param move_group_ptr_
     * @param grasp_pose
     */
    void pick(moveit::planning_interface::MoveGroupInterface *move_group_ptr_, geometry_msgs::Pose grasp_pose);

    /**
     * @brief given an object pose , approaches to object , closes gripper, pulls the object back and then place()
     * function takes action
     *
     * @param move_group_ptr_
     * @param grasp_pose
     */
    void pick(moveit::planning_interface::MoveGroupInterface *move_group_ptr_, geometry_msgs::Point pre_grasp_point,
              geometry_msgs::Point grasp_point);

    /**
     * @brief given an placing pose , with object in its gripper , goes to placing location opens gripper and places
     * object into that pose() function takes action
     *
     * @param move_group_ptr_
     * @param grasp_pose
     */
    void place(moveit::planning_interface::MoveGroupInterface *move_group_ptr_, geometry_msgs::Pose home_pose);

    /**
     * @brief Adefault pose , where robot goes and takes scaning shots in order to detect objects
     *
     * @param move_group_ptr_
     * @param scan_objects_pose
     */
    void goto_scanning_pose(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                            std::vector<double> joint_values);

    /**
     * @brief Adefault pose , where robot goes and takes scaning shots in order to detect objects
     *
     * @param move_group_ptr_
     * @param scan_objects_pose
     */
    void goto_scanning_pose(moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                            geometry_msgs::Pose scan_objects_pose);
};
#endif
