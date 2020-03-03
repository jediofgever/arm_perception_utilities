/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 09:59:41
 * @modify date 2019-11-28 09:59:41
 * @desc [description]
 */
#ifndef robot_state_visualization_H
#define robot_state_visualization_H
#include <ros/ros.h>

#include <arm_perception_utilities/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

/**
 * @brief
 * States of robot visualizer in RVIZ
 */
class RobotStateVisualization {
   private:
    // a ROS MARKER PUBLISHER, FOR pOSE OF END EFFECTOR LINK AND STATES OF JOINT ANGLES
    ros::Publisher robot_state_visualization_marker_pub_;

    // We show angles in Degrees , we need this constant to do transforms between radians and degrees
    double const kDEG2RAD = M_PI / 180.0;

    // We show distance in millimeters , we need this constant to do transforms between millimeters  and meters
    double const kMM2M = 0.001;

    // ROS node handler to handle usual ROS stuff(publiher subscriber etc. )
    ros::NodeHandlePtr node_handle_ptr_;

    // move_group class pointer
    moveit::planning_interface::MoveGroupInterface *move_group;

   public:
    /**
     * @brief Construct a new Robot State Visualization object
     *
     */
    RobotStateVisualization();

    /**
     * @brief Destroy the Robot State Visualization object
     *
     */
    ~RobotStateVisualization();

    /**
     * @brief Gets Robot states from move_group and publishes then real-time as RVIZ ros markers
     *
     */
    void visualizeRobotStates();
};
#endif