/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:06:29
 * @modify date 2019-11-28 10:06:29
 * @desc [description]
 */
#ifndef robot_controller_H
#define robot_controller_H
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

/**
 * @brief
 * Robot Controller class, moves Robot TCP and Joints into desired values with preffered spaces(joint/cartesian,tool)
 */
class RobotController {
   private:
    // ros node handle pointer
    ros::NodeHandle *node_handle_ptr_;
    // fake command publisher for gazebo gripper controlller
    ros::Publisher finger1_command_pub_;
    ros::Publisher finger2_command_pub_;
    ros::Publisher finger3_command_pub_;

   public:
    /**
     * @brief Construct a new Robot Controller object
     *
     */
    RobotController(/* args */);

    /**
     * @brief Destroy the Robot Controller object
     *
     */
    ~RobotController();

    /**
     * @brief moves robot tcp in joint space, a straight line following is not guarenteed
     *
     * @param robot_tcp_goal_in_joint_space
     * @param move_group_ptr_
     */
    void moveEndEffectortoGoalinJointSpace(geometry_msgs::Pose robot_tcp_goal_in_joint_space,
                                           moveit::planning_interface::MoveGroupInterface *move_group_ptr_);

    /**
     * @brief moves robot tcp in joint space, a straight line following is  guarenteed
     *
     * @param robot_tcp_goal_in_cartesian_space
     * @param move_group_ptr_
     */
    void moveEndEffectortoGoalinCartesianSpace(geometry_msgs::Pose robot_tcp_goal_in_cartesian_space,
                                               moveit::planning_interface::MoveGroupInterface *move_group_ptr_);

    /**
     * @brief moves each individual joint to given target position
     *
     * @param robot_joint_states
     * @param move_group_ptr_
     */
    void moveJointstoTargetPositions(std::vector<double> robot_joint_states,
                                     moveit::planning_interface::MoveGroupInterface *move_group_ptr_);

    /**
     * @brief moves robot along End-effector(TOOL) link frames
     *
     * @param robot_tcp_goal_in_tool_space
     * @param move_group_ptr_
     * @param listener_
     */
    void moveEndEffectortoGoalinToolSpace(geometry_msgs::Pose robot_tcp_goal_in_tool_space,
                                          moveit::planning_interface::MoveGroupInterface *move_group_ptr_,
                                          tf::TransformListener *listener_);

    /**
     * @brief not implemented yet, but will open the robot gripper
     *
     */
    void openGripper();

    /**
     * @brief not implemneted yet , but will close robots hand
     *
     */
    void closeGripper();

    /**
     * @brief stops current execution of any trajectory
     *
     * @param move_group_ptr_
     */
    void stopRobotTrajectrory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_);

    /**
     * @brief stops current execution of any trajectory but keeps memory of goal, so the execution can be continued
     *
     * @param move_group_ptr_
     */
    void pauseRobotTrajectrory(moveit::planning_interface::MoveGroupInterface *move_group_ptr_);
};
#endif
