/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:14:05
 * @modify date 2019-11-28 10:14:05
 * @desc [description]
 */

#include <arm_perception_utilities/visualization/RobotStateVisualization.h>

using namespace utils;
/**
 * @brief Construct a new Robot State Visualization:: Robot State Visualization object
 *
 */
RobotStateVisualization::RobotStateVisualization(/* args */) {
    node_handle_ptr_ = ros::NodeHandlePtr(new ros::NodeHandle());

    robot_state_visualization_marker_pub_ =
        node_handle_ptr_->advertise<visualization_msgs::MarkerArray>("visualization_marker", 0);

    static const std::string PLANNING_GROUP = "manipulator";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
}

/**
 * @brief Destroy the Robot State Visualization:: Robot State Visualization object
 *
 */
RobotStateVisualization::~RobotStateVisualization() {}

/**
 * @brief
 * Visualizes states of robot in RVIZ
 */
void RobotStateVisualization::visualizeRobotStates() {
    geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion current_orientation_quat;
    tf::quaternionMsgToTF(current_pose.pose.orientation, current_orientation_quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(current_orientation_quat).getRPY(roll, pitch, yaw);

    // Set marker info
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = current_pose.pose.position.x;
    marker.pose.position.y = current_pose.pose.position.y;
    marker.pose.position.z = current_pose.pose.position.z + 0.9;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.08;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    // text info will be displayed in RVIZ
    marker.text = "X: " + std::to_string(utils::round(current_pose.pose.position.x / kMM2M)) + "        " +
                  "J1:" + std::to_string(utils::round(move_group->getCurrentJointValues()[0] / kDEG2RAD)) + "\n" +
                  +"Y: " + std::to_string(utils::round(current_pose.pose.position.y / kMM2M)) + "        " +
                  "J2:" + std::to_string(utils::round(move_group->getCurrentJointValues()[1] / kDEG2RAD)) + "\n" +
                  +"Z: " + std::to_string(utils::round(current_pose.pose.position.z / kMM2M - 330.0)) + "        " +
                  "J3:" + std::to_string(utils::round(move_group->getCurrentJointValues()[2] / kDEG2RAD)) + "\n" +
                  "RX: " + std::to_string(utils::round(roll / kDEG2RAD)) + "        " +
                  "J4:" + std::to_string(utils::round(move_group->getCurrentJointValues()[3] / kDEG2RAD)) + "\n" +
                  "RY: " + std::to_string(utils::round(pitch / kDEG2RAD)) + "        " +
                  "J5:" + std::to_string(utils::round(move_group->getCurrentJointValues()[4] / kDEG2RAD)) + "\n" +
                  "RZ: " + std::to_string(utils::round(yaw / kDEG2RAD)) + "        " +
                  "J6:" + std::to_string(utils::round(move_group->getCurrentJointValues()[5] / kDEG2RAD));

    // push defined marker
    marker_array.markers.push_back(marker);
    marker.lifetime = ros::Duration(0);
    // publish robot state visualization marker
    robot_state_visualization_marker_pub_.publish(marker_array);
}
