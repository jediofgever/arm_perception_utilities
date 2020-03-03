/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:13:08
 * @modify date 2019-11-28 10:13:08
 * @desc [description]
 */
#include <arm_perception_utilities/pickplace/RealPickPlace.h>

/**
 * @brief Construct a new Fake Pick Place:: Fake Pick Place object
 *
 */
RealPickPlace::RealPickPlace(/* args */) {
    // Create a move_group pointer, that refers to our model "manipulator"
    static const std::string PLANNING_GROUP = "manipulator";
    move_group_ptr_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    // essential to perform any kind of picking and placing
    pick_placer_ = new PickandPlacer();

    // Define where to place the objects
    home_pose.orientation = utils::EulertoQuaternion(180.0, 0.0, 30.0, kDEG2RAD);
    home_pose.position.x = 0.645;
    home_pose.position.y = 0.350;
    home_pose.position.z = 0.200;

    // define where to take shot to do 3d deetection from 3D camera
    scan_scene_pose.orientation = utils::EulertoQuaternion(180.0, 10.0, 0.0, kDEG2RAD);
    scan_scene_pose.position.x = 0.830;
    scan_scene_pose.position.y = 0.000;
    scan_scene_pose.position.z = 0.510;
    nh_ = new ros::NodeHandle();

    // Subscibe to detected  3d detections and do pick and place operation inside binded function
    // processRealPickPlaceCallback()
    detected_object_poses_sub =
        nh_->subscribe("grab_object_poses", 1, &RealPickPlace::processRealPickPlaceCallback, this);
}

/**
 * @brief Destroy the Fake Pick Place:: Fake Pick Place object
 *
 */
RealPickPlace::~RealPickPlace() {}

/**
 * @brief subscribe to detected objects and perfrom Pick and placing, simple
 *
 * @param detected_obj_pose_arrows
 */
void RealPickPlace::processRealPickPlaceCallback(
    const visualization_msgs::MarkerArrayConstPtr &detected_obj_pose_arrows) {
    pick_placer_->goto_scanning_pose(move_group_ptr_, scan_scene_pose);
    sleep(5.0);
    for (int i = 0; i < detected_obj_pose_arrows->markers.size(); i++) {
        visualization_msgs::Marker current_marker = detected_obj_pose_arrows->markers[i];
        pick_placer_->pick(move_group_ptr_, current_marker.points.front(), current_marker.points.back());
        pick_placer_->place(move_group_ptr_, home_pose);
    }
}
