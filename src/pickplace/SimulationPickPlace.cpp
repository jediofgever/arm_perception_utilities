/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:13:08
 * @modify date 2019-11-28 10:13:08
 * @desc [description]
 */
#include <arm_perception_utilities/pickplace/SimulationPickPlace.h>

/**
 * @brief Construct a new Fake Pick Place:: Fake Pick Place object
 *
 */
SimulationPickPlace::SimulationPickPlace(/* args */) {
    // Create a move_group pointer, that refers to our model "manipulator"
    static const std::string PLANNING_GROUP = "manipulator";
    move_group_ptr_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    // essential to perform any kind of picking and placing
    pick_placer_ = new PickandPlacer();

    // Define where to place the objects
    placing_pose.orientation = utils::EulertoQuaternion(180.0, 0.0, 180.0, kDEG2RAD);
    placing_pose.position.x = 0.450;
    placing_pose.position.y = 0.300;
    placing_pose.position.z = 0.20;
    placing_poses_vector.push_back(placing_pose);
    placing_pose.position.x -= 0.15;
    placing_poses_vector.push_back(placing_pose);
    placing_pose.position.x -= 0.15;
    placing_poses_vector.push_back(placing_pose);
    placing_pose.position.y += 0.15;
    placing_poses_vector.push_back(placing_pose);
    placing_pose.position.x += 0.15;
    placing_poses_vector.push_back(placing_pose);
    placing_pose.position.x += 0.15;
    placing_poses_vector.push_back(placing_pose);

    nh_ = new ros::NodeHandle();

    // Subscibe to detected  3d detections and do pick and place operation inside binded function
    // processRealPickPlaceCallback()
    detected_object_poses_sub =
        nh_->subscribe("grab_object_poses", 1, &SimulationPickPlace::processSimulationPickPlaceCallback, this);
}

/**
 * @brief Destroy the Fake Pick Place:: Fake Pick Place object
 *
 */
SimulationPickPlace::~SimulationPickPlace() {}

/**
 * @brief subscribe to detected objects and perfrom Pick and placing, simple
 *
 * @param detected_obj_pose_arrows
 */
void SimulationPickPlace::processSimulationPickPlaceCallback(
    const visualization_msgs::MarkerArrayConstPtr &detected_obj_pose_arrows) {
    std::vector<double> zeros = {0, 0.349066, 1.22173, 0, 0, 0};
    pick_placer_->goto_scanning_pose(move_group_ptr_, zeros);
    sleep(10.0);
    for (int i = 0; i < detected_obj_pose_arrows->markers.size(); i++) {
        int placing_pose_index = i % placing_poses_vector.size();
        visualization_msgs::Marker current_marker = detected_obj_pose_arrows->markers[i];
        pick_placer_->pick(move_group_ptr_, current_marker.points.front(), current_marker.points.back());
        pick_placer_->place(move_group_ptr_, placing_poses_vector[placing_pose_index]);
    }
}
