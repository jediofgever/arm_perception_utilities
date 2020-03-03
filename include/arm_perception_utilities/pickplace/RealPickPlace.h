/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:03:38
 * @modify date 2019-11-28 10:03:38
 * @desc [description]
 */
#ifndef real_pick_place_H
#define real_pick_place_H
#include <arm_perception_utilities/pickplace/PickandPlacer.h>
#include <arm_perception_utilities/utils.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace utils;
/**
 * @brief
 * Real Pick and place class to run Pick and Place on Robot
 */
class RealPickPlace {
   private:
    // Default home pose , where robot returns
    geometry_msgs::Pose home_pose;

    // the pose where robot takes scanning shots , in order to detect objects
    geometry_msgs::Pose scan_scene_pose;

    // move_group class pointer
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_;

    // Pick and placer in order to plan and execute motions of robot , to manipukate objects
    PickandPlacer *pick_placer_;

    // ROS node handler to handle usual ROS stuff
    ros::NodeHandle *nh_;

    // Detectd Objects are publisher from vision/Object3DDetector class, this subscriber listens to latest detcetions
    ros::Subscriber detected_object_poses_sub;

    // Radians are feeded into functions , Degrees are mostly for visualizations
    double const kDEG2RAD = M_PI / 180.0;

   public:
    /**
     * @brief Construct a new Fake Pick Place object
     *
     */
    RealPickPlace(/* args */);

    /**
     * @brief Destroy the Fake Pick Place object
     *
     */
    ~RealPickPlace();

    /**
     * @brief listens to detected objects and process tthe picking and placing them
     *
     * @param detected_obj_pose_arrows
     */
    void processRealPickPlaceCallback(const visualization_msgs::MarkerArrayConstPtr &detected_obj_pose_arrows);
};
#endif
