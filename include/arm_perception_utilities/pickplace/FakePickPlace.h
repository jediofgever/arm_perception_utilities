/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:03:38
 * @modify date 2019-11-28 10:03:38
 * @desc [description]
 */
#ifndef fake_pick_place_H
#define fake_pick_place_H
#include <arm_perception_utilities/pickplace/PickandPlacer.h>
#include <arm_perception_utilities/utils.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace utils;
/**
 * @brief
 * Fake Pick and place class to run experiments on Robot
 */
class FakePickPlace {
   private:
    // a list of object with their Poses
    std::vector<geometry_msgs::Pose> object_list;
    // default home pose robot returns to
    geometry_msgs::Pose home_pose;
    // Pointer to move_group_ Class
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_;

    // picker and placer class, refer to class for details
    PickandPlacer *pick_placer_;

    // ROS node handler to handle usual ROS stuff
    ros::NodeHandle *nh_;

    // we need to feed radians NOT degrees
    double const kDEG2RAD = M_PI / 180.0;

   public:
    /**
     * @brief Construct a new Fake Pick Place object
     *
     */
    FakePickPlace(/* args */);

    /**
     * @brief Destroy the Fake Pick Place object
     *
     */
    ~FakePickPlace();

    /**
     * @brief Construct a new process Pick Place object
     *
     */
    int processFakePickPlace();
};
#endif
