/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:13:08
 * @modify date 2019-11-28 10:13:08
 * @desc [description]
 */
#include <arm_perception_utilities/pickplace/FakePickPlace.h>

/**
 * @brief Construct a new Fake Pick Place:: Fake Pick Place object
 *
 */
FakePickPlace::FakePickPlace(/* args */) {
    // Create a move_group pointer, that refers to our model "manipulator"

    static const std::string PLANNING_GROUP = "manipulator";
    move_group_ptr_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    // essential to perform any kind of picking and placing
    pick_placer_ = new PickandPlacer();

    geometry_msgs::Pose sample_object;

    // ROS node handler to handle usual ROS stuff(publisher, subscriber etc)
    nh_ = new ros::NodeHandle();

    // get , load Fake object poses from cfg/fake_objects.yaml
    int number_of_fake_objects;
    nh_->getParam("/fake_objects/number_of_fake_objects", number_of_fake_objects);

    for (int i = 0; i < number_of_fake_objects; i++) {
        geometry_msgs::Pose fake_object_pose_msgs;
        std::vector<double> object_pose;

        std::string path_to_fake_object_pose = "/fake_objects/fake_object_" + std::to_string(i) + "_pose";
        nh_->getParam(path_to_fake_object_pose, object_pose);
        fake_object_pose_msgs.position.x = object_pose[0];
        fake_object_pose_msgs.position.y = object_pose[1];
        fake_object_pose_msgs.position.z = object_pose[2];
        fake_object_pose_msgs.orientation =
            utils::EulertoQuaternion(object_pose[3], object_pose[4], object_pose[5], kDEG2RAD);
        object_list.push_back(fake_object_pose_msgs);
    }
    // home pose is where we place virtual objects
    home_pose = sample_object;
    home_pose.position.y = -sample_object.position.y;
    home_pose.orientation = utils::EulertoQuaternion(180.0, 0.0, 30.0, kDEG2RAD);

    std::vector<double> home_pose_vector;

    std::string path_to_home_pose = "/fake_objects/home_pose";
    nh_->getParam(path_to_home_pose, home_pose_vector);
    home_pose.position.x = home_pose_vector[0];
    home_pose.position.y = home_pose_vector[1];
    home_pose.position.z = home_pose_vector[2];
    home_pose.orientation =
        utils::EulertoQuaternion(home_pose_vector[3], home_pose_vector[4], home_pose_vector[5], kDEG2RAD);
}

/**
 * @brief Destroy the Fake Pick Place:: Fake Pick Place object
 *
 */
FakePickPlace::~FakePickPlace() {}

/**
 * @brief Once we loaded fake object poses from cfg/fake_objects.yaml
 * then we can fed these objects into pick and placer pipline
 *
 * @return int
 */
int FakePickPlace::processFakePickPlace() {
    for (int i = 0; i < object_list.size(); i++) {
        pick_placer_->pick(move_group_ptr_, object_list.at(i));
        pick_placer_->place(move_group_ptr_, home_pose);
    }
    return 0;
}
