/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:13:53
 * @modify date 2019-11-28 10:13:53
 * @desc [description]
 */
#include <arm_perception_utilities/vision/Object3DDetector.h>
#include <arm_perception_utilities/vision/PointCloudManager.h>
#include <pwd.h>
#include <python3.6m/Python.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <sstream>

/**
 * @brief MAIN ros node responsible for 3D detection of obejct, This node subscribes to masrcnn segmented image, uses
 * utilities of Pointcloud Manager to achieve final 3D object Pose and dimensions detection
 *
 */

using namespace boost::filesystem;
int main(int argc, char **argv) {
    // init node
    ros::init(argc, argv, "vision_ros_node");

    // a node handler to handle usual ROS stuff
    ros::NodeHandle node_handle;

    // Create a multithreaded spinner for this node
    ros::AsyncSpinner spinner(10);
    // init spinner
    spinner.start();

    // init pointers to point clous manager and 3d object detector objects
    PointCloudManager *pointcloud_manager_ = new PointCloudManager();
    Object3DDetector object3d_detector(pointcloud_manager_);

    // into infine loop of spinning
    while (ros::ok()) {
        int node_loop_rate = 50;
        ros::Rate loop_rate(node_loop_rate);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
