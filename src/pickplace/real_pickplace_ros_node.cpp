/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:13:02
 * @modify date 2019-11-28 10:13:02
 * @desc [description]
 */
#include <arm_perception_utilities/pickplace/RealPickPlace.h>
#include <ros/ros.h>

/**
 * @brief
 * @brief ROS node to  do a real pick and place operation, virtual object poses are listed in cfg/fake_objects.yaml, we
 * get these poses of "fake objects and perform fake pick and place operation on robot "
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
    // init the node and give it a proper name
    ros::init(argc, argv, "real_pickplace_ros_node");
    // node handler to handle usual ROS stuff
    ros::NodeHandle node_handle;

    // Real Pick and place operations are in this class , creates pointer to that object
    RealPickPlace *real_pickplace_ = new RealPickPlace();

    // Multithreaded ROS spinner
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok()) {
        // execute All pick and place operations and return with sucess
        int node_loop_rate = 50;
        ros::Rate loop_rate(node_loop_rate);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
