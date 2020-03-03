/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:14:10
 * @modify date 2019-11-28 10:14:10
 * @desc [description]
 */
#include <arm_perception_utilities/visualization/RobotStateVisualization.h>
#include <ros/ros.h>

/**
 * @brief
 * A ROS Node responsible for visualizing robot states in RVIZ
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
    // init node
    ros::init(argc, argv, "state_visualization_ros_node");

    // a node hnadler to handle usual ROS stuff
    ros::NodeHandle node_handle;

    // init pointer to robot state visualizer
    RobotStateVisualization *robot_state_vis_ = new RobotStateVisualization();

    // multutherded ROS spinner
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok()) {
        int node_loop_rate = 50;
        ros::Rate loop_rate(node_loop_rate);
        robot_state_vis_->visualizeRobotStates();

        loop_rate.sleep();

        ros::spinOnce();
    }

    return 0;
}
