/**
 * @author [Fetullah Atas]
 * @email [fetulahatas1@gmail.com]
 * @create date 2020-02-18 10:10:48
 * @modify date 2020-02-18 10:10:48
 * @desc [description]
 */

#ifndef object_3DDetector_H
#define object_3DDetector_H
#include <arm_perception_utilities/vision/PointCloudManager.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <chrono>

/**
 * @brief Subscribes to Realsense Point clouds, as well as segmented MakRCNN image,
 * Makes use of functions located at PointCloud MAnAGER IN ORDER TO ACHEIVE 3d detection of obejcts
 *
 */
class Object3DDetector {
   private:
    // ROS nOde handler to handle
    ros::NodeHandle *nh_;

    // MOve_group Class pointer
    moveit::planning_interface::MoveGroupInterface *move_group_ptr_;

    // Planning scene interfce for Robot
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Subscribe to  segmented image from Maskrcnn
    ros::Subscriber segmented_img_sub_;

    // Subscribe to raw realsense point cloud
    ros::Subscriber raw_cloud_sub_;

    // Transform listener to listen transforms betwenn links
    tf::TransformListener *listener_;

    // Subscribe to raw point cloud from realsense , Topic name
    std::string sub_raw_cloud_topic_name;

    // Subscribe to sgemented image from Maskrcnn , Topic name
    std::string sub_segmented_image_topic_name;

    // ground plane model params found by PCL
    pcl::ModelCoefficients ground_plane_;

    // PCL type point cloud , that holds ground plane points removed cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr background_removed_raw_cloud_PointCloud_;

    // Boss of Point clouds, has many utuility functions used for point cloud perception
    PointCloudManager *pointcloud_manager_;

    // depth camera projection matrix P
    Eigen::MatrixXf TRANS_COLORCAM_TO_IMAGE;

    // Rigid body trand=sform matrix between two coordinate frames, depth to color cams
    Eigen::MatrixXf TRANS_DEPTHCAM_TO_COLORCAM;

    // created with mulplication of Projection and transformation matroces above
    Eigen::MatrixXf TRANS_DEPTHCAM_TO_IMAGE;



   public:
    /**
     * @brief Construct a new Object 3 D Detector object
     *
     * @param pointcloud_manager_
     */
    Object3DDetector(PointCloudManager *pointcloud_manager_);

    /**
     * @brief Destroy the Object 3 D Detector object
     *
     */
    ~Object3DDetector();

    /**
     * @brief inputs segmented image and background removed point cloud , segments point clouds using this segmented
     * image, finally minimal valume boxes are constructed around each segment in segmented point clouds
     *
     * @param raw_rgb_img
     */
    void process3DDetection(const sensor_msgs::ImageConstPtr &segmented_rgb_img);

    /**
     * @brief Subscribes to raw realsense cloud, removes ground plane from cloud
     *
     * @param pcl
     */
    void processPointCloudTasks(const sensor_msgs::PointCloud2ConstPtr &pcl);

    /**
     * @brief Add some collision object to make robot operate safer
     *
     */
    void addCollisionObjects();
};

#endif
