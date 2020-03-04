/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 09:59:16
 * @modify date 2019-11-28 09:59:16
 * @desc [description]
 */
#ifndef pointcloudManager_H
#define pointcloudManager_H
#include <cv_bridge/cv_bridge.h>
#include <arm_perception_utilities/utils.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <iostream>
#include "ApproxMVBB/ComputeApproxMVBB.hpp"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace utils;

/**
 * @brief Point Cloud class for point cloud related tasks such as;
 *        * artifical point cloud dataset creation for custom objects
 *        * point cloud segmentation using 2D image(DOPE)
 *        * point cloud segmentation using 3D point cloud(POINTNET)
 *        * point cloud frame transforms, noise addition, down sapling ....
 *        are implemented in this class. The options are loaded from .yaml file
 *        and feeded into parameters using getParam(). options can be modified
 * in cfg/vision_node_options.yaml
 *
 */
class PointCloudManager

{
   private:
    // ROS node handler pointer
    ros::NodeHandle *nh_;

    // we segment cloud from masrcnn segmented image, and publish that segmented cloud with this publisher
    ros::Publisher segmented_cloud_from_maskrcnn_pub_;

    // we detect a 3d box for each object and then publish them as RVIZ markers with ths publisher
    ros::Publisher detected_3d_box_pub_;

    // IGNORE THIS PUBLISHER, DEPRECEATED
    ros::Publisher labeled_cloud_pub_;

    // Arrow markers that defines 6DOF grab pose for each object
    ros::Publisher grab_object_posw_arrow_pub_;

    // publishe segmented point cloud clusters with this publisher
    ros::Publisher clusters_pub_;

    // ROS uses right hand sided frame x:forward , y:left, z:upward
    // original depth cloud is no in ROS frames , so need to recorrect cloud
    sensor_msgs::PointCloud2 corrected_cloud;

    // options loaded from .yaml file
    // pub stands for publish
    std::string camera_frame;
    std::string pub_segmented_cloud_from_maskrcnn_topic_name;
    // sub stands for subscribe

    // minimal object builder
    ApproxMVBB::OOBB oobb;

    // Broadcast transform between object frames and base_link, this should be optional reall
    tf::TransformBroadcaster br;

    // tf transform matrix
    tf::Transform transform;

    // DEPRECECATED BUT KEPT IN CODE NOW
    // a int variable to start point cloud frames for creating artificial point
    // cloud dataset
    // if set to 0 , the craeted file names will start from 0 to number of
    // samples passed to writeDataset()
    int counter = 0;

    // tf listener, that listens to transform between all existing frames
    tf::TransformListener *listener_;

    //corrected Cloud publisher
    ros::Publisher corrected_cloud_pub_;

    // LISTEN TO TRANSFORM AND GET MATRIXES OF TRANSFORM
    tf::StampedTransform camera_depth_optical_frame_to_base_link_transform;

   public:
    /**
     * @brief Construct a new Point Cloud Class object
     *
     */
    PointCloudManager();

    /**
     * @brief Destroy the Point Cloud Class object
     *
     */
    ~PointCloudManager();

    /**
     * @brief DEPCERATED!, used to publih a point cloud that can be fed to Pointnet for instance segmentation
     *
     * @param pcl
     */
    void publishPointCloud2forPoitNETInference(sensor_msgs::PointCloud2 *pcl);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentCloudfrom2DImage(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, Eigen::MatrixXf TRANS_DEPTHCAM_TO_IMAGE,
        cv::Mat *segmented_img, pcl::ModelCoefficients ground_plane, pcl::PCLHeader header);

    /**
     * @brief downsample input cloud
     *
     * @param cloud
     * @param cloud_filtered
     */
    void downsamplePCL(pcl::PCLPointCloud2::Ptr cloud, pcl::PCLPointCloud2::Ptr cloud_filtered);

    /**
     * @brief gets cloud , removes the ground plane and returns found ground plane parameters
     *
     * @param cloud
     * @param cloud_filtered
     * @return pcl::ModelCoefficients
     */
    pcl::ModelCoefficients removeGroundPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered);

    /**
     * @brief checks wheter a given point is on a segmented object, if yes returns true
     *
     * @param segmented_3d_point
     * @return true
     * @return false
     */
    bool isPointonSegmentedObject(pcl::PointXYZRGB segmented_3d_point);

    /**
     * @brief Point cloud smoothing and noise removing utility function
     *
     * @param cloud
     * @param cloud_filtered
     */
    void removeNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered);

    /**
     * @brief function used to get the surface plane of segmented objects in point cloud
     *
     * @param cloud
     * @param cloud_filtered
     * @return pcl::PointCloud<PointXYZRGB>::Ptr
     */
    pcl::PointCloud<PointXYZRGB>::Ptr segmentSurfacePlane(pcl::PointCloud<PointXYZRGB>::Ptr cloud,
                                                          pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered);

    /**
     * @brief builds a 3D box around each object segment  in point cloud, minimal volume object builder libarray at
     * following link is used to achieve minmal volume 3D box;  https://github.com/gabyx/ApproxMVBB
     *
     * @param cloud_clusters
     */
    void minVolObjectBuilder(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters);

    /**
     * @brief simulated depth publishes point clouds in a different frame than
     convential ROS frames, this method recorrect point cloud to right-handed frames
    x;forward, y:left, z:upright
    *
    * @param msg
    * @return sensor_msgs::PointCloud2*
    */
    sensor_msgs::PointCloud2 *recorrectPointcloudCallback( sensor_msgs::PointCloud2* msg) ;


    /**
     * @brief  * @brief simulated depth publishes point clouds in a different frame than
     convential ROS frames, this method recorrect point cloud to right-handed frames
    x;forward, y:left, z:upright 
    * 
    * @param msg 
    * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr recorrectPointcloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg) ;
 
};
#endif