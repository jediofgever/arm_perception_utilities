/**
 * @author [Fetullah Atas]
 * @email [fetulahatas1@gmail.com]
 * @create date 2020-02-18 10:50:56
 * @modify date 2020-02-18 10:50:56
 * @desc [description]
 */
#include <arm_perception_utilities/vision/Object3DDetector.h>
Object3DDetector::Object3DDetector(PointCloudManager *pointcloud_manager) {
    // init  pointers
    nh_ = new ros::NodeHandle();
    listener_ = new tf::TransformListener();
    static const std::string PLANNING_GROUP = "manipulator";
    move_group_ptr_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    pointcloud_manager_ = pointcloud_manager;

    // get topic names from cfg/vision_options.yaml
    nh_->getParam("sub_raw_cloud_topic_name", sub_raw_cloud_topic_name);
    nh_->getParam("sub_segmented_image_topic_name", sub_segmented_image_topic_name);

    // bind Subscribers to the callbacks , raw point cloud , segmented image,
    raw_cloud_sub_ = nh_->subscribe("/camera/depth/points", 1, &Object3DDetector::processPointCloudTasks, this);
    segmented_img_sub_ = nh_->subscribe(sub_segmented_image_topic_name, 1, &Object3DDetector::process3DDetection, this);

    // REALSENSE INSTRINTIC AND EXTRINSIC CAMERA PARAMETERS
    // depth camera projection matrix P(3,4)
    TRANS_COLORCAM_TO_IMAGE = Eigen::MatrixXf::Zero(3, 4);
    TRANS_DEPTHCAM_TO_COLORCAM = Eigen::MatrixXf::Zero(4, 4);
    TRANS_COLORCAM_TO_IMAGE << 577.2959393832757, 0.0, 320.5, -0.0, 0.0, 577.2959393832757, 240.5, 0.0, 0.0, 0.0, 1.0,
        0.0;
    TRANS_DEPTHCAM_TO_COLORCAM << 0.9999990463256836, -0.0005898026865907013, 0.0012526829959824681,
        0.015111126005649567, 0.0005937033565714955, 0.9999949932098389, -0.003115807892754674, 0.00044604094000533223,
        -0.0012508389772847295, 0.003116548527032137, 0.9999943375587463, -0.000181241164682433, 0, 0, 0, 1;
    TRANS_DEPTHCAM_TO_IMAGE = TRANS_COLORCAM_TO_IMAGE * TRANS_DEPTHCAM_TO_COLORCAM;
    // addCollisionObjects();
}

Object3DDetector::~Object3DDetector() {}

/**
 * @brief Add some collision object to make robot operate safer inside a workcell enoirenment
 *
 */
void Object3DDetector::addCollisionObjects() {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_ptr_->getPlanningFrame();
    collision_object.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.0;
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 1.2;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.6;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.8;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    collision_object.id = "box2";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.0;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 1.2;

    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = -0.8;
    box_pose.position.z = 0.8;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    collision_object.id = "box3";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.0;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 1.2;

    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.8;
    box_pose.position.z = 0.8;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);
}

/**
 * @brief Subscribes to raw realsense cloud, removes ground plane from cloud and stores this cloud in
 * background_removed_raw_cloud_PointCloud_
 *
 * @param pcl
 */
void Object3DDetector::processPointCloudTasks(const sensor_msgs::PointCloud2ConstPtr &pcl) {
    // convert sensor_msgs to  pcl::PointCloud and remove  the background
    pcl::PCLPointCloud2 original_cloud_PCLPointCloud2;
    pcl_conversions::toPCL(*pcl, original_cloud_PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud_PointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(original_cloud_PCLPointCloud2, *original_cloud_PointCloud);

    // store groud plane params in ground_plane_ global variable
    ground_plane_ = pointcloud_manager_->removeGroundPlane(original_cloud_PointCloud, original_cloud_PointCloud);

    // store groud plane remove cloud in background_removed_raw_cloud_PointCloud_ global variable
    background_removed_raw_cloud_PointCloud_ = original_cloud_PointCloud;

    // pointcloud_manager_->publishPointCloud2forPoitNETInference(&nonconst_original_cloud_ROSMsg);
}

/**
 * @brief inputs segmented image and background removed point cloud , segments point clouds using this segmented
 * image, finally minimal valume boxes are constructed around each segment in segmented point clouds
 *
 * @param raw_rgb_img
 */
void Object3DDetector::process3DDetection(const sensor_msgs::ImageConstPtr &segmented_rgb_img) {
    cv_bridge::CvImagePtr cv_segmented_ptr;

    /**** RETRIEVE IMAGE MESSAGE AND CONVERT IT TO CV IMAGE************/
    try {
        cv_segmented_ptr = cv_bridge::toCvCopy(segmented_rgb_img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat maskrcnn_masked_img = cv_segmented_ptr->image;
    /******* SEGMENT CLOUD AND PUBLISH IT AS SENSOR  MESG ******/
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters = pointcloud_manager_->segmentCloudfrom2DImage(
        background_removed_raw_cloud_PointCloud_, TRANS_DEPTHCAM_TO_IMAGE, &maskrcnn_masked_img, ground_plane_,
        background_removed_raw_cloud_PointCloud_->header);
    /*
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time taken by segmentCloudfrom2DImage: " << duration.count() << " milliseconds" << std::endl;
    */
    /*************** BUILD 3D BOX AROUND SEGMENTS****/
    // Get starting timepoint
    start = std::chrono::high_resolution_clock::now();
    pointcloud_manager_->minVolObjectBuilder(cloud_clusters);
    /*stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time taken by OBJECT BUILDER: " << duration.count() << " milliseconds" << std::endl;*/
}