/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:13:44
 * @modify date 2019-11-28 10:13:44
 * @desc [description]
 */
#include <arm_perception_utilities/vision/PointCloudManager.h>
#include <tf/transform_broadcaster.h>
/**
 * @brief Construct a new Point Cloud Class:: Point Cloud Class object
 *
 */
PointCloudManager::PointCloudManager() {
    // init node handler pointer
    nh_ = new ros::NodeHandle();

    // init transform listener pointer
    listener_ = new tf::TransformListener();

    // Load parameters from from options.yaml file
    nh_->getParam("camera_frame", camera_frame);
    nh_->getParam("pub_segmented_cloud_from_maskrcnn_topic_name", pub_segmented_cloud_from_maskrcnn_topic_name);

    // Init publisher for segmented cloud from DOPE
    segmented_cloud_from_maskrcnn_pub_ =
        nh_->advertise<sensor_msgs::PointCloud2>(pub_segmented_cloud_from_maskrcnn_topic_name, 1);

    // Init publisher for corrected cloud into ROS frame convention
    // each point is labeled according to their correspondant classes
    labeled_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("pointnet_inference_cloud", 1);

    // publish segmented point cloud(colorful)
    clusters_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1);

    // Publish Deteceted Obstacles
    detected_3d_box_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("detected_box_pub", 1);

    // publish arrows that defines 6DOF pose object
    grab_object_posw_arrow_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("grab_object_poses", 1);

    corrected_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("corrected_cloud", 1);

}
/**
 * @brief Destroy the Point Cloud Class:: Point Cloud Class object
 *
 */
PointCloudManager::~PointCloudManager() {}

/**
 * @brief input raw point cloud remove noise , downsample and publish it for pointnet inference
 *
 * @param pcl
 */
void PointCloudManager::publishPointCloud2forPoitNETInference(sensor_msgs::PointCloud2 *pcl) {
    /***** DEPRECETED BUT KEPT FOR NOW ******/
    pcl::PCLPointCloud2::Ptr original_cloud_PCLPointCloud2(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*pcl, *original_cloud_PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud_PointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2::Ptr downsampled_cloud_PCLPointCloud2(new pcl::PCLPointCloud2);
    downsamplePCL(original_cloud_PCLPointCloud2, downsampled_cloud_PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud_PointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromPCLPointCloud2(*downsampled_cloud_PCLPointCloud2, *downsampled_cloud_PointCloud);

    removeNoise(downsampled_cloud_PointCloud, downsampled_cloud_PointCloud);

    // convert labeled cloud to ros type and publish for visualiztion
    sensor_msgs::PointCloud2 downsampled_cloud_4096_points_ROSmsg;
    downsampled_cloud_4096_points_ROSmsg.header = pcl->header;
    while (downsampled_cloud_PointCloud->points.size() > 4096) {
        downsampled_cloud_PointCloud->points.pop_back();
    }
    downsampled_cloud_PointCloud->width = 1;
    downsampled_cloud_PointCloud->height = downsampled_cloud_PointCloud->points.size();
    // convert it to ROS message type
    pcl::toROSMsg(*downsampled_cloud_PointCloud, downsampled_cloud_4096_points_ROSmsg);
    pcl::PCLPointCloud2::Ptr labeled_4096_points_cloud_PCLPointCloud2(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*downsampled_cloud_PointCloud, *labeled_4096_points_cloud_PCLPointCloud2);

    std::string this_ply_file_path = "~/pcl_dataset/" + std::to_string(counter) + ".pcd";

    downsampled_cloud_4096_points_ROSmsg.header.frame_id = camera_frame;
    downsampled_cloud_4096_points_ROSmsg.header.stamp = ros::Time::now();

    // publish cloud
    labeled_cloud_pub_.publish(downsampled_cloud_4096_points_ROSmsg);
    ROS_INFO(
        "PUBLISHED CLOUD WITH %d POINTS FOR POINTNET"
        " INFERENCE",
        downsampled_cloud_PointCloud->points.size());
}

/**
 * @brief segmented image comes from MASKRCNN, projects in_cloud onto segmented_img, uses TRANS_DEPTHCAM_TO_IMAGE to do
 * this transform. Ignores blank pixels in segmented_img, other pixels color values are assigned to in_cloud points. We
 * get a colorful point cloud where each color indicates a objects identity.
 *
 *
 * @param in_cloud
 * @param TRANS_DEPTHCAM_TO_IMAGE
 * @param segmented_img
 * @param header
 * @return std::vector<PointICloudPtr>
 */

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PointCloudManager::segmentCloudfrom2DImage(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, Eigen::MatrixXf TRANS_DEPTHCAM_TO_IMAGE, cv::Mat *segmented_img,
    pcl::ModelCoefficients ground_plane, pcl::PCLHeader header) {
    // convert input cloud to Eigen matrix, because we need to project this
    // cloud into 2D image using matrix transformation
    Eigen::MatrixXf matrix_velodyne_points_in_cam_frame = Eigen::MatrixXf::Zero(4, in_cloud->size());
#pragma omp parallel for
    for (int i = 0; i < in_cloud->size(); ++i) {
        matrix_velodyne_points_in_cam_frame(0, i) = in_cloud->points[i].x;
        matrix_velodyne_points_in_cam_frame(1, i) = in_cloud->points[i].y;
        matrix_velodyne_points_in_cam_frame(2, i) = in_cloud->points[i].z;
        matrix_velodyne_points_in_cam_frame(3, i) = 1;
    }

    // PROJECT POINT CLOUD TO IMAGE PPLANE
    Eigen::MatrixXf image_points_as_matrix =
        utils::transform3DPointsToImage(matrix_velodyne_points_in_cam_frame, TRANS_DEPTHCAM_TO_IMAGE);

    // Stores points that corresponds to colorful pixel in image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> pointcloud_ids;

    // BEGIN  FOR LOOP TO DETECT SEGMENTED POINT CLOUD FROM IMAGE PLANE
#pragma omp parallel for
    for (int m = 0; m < image_points_as_matrix.cols(); m++) {
        cv::Point point;

        // Mulpily by two since image was resized to be double size
        point.x = image_points_as_matrix(0, m);
        point.y = image_points_as_matrix(1, m);

        // Store korners in pixels only of they are on image plane
        if (point.x > 0 && point.x < segmented_img->cols) {
            if (point.y > 0 && point.y < segmented_img->rows) {
                pcl::PointXYZRGB segmented_3d_point;
                cv::Vec3b rgb_pixel = segmented_img->at<cv::Vec3b>(point.y, point.x);
                // Get x,y,z values of this point from original raw point cloud
                segmented_3d_point.x = matrix_velodyne_points_in_cam_frame(0, m);
                segmented_3d_point.y = matrix_velodyne_points_in_cam_frame(1, m);
                segmented_3d_point.z = matrix_velodyne_points_in_cam_frame(2, m);

                // get r,g,b value of this point from segmented image
                segmented_3d_point.r = rgb_pixel[2];
                segmented_3d_point.g = rgb_pixel[1];
                segmented_3d_point.b = rgb_pixel[0];

                 //cv::circle(*segmented_img, point, 1, cv::Scalar(segmented_3d_point.x*segmented_3d_point.z, 255, segmented_3d_point.z*segmented_3d_point.z), 2, 8, 0);

                if (isPointonSegmentedObject(segmented_3d_point)) {
                    // This point on image is NOT white SO, IT MUST BE BELONG TO
                    // SOME OBJECT the background of segmented image is white
                    int id = segmented_3d_point.r * segmented_3d_point.b * segmented_3d_point.g;
                    pointcloud_ids.push_back(id);
                    segmented_cloud->points.push_back(segmented_3d_point);
                }
            }
        }
    }
    // debug image to control if fusion was correct
     //cv::imwrite("/home/atas/fucintel.png", *segmented_img);
    /******END  FOR LOOP TO DETECT SEGMENTED POINT CLOUD FROM IMAGE
     * PLANE ****************/

    // find how many unique objects we have, only remain unquie ids , remove duplicates
    std::sort(pointcloud_ids.begin(), pointcloud_ids.end());
    pointcloud_ids.erase(std::unique(pointcloud_ids.begin(), pointcloud_ids.end()), pointcloud_ids.end());

    // initiliaze empty point cloud pointers, they are going to be filled with
    // the points belonging to same color
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;

    // initalize a vector for point cloud clusters, each cluster has its own pcl::PointCloud
    for (int k = 0; k < pointcloud_ids.size(); k++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_clusters.push_back(cld_ptr);
    }
    std_msgs::Header std_header;

    std_header.frame_id = header.frame_id;
    std_header.seq = header.seq;
    std_header.stamp = ros::Time::now();

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*in_cloud, msg);
    msg.header = std_header;
    msg =  *recorrectPointcloudCallback(&msg);
    segmented_cloud = recorrectPointcloudCallback(segmented_cloud);
    corrected_cloud_pub_.publish(msg);


// iterate through each segmented point in order to find which cluster they
// actually belongs to
#pragma omp parallel for
    for (int i = 0; i < segmented_cloud->points.size(); i++) {
        pcl::PointXYZRGB point_to_ROS_frames;

        point_to_ROS_frames = segmented_cloud->points.at(i);

        // ignore points below ground plane
        if (segmented_cloud->points.at(i).z > (-ground_plane.values[3])) {
            //continue;
        }

        // iterate through each cluster in order to find right cluster for this
        // point (point_to_ROS_frames)
        for (int p = 0; p < pointcloud_ids.size(); p++) {
            // chechk this points id to existing unique cluster ids
            if ((point_to_ROS_frames.r * point_to_ROS_frames.g * point_to_ROS_frames.b) == pointcloud_ids[p]) {
                // this point belongs to cloud_clusters(pointcloud_ids.at(p)) so
                // assing and append this point(point_to_ROS_frames) to this
                // cluster(cloud_clusters[p])
                cloud_clusters[p]->points.push_back(point_to_ROS_frames);
            }
        }
    }

    // the cloud can be just unordered
    segmented_cloud->width = 1;
    segmented_cloud->height = segmented_cloud->points.size();

    for (int k = 0; k < cloud_clusters.size(); k++) {
        cloud_clusters[k]->width = 1;
        cloud_clusters[k]->height = cloud_clusters[k]->points.size();
        if (cloud_clusters[k]->points.size() > 0) {
            cloud_clusters[k] = segmentSurfacePlane(cloud_clusters[k], cloud_clusters[k]);
        }
    }


    utils::publishClustersCloud(clusters_pub_, std_header, cloud_clusters);

    // init ros message for segmented clouud, to be published
    sensor_msgs::PointCloud2 segmented_cloud_msg;
    segmented_cloud_msg.header.frame_id = in_cloud->header.frame_id;  // camera_frame;
                                                                      // convert it to ros message from pcl
    pcl::toROSMsg(*segmented_cloud, segmented_cloud_msg);

    // publish segmented cloud
    segmented_cloud_msg.header = std_header;
    segmented_cloud_from_maskrcnn_pub_.publish(segmented_cloud_msg);
    // return vector of point clouds which contains all clusters.
    // this is for finding 3D bounding box around each cluster
    return cloud_clusters;
}

/**
 * @brief if this point on a white pixle return false, if this point on non-white color return true
 *
 * @param segmented_3d_point
 * @return true
 * @return false
 */
bool PointCloudManager::isPointonSegmentedObject(pcl::PointXYZRGB segmented_3d_point) {
    if (segmented_3d_point.g != 255 && segmented_3d_point.b != 255 != segmented_3d_point.r != 255) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief input object clusters, builds a minimal volume 3d box around each cluster, publishes tyhis 3d box as marker,
 * normal vector of each clusater is alsopublished as an arrow marker
 *
 * @param cloud_clusters
 */
void PointCloudManager::minVolObjectBuilder(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters) {
    // RVIZ 3D BOX marker for visualization
    visualization_msgs::MarkerArray object_box_array;
    // RCIZ ARROW marker to define 6DOF pose of objects
    visualization_msgs::MarkerArray grab_object_pose_arrow_array;

    // iterate through each CLUSTER
    for (int k = 0; k < cloud_clusters.size(); k++) {
        // FEED points of this cluster into points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr this_cluster = cloud_clusters.at(k);
        ApproxMVBB::Matrix3Dyn points;

        points.resize(3, this_cluster->points.size());
        if (this_cluster->points.size() < 10) {
            return;
        }
#pragma omp parallel for
        for (int i = 0; i < this_cluster->points.size(); i++) {
            points(0, i) = this_cluster->points[i].x;
            points(1, i) = this_cluster->points[i].y;
            points(2, i) = this_cluster->points[i].z;
        }

        // buiuld themininmal volume box here
        oobb = ApproxMVBB::approximateMVBB(points, 0.0, points.size() / 5, 0.0, 0, 1);

        // In some cases the box doesnt cover all points of this cluster, this step is intend to fix that issue
        ApproxMVBB::Matrix33 A_KI = oobb.m_q_KI.matrix().transpose();
        auto size = points.cols();
#pragma omp parallel for
        for (unsigned int i = 0; i < size; ++i) {
            oobb.unite(A_KI * points.col(i));
        }

        // get corners of builded object
        ApproxMVBB::TypeDefsPoints::Vector3List corners = oobb.getCornerPoints();
        visualization_msgs::Marker object_box_;

        // this object is camera frame(depth camera frmae to be precise)
        object_box_.type = visualization_msgs::Marker::LINE_STRIP;
        object_box_.header.frame_id = "camera_link";
        object_box_.header.stamp = ros::Time::now();
        object_box_.ns = "DetectionBox";
        object_box_.id = k;
        object_box_.action = visualization_msgs::Marker::ADD;
        object_box_.lifetime = ros::Duration(1.2);

        utils::SetMarkerData(&object_box_, 0, 0, 0, 0, 0, 0, 1.0, 0.005, 0, 0, 1, 0, 1, 1);
        std::vector<geometry_msgs::Point> corners_geometry_msgs = utils::Vector3ListGeometryMsgs(corners);

        // ADD LINE STRIPES TO CONNECT EACH 8 CORNERS OF 3D BOX, there is 12 lines that construct a 3D box
        object_box_.points.push_back(corners_geometry_msgs.at(0));
        object_box_.points.push_back(corners_geometry_msgs.at(1));
        object_box_.points.push_back(corners_geometry_msgs.at(3));
        object_box_.points.push_back(corners_geometry_msgs.at(0));
        object_box_.points.push_back(corners_geometry_msgs.at(4));
        object_box_.points.push_back(corners_geometry_msgs.at(1));
        object_box_.points.push_back(corners_geometry_msgs.at(2));
        object_box_.points.push_back(corners_geometry_msgs.at(1));
        object_box_.points.push_back(corners_geometry_msgs.at(5));
        object_box_.points.push_back(corners_geometry_msgs.at(1));
        object_box_.points.push_back(corners_geometry_msgs.at(2));
        object_box_.points.push_back(corners_geometry_msgs.at(6));
        object_box_.points.push_back(corners_geometry_msgs.at(2));
        object_box_.points.push_back(corners_geometry_msgs.at(3));
        object_box_.points.push_back(corners_geometry_msgs.at(3));
        object_box_.points.push_back(corners_geometry_msgs.at(7));
        object_box_.points.push_back(corners_geometry_msgs.at(7));
        object_box_.points.push_back(corners_geometry_msgs.at(4));
        object_box_.points.push_back(corners_geometry_msgs.at(7));
        object_box_.points.push_back(corners_geometry_msgs.at(6));
        object_box_.points.push_back(corners_geometry_msgs.at(4));
        object_box_.points.push_back(corners_geometry_msgs.at(5));
        object_box_.points.push_back(corners_geometry_msgs.at(5));
        object_box_.points.push_back(corners_geometry_msgs.at(6));
        // PUSH EACH BOX INTO ARRAY TO BE PUBLISHED IN THE ENMD OF LOOP
        object_box_array.markers.push_back(object_box_);

        // FOR NORMAL VECTOR OF OBJECT
        visualization_msgs::Marker grab_object_pose_arrow_;

        grab_object_pose_arrow_.type = visualization_msgs::Marker::ARROW;
        grab_object_pose_arrow_.header.frame_id = "base_link";
        grab_object_pose_arrow_.header.stamp = ros::Time::now();
        grab_object_pose_arrow_.ns = "camera_link";
        grab_object_pose_arrow_.id = k;
        grab_object_pose_arrow_.action = visualization_msgs::Marker::ADD;
        grab_object_pose_arrow_.lifetime = ros::Duration(1.2);

        // MINIMAL CORNER OF BOX IS STORED IN FIRST ELEMENT, MAXIMAL CORNER OF BOX IS STORED IN LAST ELEMENT OF KORNERS
        // ARRAY
        geometry_msgs::Point start_point_in_camera_depth_optical_frame_msg, end_point_in_camera_depth_optical_frame_msg;
        end_point_in_camera_depth_optical_frame_msg = corners_geometry_msgs.at(0);
        start_point_in_camera_depth_optical_frame_msg = corners_geometry_msgs.at(7);

        // TO FIND NORMAL VECTOR OF OBJECT WE NEED AT LEAST ONE POINT ON SURFACE AND AT LEAST ONBE POINT IN BOTTOM OF
        // OBJECT
        geometry_msgs::Point cross_end_point_in_camera_depth_optical_frame_msg,
            cross_start_point_in_camera_depth_optical_frame_msg;

        // GET CLOSEST CORNERS TO MIN MAX CORNERS TO FIND NORMAL VECTOR
        cross_end_point_in_camera_depth_optical_frame_msg = utils::findNearestCorner(0, corners_geometry_msgs);
        cross_start_point_in_camera_depth_optical_frame_msg = utils::findNearestCorner(7, corners_geometry_msgs);

        // THIS POINTS ARE ON CAMERA  FRAME, BUT WE WANT NORMAL VECTOR TO BE IN  BASE_LINKFRAME OF ROBOT, SO PICK AND
        // PLACE CAN BE DONE ONTHIS NORMAL VECTOR
        tf::Point start_point_in_camera_depth_optical_frame_tf, end_point_in_camera_depth_optical_frame_tf;
        tf::Point cross_start_point_in_camera_depth_optical_frame_tf, cross_end_point_in_camera_depth_optical_frame_tf;

        // ROS MSG TO TF
        tf::pointMsgToTF(start_point_in_camera_depth_optical_frame_msg, start_point_in_camera_depth_optical_frame_tf);
        tf::pointMsgToTF(end_point_in_camera_depth_optical_frame_msg, end_point_in_camera_depth_optical_frame_tf);
        tf::pointMsgToTF(cross_start_point_in_camera_depth_optical_frame_msg,
                         cross_start_point_in_camera_depth_optical_frame_tf);
        tf::pointMsgToTF(cross_end_point_in_camera_depth_optical_frame_msg,
                         cross_end_point_in_camera_depth_optical_frame_tf);

        // LISTEN TO TRANSFORM AND GET MATRIXES OF TRANSFORM
        tf::StampedTransform camera_depth_optical_frame_to_base_link_transform;
        // lookup transform (this should be cached, since it’s probably static)
        listener_->lookupTransform("base_link", "camera_link", ros::Time(0.0f),
                                   camera_depth_optical_frame_to_base_link_transform);

        tf::Point start_point_in_base_link_tf, end_point_in_base_link_tf;
        tf::Point cross_start_point_in_base_link_tf, cross_end_point_in_base_link_tf;

        // GET POINTS IN BASE_LINK FRAME OF ROBOT
        start_point_in_base_link_tf =
            camera_depth_optical_frame_to_base_link_transform * start_point_in_camera_depth_optical_frame_tf;
        end_point_in_base_link_tf =
            camera_depth_optical_frame_to_base_link_transform * end_point_in_camera_depth_optical_frame_tf;
        cross_start_point_in_base_link_tf =
            camera_depth_optical_frame_to_base_link_transform * cross_start_point_in_camera_depth_optical_frame_tf;
        cross_end_point_in_base_link_tf =
            camera_depth_optical_frame_to_base_link_transform * cross_end_point_in_camera_depth_optical_frame_tf;

        // CONVERT THEM BACK TO ROS MSG
        geometry_msgs::Point start_point_in_base_link_msg, end_point_in_base_link_msg;
        geometry_msgs::Point cross_start_point_in_base_link_msg, cross_end_point_in_base_link_msg;

        tf::pointTFToMsg(start_point_in_base_link_tf, start_point_in_base_link_msg);
        tf::pointTFToMsg(end_point_in_base_link_tf, end_point_in_base_link_msg);
        tf::pointTFToMsg(cross_start_point_in_base_link_tf, cross_start_point_in_base_link_msg);
        tf::pointTFToMsg(cross_end_point_in_base_link_tf, cross_end_point_in_base_link_msg);

        /*
        BELOW IS AN ILLUSTRAION OF OBJECT BOX
        THEULTIMATE GOAL IS TO FIND ARROW END POINT(a) AND ARROW START POINT(b)
                  ^
                  |
            *-------------*
          / -     |      /-
         /  -    *| a   / -
        *--------------*  -
        -   -     |   -   -
        -   -     |   -   -
        -   *-------------*
        -  /     *|b  -  /
        - /           - /
        *--------------*


        */
        geometry_msgs::Point arrow_start_point, arrow_end_point;
        geometry_msgs::Point arrow_cross_start_point, arrow_cross_end_point;

        arrow_start_point = start_point_in_base_link_msg;
        arrow_end_point = end_point_in_base_link_msg;
        arrow_cross_start_point = cross_start_point_in_base_link_msg;
        arrow_cross_end_point = cross_end_point_in_base_link_msg;

        // SEE ABOVE FIGURE TOMAKE SENSE OF ARROW END START POINTS
        arrow_end_point.x = (arrow_end_point.x + arrow_cross_start_point.x) / 2;
        arrow_end_point.y = (arrow_end_point.y + arrow_cross_start_point.y) / 2;
        arrow_end_point.z = (arrow_end_point.z + arrow_cross_start_point.z) / 2;

        arrow_start_point.x = (arrow_start_point.x + arrow_cross_end_point.x) / 2;
        arrow_start_point.y = (arrow_start_point.y + arrow_cross_end_point.y) / 2;
        arrow_start_point.z = (arrow_start_point.z + arrow_cross_end_point.z) / 2;

        if (arrow_start_point.z > arrow_end_point.z) {
            std::swap(arrow_end_point, arrow_start_point);
        }

        geometry_msgs::Point32 diff;
        diff.x = arrow_end_point.x - arrow_start_point.x;
        diff.y = arrow_end_point.y - arrow_start_point.y;
        diff.z = arrow_end_point.z - arrow_start_point.z;

        arrow_start_point.x += 10 * diff.x;
        arrow_start_point.y += 10 * diff.y;
        arrow_start_point.z += 10 * diff.z;

        grab_object_pose_arrow_.points.push_back(arrow_start_point);
        grab_object_pose_arrow_.points.push_back(arrow_end_point);
        // FIND ORIENATTAION OF NORMAL VECTOR AND PLACE A FRAME ON START POINT OF NORMAL VECTOR
        float rot_x = std::atan2(arrow_start_point.z - arrow_end_point.z, arrow_start_point.x - arrow_end_point.x);
        float rot_y = std::atan2(arrow_start_point.z - arrow_end_point.z, arrow_start_point.y - arrow_end_point.y);
        float rot_z = std::atan2(arrow_start_point.x - arrow_end_point.x, arrow_start_point.y - arrow_end_point.y);

        transform.setOrigin(tf::Vector3(arrow_start_point.x, arrow_start_point.y, arrow_start_point.z));
        transform.setRotation(tf::Quaternion(-rot_x + 1.57, rot_y + 1.57, 0));

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", std::to_string(k)));

        /// RELATED TO THE VISUAL PROPERTIES OF ARROW
        grab_object_pose_arrow_.scale.x = 0.03;
        grab_object_pose_arrow_.scale.y = 0.045;
        grab_object_pose_arrow_.scale.z = 0.03;
        grab_object_pose_arrow_.color.r = 1;
        grab_object_pose_arrow_.color.g = 0;
        grab_object_pose_arrow_.color.b = 0;
        grab_object_pose_arrow_.color.a = 1.0;

        grab_object_pose_arrow_array.markers.push_back(grab_object_pose_arrow_);
    }
    detected_3d_box_pub_.publish(object_box_array);
    grab_object_posw_arrow_pub_.publish(grab_object_pose_arrow_array);
}

/**
 * @brief downsample cloud
 *
 * @param cloud
 * @param cloud_filtered
 */
void PointCloudManager::downsamplePCL(pcl::PCLPointCloud2::Ptr cloud, pcl::PCLPointCloud2::Ptr cloud_filtered) {
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);
}

/**
 * @brief DETECT GROUNDPLANE AND REMOVE POINTS FALLING INTO GROUND PLANE
 *
 * @param cloud
 * @param cloud_filtered
 * @return pcl::ModelCoefficients
 */
pcl::ModelCoefficients PointCloudManager::removeGroundPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered) {
    pcl::PointIndicesPtr ground(new pcl::PointIndices);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    cloud_filtered->header = cloud->header;
    cloud_filtered->height = 1;
    cloud_filtered->width = cloud_filtered->points.size();

    return *coefficients;
}

/**
 * @brief REMOVES OUTLIERS FROM GIVEN CLOUD
 *
 * @param cloud
 * @param cloud_filtered
 */
void PointCloudManager::removeNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered) {
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;

    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    sor.setNegative(false);
    sor.filter(*cloud_filtered);
}

/**
 * @brief DETECT SURFACE OF GIVEN POINT CLOUD
 * REMOVES POINTS WHICH ARE NOT ON THIS DETECTED SURFACE
 *
 * @param cloud
 * @param cloud_filtered
 * @return pcl::PointCloud<PointXYZRGB>::Ptr
 */
pcl::PointCloud<PointXYZRGB>::Ptr PointCloudManager::segmentSurfacePlane(
    pcl::PointCloud<PointXYZRGB>::Ptr cloud, pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointXYZRGB> seg;
    pcl::ExtractIndices<PointXYZRGB> extract;

    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.008);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);

    return cloud_filtered;
}

/**
 * @brief simulated depth publishes point clouds in a different frame than
 convential ROS frames, this method recorrect point cloud to right-handed frames
 x;forward, y:left, z:upright
 *
 * @param msg
 * @return sensor_msgs::PointCloud2*
 */
sensor_msgs::PointCloud2 *PointCloudManager::recorrectPointcloudCallback( sensor_msgs::PointCloud2* msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> temp_cloud_in_ros_frames;
    pcl::PCLPointCloud2 pcl2in_ros_frames;

    for (int i = 0; i < temp_cloud->points.size(); i++) {
        pcl::PointXYZRGB point_to_ROS_frames;
        point_to_ROS_frames.x = temp_cloud->points.at(i).z;
        point_to_ROS_frames.y = -temp_cloud->points.at(i).x;
        point_to_ROS_frames.z = -temp_cloud->points.at(i).y;

        point_to_ROS_frames.r = temp_cloud->points.at(i).r;
        point_to_ROS_frames.g = temp_cloud->points.at(i).g;
        point_to_ROS_frames.b = temp_cloud->points.at(i).b;

        temp_cloud_in_ros_frames.points.push_back(point_to_ROS_frames);
    }

    pcl::toPCLPointCloud2(temp_cloud_in_ros_frames, pcl2in_ros_frames);
    pcl_conversions::fromPCL(pcl2in_ros_frames, corrected_cloud);
    corrected_cloud.header.frame_id = camera_frame;
    corrected_cloud.header.stamp = ros::Time::now();

    return &corrected_cloud;
}


/**
 * @brief simulated depth publishes point clouds in a different frame than
 convential ROS frames, this method recorrect point cloud to right-handed frames
 x;forward, y:left, z:upright
 *
 * @param msg
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManager::recorrectPointcloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_in_ros_frames(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 pcl2in_ros_frames;

    for (int i = 0; i < cloud->points.size(); i++) {
        pcl::PointXYZRGB point_to_ROS_frames;
        point_to_ROS_frames.x = cloud->points.at(i).z;
        point_to_ROS_frames.y = -cloud->points.at(i).x;
        point_to_ROS_frames.z = -cloud->points.at(i).y;

        point_to_ROS_frames.r = cloud->points.at(i).r;
        point_to_ROS_frames.g = cloud->points.at(i).g;
        point_to_ROS_frames.b = cloud->points.at(i).b;

        temp_cloud_in_ros_frames->points.push_back(point_to_ROS_frames);
    }
 
 
    return temp_cloud_in_ros_frames;
}