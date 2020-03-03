/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 10:14:30
 * @modify date 2019-11-28 10:14:30
 * @desc [description]
 */
#include <arm_perception_utilities/utils.h>

/**
 * @brief
 * A namespace to keep utility functions
 */
namespace utils {

/**
 * @brief
 * round after 2 decimals
 * @param var
 * @return double
 */
double round(double var) {
    double value = (int)(var * 1000 + .5);
    return (double)value / 1000;
}

/**
 * @brief
 * given euler angles in radian, returns geometry_msgs::Quaternion ROS type
 * @param robot_rx_deg
 * @param robot_ry_deg
 * @param robot_rz_deg
 * @param kDEG2RAD
 * @return geometry_msgs::Quaternion
 */
geometry_msgs::Quaternion EulertoQuaternion(double robot_rx_deg, double robot_ry_deg, double robot_rz_deg,
                                            double kDEG2RAD) {
    tf2::Quaternion robot_goal_orientation_quat;
    robot_goal_orientation_quat.setRPY(
        robot_rx_deg * kDEG2RAD, robot_ry_deg * kDEG2RAD,
        robot_rz_deg * kDEG2RAD);  // Create this quaternion from roll/pitch/yaw (in radians)

    // Normalize the QUATs to make the squarred root sum of x,y,z,w equal to 1.0
    robot_goal_orientation_quat.normalize();

    // Robot Pose should be set in terms of ROS conventations geotmetry_msgs::Pose
    geometry_msgs::Quaternion robot_goal_orientation_geo_msg;

    // tf::Quaternions to geotmetry_msgs::Quaternion
    robot_goal_orientation_geo_msg = tf2::toMsg(robot_goal_orientation_quat);

    return robot_goal_orientation_geo_msg;
}

/**
 * @brief
 *
 * @param corners
 * @param image
 */
void construct3DBoxOnImage(std::vector<std::vector<double>> korners, cv::Mat *image) {
    cv::Scalar clr = cv::Scalar(0, 0, 255);
    cv::Scalar clr_b = cv::Scalar(0, 0, 255);
    cv::Scalar clr_ta = cv::Scalar(0, 0, 255);

    // Declare cv Point to keep pixel coordinatres
    // Declare image_points vector to keep image coordinates
    // returned by CameraReproh->Project3Dpoint
    std::vector<cv::Point> image_points;

    for (int i = 0; i < korners.size(); i++) {
        cv::Point image_point;
        image_point.x = korners.at(i)[0];
        image_point.y = korners.at(i)[1];
        image_points.push_back(image_point);
    }

    // Draw 12 lines that costructs box

    if (image_points.size() > 7) {
        cv::line(*image, image_points[0], image_points[1], clr_b, 2, 8);
        cv::line(*image, image_points[0], image_points[3], clr, 2, 8);
        cv::line(*image, image_points[0], image_points[4], clr_ta, 2, 8);
        cv::line(*image, image_points[1], image_points[2], clr, 2, 8);
        cv::line(*image, image_points[1], image_points[5], clr_ta, 2, 8);
        cv::line(*image, image_points[2], image_points[6], clr_ta, 2, 8);
        cv::line(*image, image_points[2], image_points[3], clr_b, 2, 8);
        cv::line(*image, image_points[3], image_points[7], clr_ta, 2, 8);
        cv::line(*image, image_points[7], image_points[4], clr, 2, 8);
        cv::line(*image, image_points[7], image_points[6], clr_b, 2, 8);
        cv::line(*image, image_points[4], image_points[5], clr_b, 2, 8);
        cv::line(*image, image_points[5], image_points[6], clr, 2, 8);
    }
}

void SetMarkerData(visualization_msgs::Marker *marker, double px, double py, double pz, double ox, double oy, double oz,
                   double ow, double sx, double sy, double sz, double r, double g, double b, double a) {
    marker->pose.position.x = px;
    marker->pose.position.y = py;
    marker->pose.position.z = pz;

    marker->pose.orientation.x = ox;
    marker->pose.orientation.y = oy;
    marker->pose.orientation.z = oz;
    marker->pose.orientation.w = ow;

    marker->scale.x = sx;
    marker->scale.y = sy;
    marker->scale.z = sz;

    marker->color.r = r;
    marker->color.g = g;
    marker->color.b = b;
    marker->color.a = a;
}

Eigen::MatrixXf transform3DPointsToImage(const Eigen::MatrixXf &rect_cam_points,
                                         Eigen::MatrixXf TRANS_RECTCAM_TO_IMAGE) {
    Eigen::MatrixXf image_points = TRANS_RECTCAM_TO_IMAGE * rect_cam_points;
    Eigen::MatrixXf uv = Eigen::MatrixXf::Zero(3, rect_cam_points.cols());
    uv.row(0) = image_points.row(0).array() / image_points.row(2).array();
    uv.row(1) = image_points.row(1).array() / image_points.row(2).array();
    uv.row(2) = image_points.row(2);
    return uv;
}

visualization_msgs::MarkerArray jsk2Marker(jsk_recognition_msgs::BoundingBoxArray jsk_lbl) {
    double object_dim_x, object_dim_y, object_dim_z;

    object_dim_x = 0.05;
    object_dim_y = 0.112;
    object_dim_z = 0.112;
    visualization_msgs::MarkerArray marker_array;

    for (int i = 0; i < jsk_lbl.boxes.size(); i++) {
        std::vector<geometry_msgs::Point> corners_geometry_msgs;
        visualization_msgs::Marker visualization_marker_;

        visualization_marker_.type = visualization_msgs::Marker::LINE_STRIP;
        visualization_marker_.header.frame_id = "camera_link";
        visualization_marker_.header.stamp = ros::Time::now();
        visualization_marker_.ns = "3DGTBox";
        visualization_marker_.id = i;
        visualization_marker_.action = visualization_msgs::Marker::ADD;
        visualization_marker_.lifetime = ros::Duration(1.0);

        utils::SetMarkerData(&visualization_marker_, 0, 0, 0, 0, 0, 0, 0, 0.005, 0, 0, 0, 0, 1, 1);

        double x, y, z, l, w, h, alpha;
        geometry_msgs::Point p_1, p_2, p_3, p_4, p_5, p_6, p_7, p_8;
        x = jsk_lbl.boxes.at(i).pose.position.x;
        y = jsk_lbl.boxes.at(i).pose.position.y;
        z = jsk_lbl.boxes.at(i).pose.position.z;

        p_1.x = x - object_dim_x / 2;
        p_1.y = y + object_dim_y / 2;
        p_1.z = z - object_dim_z / 2;

        p_2.x = x - object_dim_x / 2;
        p_2.y = y - object_dim_y / 2;
        p_2.z = z - object_dim_z / 2;

        p_3.x = x + object_dim_x / 2;
        p_3.y = y - object_dim_y / 2;
        p_3.z = z - object_dim_z / 2;

        p_4.x = x + object_dim_x / 2;
        p_4.y = y + object_dim_y / 2;
        p_4.z = z - object_dim_z / 2;

        p_5.x = x - object_dim_x / 2;
        p_5.y = y + object_dim_y / 2;
        p_5.z = z + object_dim_z / 2;

        p_6.x = x - object_dim_x / 2;
        p_6.y = y - object_dim_y / 2;
        p_6.z = z + object_dim_z / 2;

        p_7.x = x + object_dim_x / 2;
        p_7.y = y - object_dim_y / 2;
        p_7.z = z + object_dim_z / 2;

        p_8.x = x + object_dim_x / 2;
        p_8.y = y + object_dim_y / 2;
        p_8.z = z + object_dim_z / 2;

        corners_geometry_msgs.push_back(p_1);
        corners_geometry_msgs.push_back(p_2);
        corners_geometry_msgs.push_back(p_3);
        corners_geometry_msgs.push_back(p_4);
        corners_geometry_msgs.push_back(p_5);
        corners_geometry_msgs.push_back(p_6);
        corners_geometry_msgs.push_back(p_7);
        corners_geometry_msgs.push_back(p_8);

        visualization_marker_.points.push_back(corners_geometry_msgs.at(0));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(1));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(0));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(3));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(0));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(4));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(1));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(2));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(1));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(5));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(1));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(2));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(6));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(2));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(3));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(3));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(7));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(7));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(4));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(7));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(6));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(4));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(5));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(5));
        visualization_marker_.points.push_back(corners_geometry_msgs.at(6));

        marker_array.markers.push_back(visualization_marker_);
    }

    return marker_array;
}

double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

std::vector<geometry_msgs::Point> Vector3ListGeometryMsgs(ApproxMVBB::TypeDefsPoints::Vector3List corners) {
    std::vector<geometry_msgs::Point> corners_geometry_msgs;

    for (int i = 0; i < corners.size(); i++) {
        geometry_msgs::Point korner_point;

        korner_point.x = corners[i].x();
        korner_point.y = corners[i].y();
        korner_point.z = corners[i].z();
        corners_geometry_msgs.push_back(korner_point);
    }
    return corners_geometry_msgs;
}

geometry_msgs::Point findNearestCorner(int current_point_indice,
                                       std::vector<geometry_msgs::Point> corners_geometry_msgs) {
    geometry_msgs::Point nearest_corner;
    double nearest_point_distance = 100.0;
    for (int i = 0; i < corners_geometry_msgs.size(); i++) {
        if (i != current_point_indice) {
            double dist_to_current_point =
                std::sqrt(std::pow((corners_geometry_msgs[current_point_indice].x - corners_geometry_msgs[i].x), 2) +
                          std::pow((corners_geometry_msgs[current_point_indice].y - corners_geometry_msgs[i].y), 2) +
                          std::pow((corners_geometry_msgs[current_point_indice].z - corners_geometry_msgs[i].z), 2));
            if (dist_to_current_point < nearest_point_distance) {
                nearest_point_distance = dist_to_current_point;
                nearest_corner = corners_geometry_msgs[i];
            }
        }
    }
    return nearest_corner;
}

/**
 * @brief publish clustering objects' in one point cloud
 * @param publisher
 * @param header
 * @param cloud_clusters
 * @param trans
 */
void publishClustersCloud(const ros::Publisher &publisher, const std_msgs::Header &header,
                          const std::vector<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters_array) {
    if (clusters_array.size() <= 0) {
        ROS_WARN("Publish empty clusters cloud.");
        // publish empty cloud
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(*(new pcl::PointCloud<pcl::PointXYZRGB>), msg_cloud);
        msg_cloud.header = header;
        publisher.publish(msg_cloud);
        return;
    } else {
        ROS_INFO_STREAM("Publishing " << clusters_array.size() << " clusters in one cloud.");
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // different clusters with different intensity
    float step_i = 255.0f / clusters_array.size();
    for (size_t cluster_idx = 0u; cluster_idx < clusters_array.size(); ++cluster_idx) {
        if (clusters_array[cluster_idx]->points.size() <= 0) {
            ROS_WARN_STREAM("An empty cluster #" << cluster_idx << ".");
            continue;
        }
        for (size_t idx = 0u; idx < clusters_array[cluster_idx]->points.size(); ++idx) {
            pcl::PointXYZRGB point;
            point.x = clusters_array[cluster_idx]->points[idx].x;
            point.y = clusters_array[cluster_idx]->points[idx].y;
            point.z = clusters_array[cluster_idx]->points[idx].z;

            point.r = clusters_array[cluster_idx]->points[idx].r;
            point.g = clusters_array[cluster_idx]->points[idx].g;
            point.b = clusters_array[cluster_idx]->points[idx].b;

            cloud->points.push_back(point);
        }
    }

    if (cloud->size()) {
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(*cloud, msg_cloud);
        msg_cloud.header = header;
        publisher.publish(msg_cloud);
    }
}

}  // namespace utils
