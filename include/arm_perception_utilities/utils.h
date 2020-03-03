/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-28 09:59:53
 * @modify date 2019-11-28 09:59:53
 * @desc [description]
 */
#ifndef util_H
#define util_H
#include <geometry_msgs/Quaternion.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsoncpp/json/json.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include </usr/include/eigen3/Eigen/Geometry>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

// for Gaussian noise to pcl
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/date_time/gregorian/gregorian_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/thread/thread.hpp>
#include "ApproxMVBB/ComputeApproxMVBB.hpp"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
/**
 * @brief
 * A namespace to keep utility functions
 */

namespace utils {

/**
 * @brief rounds inputted double after 2 decimals
 *
 * @param var
 * @return double
 */
double round(double var);

/**
 * @brief put roll, pitch yaw angles and get goemetry_msgs QUaternions
 *
 * @param robot_rx_deg
 * @param robot_ry_deg
 * @param robot_rz_deg
 * @param kDEG2RAD
 * @return geometry_msgs::Quaternion
 */
geometry_msgs::Quaternion EulertoQuaternion(double robot_rx_deg, double robot_ry_deg, double robot_rz_deg,
                                            double kDEG2RAD);

/**
 * @brief given 8 korners represented in image plane, draws lines between this corners , the outcome is a 3D box
 * overlayed onto image plane
 *
 * @param korners
 * @param image
 */
void construct3DBoxOnImage(std::vector<std::vector<double>> korners, cv::Mat *image);

/**
 * @brief Set the Marker Data object, we can use this function to set commonly used marker fields
 *
 * @param marker
 * @param px
 * @param py
 * @param pz
 * @param ox
 * @param oy
 * @param oz
 * @param ow
 * @param sx
 * @param sy
 * @param sz
 * @param r
 * @param g
 * @param b
 * @param a
 */
void SetMarkerData(visualization_msgs::Marker *marker, double px, double py, double pz, double ox, double oy, double oz,
                   double ow, double sx, double sy, double sz, double r, double g, double b, double a);

/**
 * @brief transform given 3D point onto 2D image Plane, with provided transform and projection matrices(or matrices
 * created as a results of their multiplication)
 *
 * @param rect_cam_points
 * @param TRANS_RECTCAM_TO_IMAGE
 * @return Eigen::MatrixXf
 */
Eigen::MatrixXf transform3DPointsToImage(const Eigen::MatrixXf &rect_cam_points,
                                         Eigen::MatrixXf TRANS_RECTCAM_TO_IMAGE);

/**
 * @brief  converts jsk_recognition_msgs::BoundingBoxArray to visualization_msgs::MarkerAraay
 *
 * @param jsk_lbl
 * @return visualization_msgs::MarkerArray
 */
visualization_msgs::MarkerArray jsk2Marker(jsk_recognition_msgs::BoundingBoxArray jsk_lbl);

/**
 * @brief randomly generates a double between fmin and fmax
 *
 * @param fMin
 * @param fMax
 * @return double
 */
double fRand(double fMin, double fMax);

/**
 * @brief Vector3List is a type coming from Object Builder Library(ApproxMVBB), the corners of built objects are in this
 * Vector3list type, howveer wee neeed this corners to be in a convient contairer for ROS, that container is a vector of
 * geometry_msgs::Point
 *
 * @param corners
 * @return std::vector<geometry_msgs::Point>
 */
std::vector<geometry_msgs::Point> Vector3ListGeometryMsgs(ApproxMVBB::TypeDefsPoints::Vector3List corners);

/**
 * @brief finds the nearest corner with respect to the given corner(current_point_indice)
 *
 * @param current_point_indice
 * @param corners_geometry_msgs
 * @return geometry_msgs::Point
 */
geometry_msgs::Point findNearestCorner(int current_point_indice,
                                       std::vector<geometry_msgs::Point> corners_geometry_msgs);

/**
 * @brief publish clustering objects' in one point cloud
 * @param publisher
 * @param header
 * @param cloud_clusters
 * @param trans
 */
void publishClustersCloud(const ros::Publisher &publisher, const std_msgs::Header &header,
                          const std::vector<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters_array);

};  // namespace utils
#endif
