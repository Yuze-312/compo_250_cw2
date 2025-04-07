/******************************************************************************
 * This file is part of the cw2_team_<your_team_number> package.              *
 *                                                                             *
 * You can modify or delete this file as you see fit. The only requirement is  *
 * that your entire solution must remain within the cw2_team_<your_team_number> *
 * package.                                                                    *
 ******************************************************************************/

#ifndef cw2_CLASS_H_
#define cw2_CLASS_H_

/* ----------------------------------------------------------------------------
 * System / Standard Library Includes
 * --------------------------------------------------------------------------*/
#include <vector>
#include <string>
#include <memory>

/* ----------------------------------------------------------------------------
 * ROS Includes
 * --------------------------------------------------------------------------*/
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

/* ----------------------------------------------------------------------------
 * MoveIt Includes
 * --------------------------------------------------------------------------*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Mesh.h>

/* ----------------------------------------------------------------------------
 * TF2 Includes
 * --------------------------------------------------------------------------*/
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

/* ----------------------------------------------------------------------------
 * PCL Includes
 * --------------------------------------------------------------------------*/
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/PolygonMesh.h>

/* ----------------------------------------------------------------------------
 * OctoMap Includes
 * --------------------------------------------------------------------------*/
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

/* ----------------------------------------------------------------------------
 * Eigen Includes
 * --------------------------------------------------------------------------*/
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

/* ----------------------------------------------------------------------------
 * Services from the cw2_world_spawner Package
 * --------------------------------------------------------------------------*/
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

// ----------------------------------------------------------------------------
// Additional commented-out includes (if needed in the future)
// #include "cw2_team_x/example.h"
// ----------------------------------------------------------------------------

/**
 * \struct DetectedObject
 * \brief Represents a detected object along with its attributes.
 */
struct DetectedObject {
  bool isValid;                 ///< True if the object was successfully detected
  bool isBasket;                ///< True if detected as a basket, false if cube
  bool isCube;                  ///< True if detected as a cube, false if basket
  geometry_msgs::Point position;///< Estimated position of the object
  std::string color;            ///< Color classification (e.g., "red", "blue", "purple", "none")
  double orientation_angle;     ///< Orientation angle in radians (for cubes)
};

/**
 * \struct OBBResult
 * \brief Holds min/max boundaries and orientation for an oriented bounding box.
 */
struct OBBResult {
  float min_x, max_x;
  float min_y, max_y;
  float min_z, max_z;
  float roll, pitch, yaw;       ///< Euler angles describing orientation
  pcl::PointXYZ center;         ///< Center position of the OBB
};

// ----------------------------------------------------------------------------
// Constants and Typedefs
// ----------------------------------------------------------------------------

/**
 * \brief Define gripper width constants (adjust if necessary).
 */
static constexpr double GRIPPER_OPEN_WIDTH = 0.15; ///< Fully open gripper width in meters
static constexpr double GRIPPER_CLOSED_WIDTH = 0.0;///< Fully closed gripper width in meters

/**
 * \brief Approximation of pi, used if <cmath> is not employed for M_PI.
 */
double pi_ = 3.1415926;

/**
 * \typedef PointT
 * \brief PCL point type with RGBA fields.
 */
typedef pcl::PointXYZRGBA PointT;

/**
 * \typedef PointC
 * \brief PCL point cloud of PointT.
 */
typedef pcl::PointCloud<PointT> PointC;

/**
 * \typedef PointCPtr
 * \brief Shared pointer to a PCL point cloud of PointT.
 */
typedef PointC::Ptr PointCPtr;

/**
 * \class cw2
 * \brief Main class providing functionality for tasks related to 
 *        robot perception, manipulation, and environment interaction.
 */
class cw2
{
public:
  /**
   * \brief Constructor
   * \param nh NodeHandle for ROS topics, services, etc.
   */
  cw2(ros::NodeHandle nh);

  /**
   * \brief Service callback for Task 1.
   */
  bool 
  t1_callback(cw2_world_spawner::Task1Service::Request &request,
              cw2_world_spawner::Task1Service::Response &response);

  /**
   * \brief Service callback for Task 2.
   */
  bool 
  t2_callback(cw2_world_spawner::Task2Service::Request &request,
              cw2_world_spawner::Task2Service::Response &response);

  /**
   * \brief Service callback for Task 3.
   */
  bool 
  t3_callback(cw2_world_spawner::Task3Service::Request &request,
              cw2_world_spawner::Task3Service::Response &response);

  /**
   * \brief Moves the robot arm to a target pose.
   * \param target_pose Desired end-effector pose.
   * \return True if the motion succeeded, false otherwise.
   */
  bool moveArm(geometry_msgs::Pose target_pose);

  /**
   * \brief Opens or closes the gripper.
   * \param width Desired gripper opening width in meters.
   * \return True if the motion succeeded, false otherwise.
   */
  bool moveGripper(float width);

  /**
   * \brief Converts a geometry_msgs::Point into a Pose with optional rotation.
   * \param point The reference point in space.
   * \param rotation The yaw rotation in radians.
   * \return A pose with orientation set according to the specified yaw.
   */
  geometry_msgs::Pose point2Pose(const geometry_msgs::Point &point, double rotation = 0.0);

  /**
   * \brief Creates a viewing pose for the camera above a specified point.
   * \param point The point over which to position the camera.
   * \return A Pose suitable for looking down at \p point.
   */
  geometry_msgs::Pose view_pose(const geometry_msgs::Point &point);

  /**
   * \brief Computes an object's orientation using PCL analysis.
   * \param approx_center Approximate center of the target object.
   * \param shape_type Type of shape to compute orientation for.
   * \return Orientation angle in radians.
   */
  double computeOrientationWithPCL(const geometry_msgs::Point &approx_center, const std::string &shape_type);

  /**
   * \brief Filters a region of the point cloud around a given center point.
   * \param center Center in world coordinates.
   * \return Filtered portion of the point cloud.
   */
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filterShapeRegion(const geometry_msgs::Point &center);

  /**
   * \brief Picks an object from a specified location with optional gripper opening.
   * \param obj_pt The 3D position of the object to pick.
   * \param rotation Yaw rotation for the end-effector.
   * \param gripper_open_ Gripper opening width prior to picking.
   * \return True if the pick operation succeeded, false otherwise.
   */
  bool pickObject(const geometry_msgs::Point &obj_pt, double rotation, double gripper_open_ = 0.1f);

  /**
   * \brief Places an object at a specified basket location.
   * \param basket_pt The 3D position of the basket or placement area.
   * \return True if the place operation succeeded, false otherwise.
   */
  bool placeObject(const geometry_msgs::Point &basket_pt);

  /**
   * \brief Returns a set of scanning poses used to survey the environment.
   * \return Vector of poses to move the robot's camera or sensor.
   */
  std::vector<geometry_msgs::Pose> getScanPoses();

  /**
   * \brief ROS callback to handle incoming point cloud messages.
   * \param cloud_msg Pointer to the received sensor_msgs::PointCloud2.
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

  /**
   * \brief Looks up a transform between two frames using TF2.
   * \param from_frame Source frame name.
   * \param to_frame Target frame name.
   * \return An Eigen::Affine3f describing the transform.
   */
  Eigen::Affine3f lookupTransform(const std::string &from_frame, const std::string &to_frame);

  /**
   * \brief Publishes a filtered point cloud message via a given publisher.
   * \param pc_pub The ROS publisher for the point cloud.
   * \param cloud The PCL point cloud to publish.
   */
  void pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &cloud);

  /**
   * \brief Captures point cloud data and classifies an object's shape by name.
   * \param pt Rough center point of the object to classify.
   * \return String describing the shape (e.g. "cube", "basket").
   */
  std::string captureAndClassifyShape(const geometry_msgs::Point &pt);

  /**
   * \brief Identifies whether a shape has a center hole or not, used for 
   *        distinguishing "nought" from "cross" in certain tasks.
   * \param cloud The PCL point cloud representing the shape.
   * \return String classification result.
   */
  std::string classifyNoughtOrCrossCenterHole(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

  /**
   * \brief Removes a plane from a point cloud using RANSAC.
   * \tparam PointT The point type used (e.g., pcl::PointXYZ).
   * \param in Input cloud.
   * \param out Plane-removed cloud.
   * \param dist_threshold Distance threshold for plane segmentation.
   */
  template <typename PointT>
  void removePlane(const typename pcl::PointCloud<PointT>::Ptr &in,
                   typename pcl::PointCloud<PointT>::Ptr &out,
                   double dist_threshold);

  /**
   * \brief Snaps a value to the closest in a set of possible values.
   * \param x_meters The value to snap.
   * \param possible_values Vector of possible discrete values.
   * \return The closest value from \p possible_values.
   */
  double snapX(double x_meters, const std::vector<double> &possible_values);

  /**
   * \brief Transforms a point cloud from sensor frame to target frame using TF2.
   * \tparam PointT PCL point type.
   * \param raw_cloud The input point cloud.
   * \param sensor_frame Frame ID of the point cloud.
   * \param target_frame Desired output frame.
   * \return True if transform succeeded, false otherwise.
   */
  template <typename PointT>
  bool transformCloudUsingTF2(typename pcl::PointCloud<PointT>::Ptr &raw_cloud,
                              const std::string &sensor_frame,
                              const std::string &target_frame);

  /**
   * \brief Publishes a marker representing an orientation vector for debugging.
   * \param position Start of the arrow.
   * \param direction Direction of the arrow.
   * \param frame_id Frame in which the marker is published.
   * \param marker_id Unique ID for the marker.
   * \param length Arrow length.
   * \param r,g,b Color components for the arrow.
   */
  void publishOrientationVector(const Eigen::Vector3f &position,
                                const Eigen::Vector3f &direction,
                                const std::string &frame_id,
                                int marker_id,
                                float length,
                                float r, float g, float b);

  /**
   * \brief Transforms a local Pose into a global Pose using known orientation 
   *        and position references.
   * \param local_pose Local coordinate pose.
   * \param global_center Global position offset.
   * \param global_q Global orientation offset (Quaternion).
   * \return Transformed global pose.
   */
  geometry_msgs::Pose transformLocalPose(const geometry_msgs::Pose &local_pose,
                                         const geometry_msgs::Point &global_center,
                                         const geometry_msgs::Quaternion &global_q);

  /**
   * \brief Prepares a plan to pick an object (no actual execution).
   * \param obj_pt The 3D position of the object.
   * \param yaw Desired orientation (yaw) for the end-effector.
   * \param gripper_opening Desired gripper opening width.
   * \return True if planning succeeded, false otherwise.
   */
  bool pickObjectPlanOnly(const geometry_msgs::Point &obj_pt,
                          double yaw,
                          double gripper_opening);

  /**
   * \brief Moves the camera above a given object at a specified height.
   * \param obj_pt The 3D position of the object.
   * \param approach_height Height above the object to move the camera to.
   * \return True if motion succeeded, false otherwise.
   */
  bool moveCameraAbove(const geometry_msgs::Point &obj_pt, double approach_height);

  /**
   * \brief Retrieves a point cloud of a particular object shape from the scene.
   * \param shape_type The shape type to filter for (e.g., "cube" or "basket").
   * \return Filtered point cloud of the requested shape.
   */
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getObjectCloud(const std::string &shape_type);

  /**
   * \brief Transforms a raw cloud, then removes a plane.
   * \param raw_cloud Input cloud.
   * \param sensor_frame Cloud source frame.
   * \param target_frame Desired output frame.
   * \param plane_dist_threshold RANSAC threshold for plane removal.
   * \return Transformed, plane-removed cloud.
   */
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformAndRemovePlane(
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr raw_cloud,
      const std::string &sensor_frame,
      const std::string &target_frame,
      double plane_dist_threshold);

  /**
   * \brief Adds a "cross" shaped collision object to the MoveIt planning scene.
   * \param pos_x, pos_y, pos_z Position in world coordinates.
   * \param roll, pitch, yaw Orientation in world frame.
   * \param arm_length, arm_width Dimensions of the cross arms.
   * \param thickness Cross thickness (height).
   * \param object_id Unique ID for the collision object.
   * \param frame_id Reference frame ID for the object.
   * \param psi Reference to the PlanningSceneInterface.
   * \return True if adding the object succeeded.
   */
  bool addCrossWithOrientation(double pos_x, double pos_y, double pos_z,
                               double roll, double pitch, double yaw,
                               double arm_length, double arm_width,
                               double thickness,
                               const std::string &object_id,
                               const std::string &frame_id,
                               moveit::planning_interface::PlanningSceneInterface &psi);

  /**
   * \brief Computes the orientation of a shape and performs a pick.
   * \param shape_cloud The shape's point cloud.
   * \param object_pt Out parameter for the object centroid.
   * \param shape_type Classification of shape (e.g., "nought", "cross").
   * \return True if pick succeeded, false otherwise.
   */
  bool computeOrientationAndPick(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr shape_cloud,
                                 geometry_msgs::Point &object_pt,
                                 const std::string &shape_type);

  /**
   * \brief Adds a rectangular box collision object for an "arm" of a shape.
   * \param cross_centroid Center position in the world.
   * \param arm_vec Direction vector for the arm.
   * \param length Local X dimension of the arm.
   * \param width Local Y dimension of the arm.
   * \param height Local Z dimension of the arm.
   * \param object_id Unique identifier for the collision object.
   * \param frame_id Reference frame ID.
   * \param psi PlanningSceneInterface to add the collision object.
   * \return True if addition was successful.
   */
  bool addArmBoxAtCentroid(const geometry_msgs::Point &cross_centroid,
                           const Eigen::Vector3f &arm_vec,
                           float length,
                           float width,
                           float height,
                           const std::string &object_id,
                           const std::string &frame_id,
                           moveit::planning_interface::PlanningSceneInterface &psi);

  /**
   * \brief Estimates orientation from shape corners in a given point cloud.
   * \param cloud_rgba The shape's point cloud.
   * \return Orientation angle in radians.
   */
  double estimateOrientationFromCorners(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_rgba);

  /**
   * \brief Performs clustering on a scene point cloud to separate objects.
   * \param cloud_in Input point cloud to cluster.
   * \return Vector of pointers, each to a separate clustered object's point cloud.
   */
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterSceneObjects(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);

  /**
   * \brief Callback that processes the incoming OctoMap point cloud.
   * \param cloud_input_msg The received sensor_msgs::PointCloud2.
   */
  void octomapCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  /**
   * \brief Callback for color image messages. Used for color-based classification.
   * \param msg The received sensor_msgs::Image.
   */
  void colorImageCallback(const sensor_msgs::Image &msg);

  /**
   * \brief Publishes a debug arrow marker for an object's orientation.
   * \param center_world The arrow's start position in the world.
   * \param direction_world The arrow's direction in the world.
   * \param frame_id The reference frame in which the marker is published.
   * \param marker_id A unique ID for the marker.
   * \param length Arrow length.
   * \param r,g,b Color components for the arrow.
   */
  void publishVectorArrow(const pcl::PointXYZRGBA &center_world,
                          const Eigen::Vector3f &direction_world,
                          const std::string &frame_id,
                          int marker_id,
                          float length,
                          float r, float g, float b);

  /**
   * \brief Publishes a marker that visualizes a rotation angle.
   * \param position Location of the marker in the world.
   * \param angle_rad The rotation angle in radians.
   * \param frame_id Reference frame for the marker.
   * \param marker_id Unique ID for the marker.
   */
  void publishAngleMarker(const Eigen::Vector3f &position,
                          double angle_rad,
                          const std::string &frame_id,
                          int marker_id);

  /**
   * \brief Scans the environment specifically for Task 3 requirements.
   */
  void scanEnvironmentForTask3();

  /**
   * \brief Classifies a cluster to distinguish among shapes or other objects.
   * \param object_point The centroid of the cluster.
   * \param dz A height offset or threshold used in classification logic.
   * \return String label of the shape/cluster type.
   */
  std::string classifyCluster(const geometry_msgs::Point &object_point, double dz);

  /**
   * \brief Retrieves the centroid of a PCL cluster.
   * \param cluster The PCL cluster.
   * \return geometry_msgs::Point representing the cluster's centroid.
   */
  geometry_msgs::Point getClusterCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster);

  /**
   * \brief Publishes the current debug cloud, labeling it with a string.
   * \param info Informational label to attach to the debug cloud.
   */
  void publishCurrentDebugCloud(const std::string &info);

  /**
   * \brief Publishes a specific debug cloud with transformations applied.
   * \param cloud_in The input cloud to be published.
   * \param sensor_frame Frame in which the cloud was captured.
   * \param target_frame Desired frame for publication.
   */
  void publishDebugCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                         const std::string &sensor_frame,
                         const std::string &target_frame);

  /**
   * \brief Classifies if a cluster is a basket or not.
   * \param cloud The PCL cloud of the object to check.
   * \return "basket" or "other" string label.
   */
  std::string classifyBasketOrNot(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

  /**
   * \brief Picks a shape identified in Task 3, given its centroid and label.
   * \param shape_centroid The 3D position of the object.
   * \param shape_label Identified label of the shape.
   * \return True if pick succeeded, false otherwise.
   */
  bool pickShapeFromTask3(const geometry_msgs::Point &shape_centroid,
                          const std::string &shape_label);

  /**
   * \brief Performs shape-specific processing for a discovered object.
   * \param object_point The centroid of the object.
   * \return String describing the processed shape.
   */
  std::string processShape(const geometry_msgs::Point &object_point);

  /**
   * \brief Checks which known shape region (if any) contains a point.
   * \param ref_points A set of reference points bounding each known region.
   * \param mystery_point The point under test.
   * \return Index of the matching region, or -1 if none.
   */
  int64_t shapeChecker(std::vector<geometry_msgs::PointStamped> ref_points,
                       geometry_msgs::PointStamped mystery_point);

  /**
   * \brief Applies a filter pipeline for Task 3.
   * \param in_cloud_ptr Input cloud pointer.
   * \param out_cloud_ptr Filtered output cloud pointer.
   */
  void applyFilterTask3(PointCPtr &in_cloud_ptr,
                        PointCPtr &out_cloud_ptr);
  
  bool planPickObjectRemoveBox(
      const geometry_msgs::Point &obj_pt,
      double yaw,
      double gripper_opening,
      const std::string &bounding_box_id);

  double crossAngleDetection(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr shape_cloud,
    geometry_msgs::Point &object_pt_world);

  Eigen::Vector3f rotateVectorAroundZ(const Eigen::Vector3f &v, double angle);
  /**
   * \brief Public handle to the ROS node.
   */
  ros::NodeHandle nh_;

  /**
   * \brief Service servers for each task.
   */
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

private:
  float gripper_open_;    ///< Stores the open gripper width.
  float gripper_closed_;  ///< Stores the closed gripper width.

  /**
   * \brief MoveIt interfaces for controlling the arm and gripper.
   */
  moveit::planning_interface::MoveGroupInterface move_group_arm_;
  moveit::planning_interface::MoveGroupInterface move_group_gripper_;

  /**
   * \brief ROS publishers and subscribers for debugging and data streams.
   */
  ros::Publisher plane_removed_pub_;
  sensor_msgs::ImageConstPtr last_color_image_;

  /**
   * \brief TF2 components for transform lookups.
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  /**
   * \brief Global point cloud pointers for processing and filtering.
   */
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr g_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr g_cloud_filtered;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr g_cloud_filtered2, g_cloud_filtered_octomap;
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_octomap_filtered;
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_octomap_ptr;
  pcl::PCLPointCloud2 g_octomap_pc;
  pcl::PassThrough<pcl::PointXYZ> g_octomap_pt;
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;

  ros::Publisher debug_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher cluster_pub_, debug_pub_cloud_in, g_pub_cloud;
  ros::Publisher debug_t2_pub_;

  ros::Subscriber cloud_sub_, octomap_sub_, image_sub_, pc_sub_;  
  ros::Subscriber octomap_pointcloud_sub_;
  ros::Publisher octomap_debug_pub_, g_pub_cloud_octomap; 

  /**
   * \brief Filters for "nought" and "cross" shape extraction.
   */
  pcl::CropBox<pcl::PointXYZRGBA> nought_filter_;
  pcl::PassThrough<pcl::PointXYZ> cross_filter_;

  /**
   * \brief Additional declarations for analyzing octomap data.
   */
  std::string g_input_pc_frame_id_;
  ros::Publisher octomap_centers_pub_;
  std::shared_ptr<octomap::OcTree> octree_;
  void publishOctomapCenters();

  double octomap_resolution_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;

  /**
   * \brief Storage and parameters for color image data.
   */
  std::vector<unsigned char> color_image_data;
  int color_image_width_;
  int color_image_height_;
  int color_image_midpoint_;
  int color_channels_ = 3;

  /**
   * \brief Example usage of filtering parameters.
   */
  double width = 0.04;
  pcl::CropBox<pcl::PointXYZRGBA> nought_filter;     // Duplicate instance
  pcl::PassThrough<PointT> g_pt;                     // Another pass-through filter
  pcl::PCLPointCloud2 g_pcl_pc;
  bool task_1_filter = false;
  bool task_3_filter = false;
  moveit::planning_interface::PlanningSceneInterface psi_;
};

#endif // cw2_CLASS_H_
