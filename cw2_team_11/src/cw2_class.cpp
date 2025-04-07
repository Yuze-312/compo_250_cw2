/******************************************************************************
 * This file is part of the cw2_team_11 package.              *
 *                                                                             *
 * You can modify or delete this file as you see fit. The only requirement is  *
 * that your entire solution must remain within the cw2_team_11 *
 * package.                                                                    *
 ******************************************************************************/

#include <cw2_class.h> // (Adjust to your actual team name/package)

/* ----------------------------------------------------------------------------
 * CONSTRUCTOR
 * ----------------------------------------------------------------------------
 * Initializes MoveIt interfaces, subscribers, publishers, and variables for
 * point cloud processing, color image capture, etc.
 */
cw2::cw2(ros::NodeHandle nh)
  : nh_(nh),
    move_group_arm_("panda_arm"),
    move_group_gripper_("hand")
{
  gripper_open_   = 100e-3f;
  gripper_closed_ = 0.0f;

  tf_buffer_ptr_    = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));
  tf_listener_ptr_  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);

  // Advertise services
  t1_service_ = nh_.advertiseService("/task1_start", &cw2::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start", &cw2::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start", &cw2::t3_callback, this);

  // Subscribe to point cloud, image, and octomap topics
  pc_sub_ = nh_.subscribe(
      "/r200/camera/depth_registered/points",
      1,
      &cw2::pointCloudCallback,
      this);

  image_sub_ = nh_.subscribe(
      "/r200/camera/color/image_raw",
      1,
      &cw2::colorImageCallback,
      this);

  octomap_sub_ = nh_.subscribe(
      "/octomap_point_cloud_centers",
      1,
      &cw2::octomapCallback,
      this);

  // Initialize point clouds
  g_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  g_cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  g_cloud_filtered2.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

  g_octomap_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
  g_octomap_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
  g_cloud_filtered_octomap.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

  // Publishers for various debug and processed clouds
  g_pub_cloud_octomap = nh.advertise<sensor_msgs::PointCloud2>("octomap_cloud", 1, true);
  g_pub_cloud         = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1, true);
  octomap_debug_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("debug_octomap_task3", 1, true);
  cluster_pub_        = nh_.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1, true);
  debug_pub_          = nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud", 1, true);
  debug_pub_cloud_in  = nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud_in", 1, true);
  debug_t2_pub_       = nh_.advertise<sensor_msgs::PointCloud2>("debug_t2_cloud", 1, true);

  // Marker publisher for visualization
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("direction_marker", 1, true);

  ROS_INFO("cw2 class initialised");
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Move the robot arm to a specified pose using MoveIt.
 * @param target_pose The target end-effector pose.
 * @return True if planning and execution succeeded, false otherwise.
 */
bool cw2::moveArm(geometry_msgs::Pose target_pose)
{
  ROS_INFO("Setting pose target");
  move_group_arm_.setPoseTarget(target_pose);

  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_arm_.plan(my_plan)
                  == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // Execute the planned path if successful
  if (success)
  {
    move_group_arm_.move();
  }

  return success;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Move the panda gripper to the specified width (open or close).
 * @param width The desired gripper opening in [0, gripper_open_].
 * @return True if plan and execution succeeded, false otherwise.
 */
bool cw2::moveGripper(float width)
{
  // Safety clamp
  if (width > gripper_open_)   width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // Each finger = half the requested width (or direct if single-joint semantics)
  double eachJoint = width;
  std::vector<double> gripperJointTargets(2, eachJoint);

  move_group_gripper_.setJointValueTarget(gripperJointTargets);

  ROS_INFO("Attempting to plan the path (gripper)");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_gripper_.plan(my_plan)
                  == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  if (success)
  {
    move_group_gripper_.move();
    // Brief pause to ensure motion completes
    ros::Duration(1).sleep();
  }

  return success;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Convert a geometry_msgs::Point + rotation_z into a Pose with
 *        the gripper pointing downward.
 * @param point The target position.
 * @param rotation The additional yaw rotation in radians.
 * @return A Pose with downward orientation and the specified rotation about Z.
 */
geometry_msgs::Pose cw2::point2Pose(const geometry_msgs::Point &point, double rotation)
{
  double angle_offset_ = pi_ / 4.0;

  // Orientation: flip gripper 180° around X, then rotate around Z
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_ + rotation);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion orientation = tf2::toMsg(q_result);

  geometry_msgs::Pose pose;
  pose.position    = point;
  pose.orientation = orientation;

  return pose;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Color image callback. Updates internal color image data for shape classification.
 * @param msg The received color image.
 */
void cw2::colorImageCallback(const sensor_msgs::Image &msg)
{
  static bool setup = [&](){
    // Camera feed resolution
    cw2::color_image_width_  = msg.width;
    cw2::color_image_height_ = msg.height;

    // Compute the index of the middle pixel in the flattened RGB array
    cw2::color_image_midpoint_ = cw2::color_channels_
                                 * ((cw2::color_image_width_ * (cw2::color_image_height_ / 2))
                                 + (cw2::color_image_width_ / 2))
                                 - cw2::color_channels_;
    return true;
  }();

  // Copy the raw data for usage in classification
  this->color_image_data = msg.data;
  return;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Callback for incoming point clouds. Stores them for filtering.
 * @param cloud_msg The input sensor_msgs::PointCloud2.
 */
void cw2::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // 1) Store the frame ID if needed
  g_input_pc_frame_id_ = cloud_msg->header.frame_id;

  // 2) Convert from ROS to PCL format
  pcl_conversions::toPCL(*cloud_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

  // If Task 1 filter is active, copy data into g_cloud_filtered(2)
  if (task_1_filter)
  {
    copyPointCloud(*g_cloud_ptr, *g_cloud_filtered);
    copyPointCloud(*g_cloud_ptr, *g_cloud_filtered2);

    // Optionally publish g_cloud_filtered2
    pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered2);

    // Convert to a simpler point type if needed for Octomap or further tasks
    pcl::PointCloud<pcl::PointXYZ> cloud_oct;
    pcl::fromROSMsg(*cloud_msg, cloud_oct);
  }
  // If Task 3 filter is active, apply dedicated filter
  else if (task_3_filter)
  {
    applyFilterTask3(g_cloud_ptr, g_cloud_filtered_octomap);
    pubFilteredPCMsg(g_pub_cloud_octomap, *g_cloud_filtered_octomap);
  }

  return;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Apply a specialized filter pipeline for Task 3.
 * @param in_cloud_ptr  The input cloud pointer.
 * @param out_cloud_ptr The filtered output cloud pointer.
 */
void cw2::applyFilterTask3(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  // Setup pass-through filter for certain ranges
  g_pt.setInputCloud(in_cloud_ptr);

  // Filter X range to ±1.0
  g_pt.setFilterFieldName("x");
  g_pt.setFilterLimits(-1.0, 1.0);

  // Filter Z range up to 0.5 m
  g_pt.setFilterFieldName("z");
  g_pt.setFilterLimits(0, 0.5);

  g_pt.filter(*out_cloud_ptr);
  return;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Callback for Octomap point cloud centers. Filters and publishes them.
 * @param cloud_input_msg Input Octomap centers as sensor_msgs::PointCloud2.
 */
void cw2::octomapCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // 1) Convert to PCL
  pcl_conversions::toPCL(*cloud_input_msg, g_octomap_pc);
  pcl::fromPCLPointCloud2(g_octomap_pc, *g_octomap_ptr);

  // 2) Filter with pass-through
  g_octomap_pt.setInputCloud(g_octomap_ptr);
  g_octomap_pt.setFilterFieldName("z");
  g_octomap_pt.setFilterLimits(0.04, 0.5);
  g_octomap_pt.filter(*g_octomap_filtered);

  // 3) Publish filtered octomap centers
  sensor_msgs::PointCloud2 octo_msg;
  pcl::toROSMsg(*g_octomap_filtered, octo_msg);
  octo_msg.header = cloud_input_msg->header;
  g_pub_cloud_octomap.publish(octo_msg);
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Publishes a filtered PCL point cloud as a ROS PointCloud2.
 * @param pc_pub The ROS publisher.
 * @param cloud The PCL point cloud to publish.
 */
void cw2::pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &cloud)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "world"; // or whichever frame you prefer
  pc_pub.publish(msg);
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Transforms a point cloud from sensor_frame to target_frame using TF2.
 * @tparam PointT The point type (e.g. pcl::PointXYZ).
 * @param raw_cloud The input (and output) PCL cloud pointer.
 * @param sensor_frame Frame ID of the cloud.
 * @param target_frame Desired frame to transform into.
 * @return True if transform succeeded, false otherwise.
 */
template <typename PointT>
bool cw2::transformCloudUsingTF2(
    typename pcl::PointCloud<PointT>::Ptr &raw_cloud,
    const std::string &sensor_frame,
    const std::string &target_frame)
{
  // 1) Convert PCL to ROS
  sensor_msgs::PointCloud2 cloud_in, cloud_out;
  pcl::toROSMsg(*raw_cloud, cloud_in);
  cloud_in.header.frame_id = sensor_frame;

  // 2) Lookup transform
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_buffer_ptr_->lookupTransform(
        target_frame, sensor_frame, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("transformCloudUsingTF2: TF lookup failed: %s", ex.what());
    return false;
  }

  // 3) Apply doTransform
  try
  {
    tf2::doTransform(cloud_in, cloud_out, transformStamped);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("transformCloudUsingTF2: doTransform failed: %s", ex.what());
    return false;
  }

  // 4) Convert back to PCL
  pcl::fromROSMsg(cloud_out, *raw_cloud);
  return true;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Publishes a visualization marker (arrow) representing a direction vector.
 * @param center_world The arrow's start position.
 * @param direction_world The arrow's direction in the world frame.
 * @param frame_id The reference frame for the marker.
 * @param marker_id Unique marker ID.
 * @param length Arrow length in meters.
 * @param r,g,b Color components (0-1 range).
 */
void cw2::publishVectorArrow(
    const pcl::PointXYZRGBA &center_world,
    const Eigen::Vector3f &direction_world,
    const std::string &frame_id,
    int marker_id,
    float length,
    float r, float g, float b)
{
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = frame_id;
  arrow.header.stamp    = ros::Time::now();
  arrow.ns              = "obb_diagonals";
  arrow.id              = marker_id;
  arrow.type            = visualization_msgs::Marker::ARROW;
  arrow.action          = visualization_msgs::Marker::ADD;

  // Start and end points of the arrow
  geometry_msgs::Point start, end;
  start.x = center_world.x;
  start.y = center_world.y;
  start.z = center_world.z;

  Eigen::Vector3f dirScaled = direction_world.normalized() * length;
  end.x = center_world.x + dirScaled.x();
  end.y = center_world.y + dirScaled.y();
  end.z = center_world.z + dirScaled.z();
  arrow.points.push_back(start);
  arrow.points.push_back(end);

  // Arrow dimensions
  arrow.scale.x = 0.005; // shaft diameter
  arrow.scale.y = 0.01;  // arrow head diameter
  arrow.scale.z = 0.0;   // unused here

  // Color
  arrow.color.r = r;
  arrow.color.g = g;
  arrow.color.b = b;
  arrow.color.a = 1.0;

  arrow.lifetime = ros::Duration(0.0);
  marker_pub_.publish(arrow);
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Moves the camera above a specified point at a given height.
 * @param obj_pt The target object point.
 * @param approach_height Height above the point to place the camera.
 * @return True if the motion succeeded, false otherwise.
 */
bool cw2::moveCameraAbove(const geometry_msgs::Point &obj_pt, double approach_height)
{
  task_1_filter = true; // Enable Task 1 filter

  geometry_msgs::Pose view_pose = point2Pose(obj_pt);
  view_pose.position.z = approach_height;
  view_pose.position.x -= 0.04;

  // Flip the camera to face downward
  tf2::Quaternion q_down;
  q_down.setRPY(M_PI, 0, 0);
  view_pose.orientation = tf2::toMsg(q_down);

  bool success = moveArm(view_pose);
  if (!success)
  {
    ROS_ERROR("moveCameraAbove: moveArm failed => abort");
    return false;
  }

  ros::Duration(1.0).sleep(); // Wait for updated sensor data
  return true;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Retrieves a point cloud of a particular shape type (e.g., nought/cross).
 * @param shape_type The shape type to retrieve ("cross" or something else).
 * @return A pointer to the relevant PCL cloud, or nullptr if empty.
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cw2::getObjectCloud(const std::string &shape_type)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

  if (shape_type == "cross")
  {
    if (!g_cloud_filtered2 || g_cloud_filtered2->empty())
    {
      ROS_ERROR("getObjectCloud: g_cloud_filtered2 empty => cannot do cross");
      return nullptr;
    }
    copyPointCloud(*g_cloud_filtered2, *raw_cloud);
  }
  else
  {
    if (!g_cloud_filtered || g_cloud_filtered->empty())
    {
      ROS_ERROR("getObjectCloud: g_cloud_filtered empty => cannot do nought");
      return nullptr;
    }
    copyPointCloud(*g_cloud_filtered, *raw_cloud);
  }
  return raw_cloud;
}

double cw2::crossAngleDetection(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr shape_cloud,
    geometry_msgs::Point &object_pt_world)
  {
    if (!shape_cloud || shape_cloud->empty())
    {
      ROS_WARN("crossAngleDetection: shape_cloud is empty => returning 0.0");
      return 0.0;
    }

    // 1) Convert shape_cloud => pcl::PointCloud<pcl::PointXYZ> if needed
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_xyz->reserve(shape_cloud->size());
    for (auto &pt_rgba : shape_cloud->points)
    {
      pcl::PointXYZ pt;
      pt.x = pt_rgba.x;
      pt.y = pt_rgba.y;
      pt.z = pt_rgba.z;
      cloud_xyz->push_back(pt);
    }

    // 2) Set up RANSAC for line detection
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01); // depends on cross thickness
    seg.setInputCloud(cloud_xyz);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
    {
      ROS_WARN("crossAngleDetection: RANSAC found no line => returning 0.0");
      return 0.0;
    }

    // 3) Extract line model => [px, py, pz, dirx, diry, dirz]
    float px  = coefficients->values[0];
    float py  = coefficients->values[1];
    float pz  = coefficients->values[2];
    float dirx= coefficients->values[3];
    float diry= coefficients->values[4];
    float dirz= coefficients->values[5];

    ROS_INFO("Line model => point(%.3f,%.3f,%.3f), dir(%.3f,%.3f,%.3f)",
            px, py, pz, dirx, diry, dirz);

    // 4) Compute angle in XY plane => e.g. orientation
    double raw_angle = std::atan2(diry, dirx);

    // 5) For debugging, let's publish an arrow from the cross centroid
    //    that uses the direction from RANSAC
    {
      // Build a PCL point for the 'center'
      pcl::PointXYZRGBA center_pcl;
      center_pcl.x = object_pt_world.x;
      center_pcl.y = object_pt_world.y;
      center_pcl.z = object_pt_world.z;
      center_pcl.r = 255; // if you want color
      center_pcl.g = 0;
      center_pcl.b = 0;
      center_pcl.a = 255;

      Eigen::Vector3f direction_vec(dirx, diry, dirz);
      // If you only care about XY direction, you can set z=0 or normalize in 2D
      // but let's keep 3D
      direction_vec.normalize();

      float arrow_length = 0.2f; // e.g. 20 cm arrow
      // Use your existing function to visualize
      publishVectorArrow(center_pcl,
                        direction_vec,
                        "world",      // or your frame
                        0,            // marker_id
                        arrow_length, 
                        1.0f, 0.0f, 0.0f); // red arrow
    }

    // 6) Return the angle
    return raw_angle;
  }

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Transform a raw cloud to a target frame and remove the dominant plane.
 * @param raw_cloud Input cloud pointer.
 * @param sensor_frame Source frame ID.
 * @param target_frame Desired target frame ID.
 * @param plane_dist_threshold RANSAC plane distance threshold.
 * @return A pointer to the plane-removed cloud, or nullptr if error/empty.
 */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cw2::transformAndRemovePlane(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr raw_cloud,
    const std::string &sensor_frame,
    const std::string &target_frame,
    double plane_dist_threshold)
{
  // Transform
  if (!transformCloudUsingTF2<pcl::PointXYZRGBA>(raw_cloud, sensor_frame, target_frame))
  {
    ROS_ERROR("transformAndRemovePlane: transform failed => returning null");
    return nullptr;
  }

  // Remove plane
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr shape_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  removePlane<pcl::PointXYZRGBA>(raw_cloud, shape_cloud, plane_dist_threshold);

  if (shape_cloud->empty())
  {
    ROS_ERROR("transformAndRemovePlane: shape_cloud empty => returning null");
    return nullptr;
  }
  return shape_cloud;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Compute orientation of a shape, then pick it using the found orientation.
 * @param shape_cloud The cloud of the shape.
 * @param object_pt The object centroid, possibly adjusted for pick approach.
 * @param shape_type "cross" or "nought" classification.
 * @return True if pick succeeded, false otherwise.
 */
bool cw2::computeOrientationAndPick(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr shape_cloud,
    geometry_msgs::Point &object_pt,
    const std::string &shape_type)
{
  // For example, if cross => use PCA vectors to find orientation
  pcl::MomentOfInertiaEstimation<pcl::PointXYZRGBA> feature_extractor;
  feature_extractor.setInputCloud(shape_cloud);
  feature_extractor.compute();

  Eigen::Vector3f major_vec, middle_vec, minor_vec;
  feature_extractor.getEigenVectors(major_vec, middle_vec, minor_vec);

  double angle = atan2(middle_vec.y(), middle_vec.x());
  angle = crossAngleDetection(shape_cloud, object_pt);
  if (angle > M_PI / 2)  angle -= M_PI;
  if (angle < -M_PI / 2) angle += M_PI;

  // // Visualize the orientation vector as a debug arrow
  // Eigen::Vector3f centroid_pt(object_pt.x, object_pt.y, object_pt.z);
  // publishOrientationVector(centroid_pt, middle_vec, "world", 0, 0.15, 1.0, 0.0, 0.0);

  // If cross => pick with a small offset and gripper width
  if (shape_type == "cross")
  {
    
    ROS_INFO_STREAM("Angle from local X= " << angle << " rad");
    object_pt.x += 1 * width * cos(angle);
    object_pt.y += 1 * width * sin(angle);

    double gripper_width = 40e-3f;

    ROS_INFO_STREAM("Testing cross pick angle= " << angle << " rad...");
    bool success = pickObject(object_pt, -angle, gripper_width);
    if (success)
    {
      ROS_INFO_STREAM("Cross pick => success at angle= " << angle);
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Pick plan failed or collided at angle= " << angle << ". Trying next...");
    }

    ROS_ERROR("computeOrientationAndPick => no feasible orientation => picking fails");
    return false;
  }
  // If nought => use corner-based estimate
  else if (shape_type == "nought")
  {
    angle = estimateOrientationFromCorners(shape_cloud);
    object_pt.x += 2 * width * sin(angle);
    object_pt.y += 2 * width * cos(angle);
    return pickObject(object_pt, angle);
  }

  ROS_WARN("computeOrientationAndPick: unknown shape => no pick");
  return false;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Estimate orientation using corners (convex hull) approach.
 * @param cloud_rgba A pointer to the cloud of the shape.
 * @return Angle in radians that approximates the shape's orientation.
 */
double cw2::estimateOrientationFromCorners(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_rgba)
{
  // Flatten Z for a 2D approach
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_xy->reserve(cloud_rgba->size());
  for (const auto &pt : cloud_rgba->points)
  {
    pcl::PointXYZ pt_flat;
    pt_flat.x = pt.x;
    pt_flat.y = pt.y;
    pt_flat.z = 0.0;
    cloud_xy->push_back(pt_flat);
  }

  // Build a 2D convex hull
  pcl::PointCloud<pcl::PointXYZ> hull;
  std::vector<pcl::Vertices> polygons;
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud(cloud_xy);
  chull.setDimension(2); // treat as 2D
  chull.reconstruct(hull, polygons);

  if (hull.size() < 3)
  {
    ROS_WARN("Corners approach: hull < 3 => no orientation => returning 0.0");
    return 0.0;
  }
  ROS_INFO_STREAM("Hull has " << hull.size() << " points.");

  // Find the two most distant hull points (the diagonal)
  double max_dist = 0.0;
  Eigen::Vector2f p1, p2;
  for (size_t i = 0; i < hull.size(); ++i)
  {
    for (size_t j = i + 1; j < hull.size(); ++j)
    {
      Eigen::Vector2f a(hull.points[i].x, hull.points[i].y);
      Eigen::Vector2f b(hull.points[j].x, hull.points[j].y);
      double d = (a - b).norm();
      if (d > max_dist)
      {
        max_dist = d;
        p1 = a;
        p2 = b;
      }
    }
  }

  Eigen::Vector2f diag = p2 - p1;
  ROS_INFO("Longest diagonal length = %.4f m", diag.norm());

  double raw_diag_angle = std::atan2(diag.y(), diag.x());
  ROS_INFO("Diagonal raw angle = %.2f deg", raw_diag_angle * 180.0 / M_PI);

  // Offset by -45 deg if you want side alignment
  double angle = raw_diag_angle - (M_PI / 4.0);

  // Normalize angle to [-π, π]
  angle = std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;

  ROS_INFO("Corner-based orientation angle => %.2f deg", angle * 180.0 / M_PI);
  return angle;
}


///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Service callback for Task 1 (pick-and-place a shape).
 */
bool cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
                      cw2_world_spawner::Task1Service::Response &response)
{
  // Extract request data
  std::string shape_type      = request.shape_type;
  geometry_msgs::Point object_pt = request.object_point.point;
  geometry_msgs::Point basket_pt = request.goal_point.point;

  // Move camera above the object
  if (!moveCameraAbove(object_pt, 0.60))
    return false;

  // Get raw cloud for shape
  auto raw_cloud = getObjectCloud(shape_type);
  if (!raw_cloud) return false; // error already reported

  // Transform + plane removal
  auto shape_cloud = transformAndRemovePlane(raw_cloud, "color", "world", 0.021);
  if (!shape_cloud) return false;

  // Debug publish
  sensor_msgs::PointCloud2 dbg_msg;
  pcl::toROSMsg(*shape_cloud, dbg_msg);
  dbg_msg.header.frame_id = "world";
  debug_pub_.publish(dbg_msg);

  // Compute orientation + pick
  bool success = computeOrientationAndPick(shape_cloud, object_pt, shape_type);
  if (!success)
  {
    ROS_ERROR("Failed to pick => abort");
    return false;
  }

  // Adjust basket point if cross vs. nought
  if (shape_type == "cross")
  {
    basket_pt.x += width;
  }
  else
  {
    basket_pt.y += 2 * width;
  }

  // Place object
  if (!placeObject(basket_pt))
  {
    ROS_ERROR("placeObject => fail => abort");
    return false;
  }

  ROS_INFO("Task1 => done");
  return true;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Publishes a second orientation arrow marker for debugging.
 */
void cw2::publishOrientationVector(
    const Eigen::Vector3f &position,
    const Eigen::Vector3f &direction,
    const std::string &frame_id,
    int marker_id,
    float length,
    float r, float g, float b)
{
  visualization_msgs::Marker arrow_marker;
  arrow_marker.header.frame_id = frame_id;
  arrow_marker.header.stamp    = ros::Time::now();
  arrow_marker.ns              = "orientation_vectors";
  arrow_marker.id              = marker_id;
  arrow_marker.type            = visualization_msgs::Marker::ARROW;
  arrow_marker.action          = visualization_msgs::Marker::ADD;

  // Start-end points
  geometry_msgs::Point start, end;
  start.x = position[0];
  start.y = position[1];
  start.z = position[2];

  Eigen::Vector3f scaled_direction = direction.normalized() * length;
  end.x = start.x + scaled_direction[0];
  end.y = start.y + scaled_direction[1];
  end.z = start.z + scaled_direction[2];

  arrow_marker.points.push_back(start);
  arrow_marker.points.push_back(end);

  arrow_marker.scale.x = 0.01;  // shaft diameter
  arrow_marker.scale.y = 0.02;  // head diameter
  arrow_marker.scale.z = 0.0;   // unused

  arrow_marker.color.r = r;
  arrow_marker.color.g = g;
  arrow_marker.color.b = b;
  arrow_marker.color.a = 1.0;

  arrow_marker.lifetime = ros::Duration(0);
  marker_pub_.publish(arrow_marker);
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Publishes a text marker for an angle, used for debugging orientation.
 */
void cw2::publishAngleMarker(
    const Eigen::Vector3f &position,
    double angle_rad,
    const std::string &frame_id,
    int marker_id)
{
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = frame_id;
  text_marker.header.stamp    = ros::Time::now();
  text_marker.ns              = "angle_markers";
  text_marker.id              = marker_id;
  text_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action          = visualization_msgs::Marker::ADD;

  text_marker.pose.position.x = position[0];
  text_marker.pose.position.y = position[1];
  text_marker.pose.position.z = position[2] + 0.05; // offset above object

  text_marker.scale.z = 0.03; // text height (~3cm)
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;

  // Convert angle to degrees for clarity
  std::ostringstream oss;
  oss << "Angle: " << (angle_rad * 180.0 / M_PI) << " deg";
  text_marker.text = oss.str();

  text_marker.lifetime = ros::Duration(0);
  marker_pub_.publish(text_marker);
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Pick an object at obj_pt with the end-effector rotated by rotation_z.
 * @param obj_pt  The object's position in 3D space.
 * @param rotation_z The yaw rotation in radians.
 * @param gripper_open_ The gripper opening width before picking.
 * @return True if the pick sequence was successful, false otherwise.
 */
bool cw2::pickObject(const geometry_msgs::Point &obj_pt, double rotation_z, double gripper_open_)
{
  // 1) Grasp pose (downward orientation at yaw)
  geometry_msgs::Pose grasp_pose = point2Pose(obj_pt, rotation_z);

  grasp_pose.position.z += 0.14; // Assume object top is ~14cm below

  // 2) Approach pose ~slightly above the grasp
  geometry_msgs::Pose approach_pose = grasp_pose;
  approach_pose.position.z += 0.13;

  bool success = true;

  // Sequence of moves
  success &= moveArm(approach_pose);
  success &= moveGripper(gripper_open_);
  success &= moveArm(grasp_pose);
  success &= moveGripper(gripper_closed_);
  success &= moveArm(approach_pose);

  // Optionally move further upward
  approach_pose.position.z += 0.3;
  success &= moveArm(approach_pose);
  ros::Duration(0.5).sleep();

  return success;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Place the currently held object at basket_pt, then open the gripper.
 * @param basket_pt The target place location.
 * @return True if place was successful, false otherwise.
 */
bool cw2::placeObject(const geometry_msgs::Point &basket_pt)
{
  move_group_arm_.setMaxVelocityScalingFactor(0.3);
  move_group_arm_.setMaxAccelerationScalingFactor(0.05);

  // 1) Release pose above the basket
  geometry_msgs::Pose release_pose = point2Pose(basket_pt);
  release_pose.position.z += 0.45; // 45 cm above

  bool success = true;
  success &= moveArm(release_pose);

  // 2) Descend
  release_pose.position.z -= 0.20;
  success &= moveArm(release_pose);

  // 3) Open gripper
  success &= moveGripper(gripper_open_);

  // 4) Move up again
  release_pose.position.z += 0.10;
  success &= moveArm(release_pose);

  // Disable filter if it was set
  task_1_filter = false;
  return success;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Remove the largest plane from a point cloud.
 * @tparam PointT The point type (e.g., pcl::PointXYZ).
 * @param input_cloud The input cloud.
 * @param output_no_plane_cloud The cloud without the plane.
 * @param distance_threshold RANSAC distance threshold for plane fitting.
 */
template <typename PointT>
void cw2::removePlane(
    const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
    typename pcl::PointCloud<PointT>::Ptr &output_no_plane_cloud,
    double distance_threshold)
{
  // 1) Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);

  // 2) Segment the largest planar component
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty())
  {
    ROS_WARN("removePlane: No plane found! Returning input cloud.");
    *output_no_plane_cloud = *input_cloud;
    return;
  }

  // 3) Extract indices (remove plane)
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*output_no_plane_cloud);

  ROS_INFO_STREAM("removePlane: Removed "
                  << inliers->indices.size() << " plane points, leftover "
                  << output_no_plane_cloud->size() << " points.");
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Distinguish between a nought or cross by checking for a center hole.
 * @param cloud The shape's point cloud.
 * @return "nought" if center is empty, "cross" if center is occupied.
 */
std::string cw2::classifyNoughtOrCrossCenterHole(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  // 1) Compute centroid
  Eigen::Vector4f c;
  pcl::compute3DCentroid(*cloud, c);

  double cx = c[0];
  double cy = c[1];
  double cz = c[2];

  // 2) CropBox around the centroid
  float half = 0.02f;
  Eigen::Vector4f min_pt(cx - half, cy - half, cz - 0.03f, 1.0f);
  Eigen::Vector4f max_pt(cx + half, cy + half, cz + 0.03f, 1.0f);

  pcl::CropBox<pcl::PointXYZ> center_filter;
  center_filter.setInputCloud(cloud);
  center_filter.setMin(min_pt);
  center_filter.setMax(max_pt);

  pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  center_filter.filter(*center_cloud);

  // 3) If center region is empty => nought, else => cross
  if (center_cloud->empty())
  {
    ROS_INFO("Center is empty => nought");
    return "nought";
  }
  else
  {
    ROS_INFO("Center has %zu points => cross", center_cloud->size());
    return "cross";
  }
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Move the arm above a shape, capture cloud data, and classify it by center hole.
 * @param pt The approximate shape center point.
 * @return "nought" or "cross" or "nth" if classification fails.
 */
std::string cw2::captureAndClassifyShape(const geometry_msgs::Point &pt)
{
  // 1) Move camera overhead
  geometry_msgs::Pose view_pose;
  view_pose.position.x = pt.x;
  view_pose.position.y = pt.y;
  view_pose.position.z = 0.70;
  tf2::Quaternion q_down;
  q_down.setRPY(M_PI, 0, 0);
  view_pose.orientation = tf2::toMsg(q_down);

  if (!moveArm(view_pose))
  {
    ROS_WARN("Could not move above shape => default cross");
    return "nth";
  }
  ros::Duration(1.0).sleep(); // Wait for updated cloud

  // 2) Copy global cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*g_cloud_filtered, *raw_cloud);

  // 3) Remove plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  removePlane<pcl::PointXYZ>(raw_cloud, no_plane_cloud, 0.02);

  // Debug publish
  sensor_msgs::PointCloud2 debug_msg;
  pcl::toROSMsg(*no_plane_cloud, debug_msg);
  debug_msg.header.frame_id = "color";
  debug_msg.header.stamp    = ros::Time::now();
  debug_t2_pub_.publish(debug_msg);

  if (no_plane_cloud->empty())
  {
    ROS_WARN("After plane removal, no points found => guess cross?");
    return "nth";
  }

  // 4) Classify shape
  return classifyNoughtOrCrossCenterHole(no_plane_cloud);
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief shapeChecker to compare a "mystery point" to references.
 * @param ref_points Vector of reference shape points.
 * @param mystery_point The shape to identify.
 * @return Index of the matching reference (1 or 2), or -1 on error.
 */
int64_t cw2::shapeChecker(std::vector<geometry_msgs::PointStamped> ref_points,
                          geometry_msgs::PointStamped mystery_point)
{
  if (ref_points.size() < 2)
  {
    ROS_ERROR("task_2 needs 2 reference points, got only %zu", ref_points.size());
    return -1;
  }

  ROS_INFO("Inspecting first reference shape...");
  std::string ref_shape_1 = processShape(ref_points[0].point);

  // Infer second shape from the first
  std::string ref_shape_2;
  if (ref_shape_1 == "nought")
    ref_shape_2 = "cross";
  else
    ref_shape_2 = "nought";

  ROS_INFO("Inspecting second reference shape => inferred: %s", ref_shape_2.c_str());
  ROS_INFO("Inspecting the mystery shape...");
  std::string mystery_shape = processShape(mystery_point.point);

  // If mismatch => shape = reference #2
  int64_t mystery_object_num = 1;
  if (mystery_shape != ref_shape_1)
    mystery_object_num = 2;

  ROS_INFO("==============================================");
  ROS_INFO_STREAM("Ref Shape1: " << ref_shape_1
                  << " | Ref Shape2: " << ref_shape_2
                  << " | Mystery: " << mystery_shape);
  ROS_INFO_STREAM("Mystery belongs to reference #" << mystery_object_num);
  ROS_INFO("==============================================");

  return mystery_object_num;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief processShape uses color data to decide if something is "nought" or "cross."
 * @param object_point The shape center point in world coords.
 * @return "nought" or "cross" based on color check, default cross if no movement.
 */
std::string cw2::processShape(const geometry_msgs::Point &object_point)
{
  // 1) Move camera overhead
  geometry_msgs::Pose image_pose = point2Pose(object_point);
  image_pose.position.z += 0.30;
  image_pose.position.x -= 0.04;

  bool success = moveArm(image_pose);
  if (!success)
  {
    ROS_WARN("Processed Shape: Could not move above shape => defaulting to cross");
    return "cross";
  }

  ros::Duration(1.0).sleep(); // Wait for updated image

  // 2) Extract pixel color from the center
  int redValue   = color_image_data[color_image_midpoint_];
  int greenValue = color_image_data[color_image_midpoint_ + 1];
  int blueValue  = color_image_data[color_image_midpoint_ + 2];

  ROS_INFO("Processed Shape => R=%d, G=%d, B=%d", redValue, greenValue, blueValue);

  // 3) Classify shape by color
  if (greenValue > redValue && greenValue > blueValue)
  {
    ROS_INFO("Shape => NOUGHT");
    return "nought";
  }
  else
  {
    ROS_INFO("Shape => CROSS");
    return "cross";
  }
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Task 2 callback (shapeChecker logic).
 */
bool cw2::t2_callback(cw2_world_spawner::Task2Service::Request &req,
                      cw2_world_spawner::Task2Service::Response &res)
{
  int64_t result = shapeChecker(req.ref_object_points, req.mystery_object_point);
  if (result < 0)
  {
    return false;
  }
  res.mystery_object_num = result;
  return true;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Task 3 callback (scan environment, cluster shapes, pick the largest, place it).
 */
bool cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
                      cw2_world_spawner::Task3Service::Response &response)
{
  ROS_INFO("Task3 callback triggered...");

  // 1) Gather octomap data
  scanEnvironmentForTask3();

  // 2) Check final octomap data
  if (!g_octomap_filtered || g_octomap_filtered->empty())
  {
    ROS_WARN("g_octomap_filtered is empty => no objects => returning 0");
    response.total_num_shapes       = 0;
    response.num_most_common_shape  = 0;
    return true;
  }
  ROS_INFO_STREAM("[DEBUG] t3_callback: clustering g_octomap_filtered with size="
                  << g_octomap_filtered->size() << " points.");

  // 3) Cluster occupant-center pointcloud
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters
      = clusterSceneObjects(g_octomap_filtered);

  if (clusters.empty())
  {
    ROS_WARN("No clusters found => no shapes => returning 0");
    response.total_num_shapes      = 0;
    response.num_most_common_shape = 0;
    return true;
  }

  // (Optional) Merge clusters into color-coded cloud for debug
  pcl::PointCloud<pcl::PointXYZRGB> merged_cluster_cloud;
  for (int i = 0; i < (int)clusters.size(); ++i)
  {
    float r = (i * 50) % 255;
    float g = (i * 80) % 255;
    float b = (i * 110) % 255;
    for (auto &pt : clusters[i]->points)
    {
      pcl::PointXYZRGB pt_colored;
      pt_colored.x = pt.x;
      pt_colored.y = pt.y;
      pt_colored.z = pt.z;
      pt_colored.r = r;
      pt_colored.g = g;
      pt_colored.b = b;
      merged_cluster_cloud.push_back(pt_colored);
    }
  }
  sensor_msgs::PointCloud2 merged_msg;
  pcl::toROSMsg(merged_cluster_cloud, merged_msg);
  merged_msg.header.frame_id = "world";
  merged_msg.header.stamp    = ros::Time::now();
  cluster_pub_.publish(merged_msg);

  // Prepare data structures
  struct ClusterInfo {
    double dx, dy, dz;
    double volume;
    geometry_msgs::Point centroid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  };

  std::vector<ClusterInfo> basket_candidates;
  std::vector<ClusterInfo> normal_clusters;

  // 4) For each cluster => compute bounding box => store either basket candidate or normal
  for (auto &c : clusters)
  {
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*c, min_pt, max_pt);
    double dx = max_pt.x - min_pt.x;
    double dy = max_pt.y - min_pt.y;
    double dz = max_pt.z - min_pt.z;
    double volume = dx * dy * dz;

    Eigen::Vector4f centroid_eig;
    pcl::compute3DCentroid(*c, centroid_eig);

    geometry_msgs::Point center_msg;
    center_msg.x = centroid_eig[0];
    center_msg.y = centroid_eig[1];
    center_msg.z = 0.0; // or actual Z

    bool basket_like = (dx > 0.25 && dy > 0.25);
    ClusterInfo info{dx, dy, dz, volume, center_msg, c};

    if (basket_like)
    {
      basket_candidates.push_back(info);
    }
    else
    {
      normal_clusters.push_back(info);
    }
  }

  // 5) Identify the true basket by largest volume
  bool foundBasket = false;
  geometry_msgs::Point basket_position;
  double largest_basket_volume = 0.0;

  for (auto &cand : basket_candidates)
  {
    if (cand.volume > largest_basket_volume)
    {
      largest_basket_volume = cand.volume;
      basket_position       = cand.centroid;
      foundBasket           = true;
    }
  }

  if (foundBasket)
  {
    ROS_INFO("Basket bounding box volume=%.3f at (%.2f,%.2f)",
             largest_basket_volume, basket_position.x, basket_position.y);
  }

  // 6) Classify shapes among normal clusters
  std::vector<std::pair<double, geometry_msgs::Point>> nought_candidates;
  std::vector<std::pair<double, geometry_msgs::Point>> cross_candidates;
  int obstacle_count = 0;

  for (auto &nc : normal_clusters)
  {
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*nc.cloud, min_pt, max_pt);
    double dx = max_pt.x - min_pt.x;
    double dy = max_pt.y - min_pt.y;
    double dz = max_pt.z - min_pt.z;
    double volume = dx * dy * dz;

    // Color-based classification
    std::string shape_label = classifyCluster(nc.centroid, dz);

    if (shape_label == "nought")
    {
      nought_candidates.emplace_back(nc.volume, nc.centroid);
    }
    else if (shape_label == "cross")
    {
      cross_candidates.emplace_back(nc.volume, nc.centroid);
    }
    else
    {
      obstacle_count++;
      ROS_INFO("Skipping => obstacle or unknown shape");
    }
  }

  // Summaries
  int nought_count = nought_candidates.size();
  int cross_count  = cross_candidates.size();
  int total_num_shapes = nought_count + cross_count;
  response.total_num_shapes = total_num_shapes;

  if (total_num_shapes == 0)
  {
    ROS_WARN("No (nought/cross) shapes => returning 0");
    response.num_most_common_shape = 0;
    return true;
  }

  // Choose the most common
  int best_count = (nought_count >= cross_count) ? nought_count : cross_count;
  response.num_most_common_shape = best_count;

  // 7) Depending on which shape is more common, pick the largest cluster of that shape
  geometry_msgs::Point pick_pt;
  std::string pick_label;

  if (nought_count >= cross_count && !nought_candidates.empty())
  {
    // Largest nought
    auto best_nought_it = std::max_element(
        nought_candidates.begin(),
        nought_candidates.end(),
        [](auto &a, auto &b) { return a.first < b.first; });
    pick_pt    = best_nought_it->second;
    pick_label = "nought";
  }
  else if (!cross_candidates.empty())
  {
    // Largest cross
    auto best_cross_it = std::max_element(
        cross_candidates.begin(),
        cross_candidates.end(),
        [](auto &a, auto &b) { return a.first < b.first; });
    pick_pt    = best_cross_it->second;
    pick_label = "cross";
  }
  else
  {
    ROS_WARN("No shape positions found => skipping pick");
    return true;
  }

  // Execute the pick
  if (!pickShapeFromTask3(pick_pt, pick_label))
  {
    ROS_ERROR("Could not pick => skipping place");
    return false;
  }

  // Place in basket if found
  if (!foundBasket)
  {
    ROS_ERROR("No basket => skipping place");
  }
  else
  {
    if (!placeObject(basket_position))
    {
      ROS_ERROR("Failed to place => continuing anyway");
    }
  }

  ROS_INFO_STREAM("Task3 => total=" << total_num_shapes
                  << ", most_common=" << best_count);
  return true;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Clusters a point cloud for object segmentation.
 * @param cloud_in The input point cloud to cluster.
 * @return A vector of clusters (each a separate point cloud).
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
cw2::clusterSceneObjects(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

  // Create a KdTree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_in);

  // Cluster extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
  cluster_extraction.setClusterTolerance(0.04); // ~4cm
  cluster_extraction.setMinClusterSize(50);
  cluster_extraction.setMaxClusterSize(10000);
  cluster_extraction.setSearchMethod(tree);
  cluster_extraction.setInputCloud(cloud_in);
  cluster_extraction.extract(cluster_indices);

  for (const auto &cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &idx : cluster.indices)
    {
      cloud_cluster->push_back((*cloud_in)[idx]);
    }
    cloud_cluster->width    = cloud_cluster->size();
    cloud_cluster->height   = 1;
    cloud_cluster->is_dense = true;
    ROS_INFO("PointCloud representing the Cluster: %lu data points.", cloud_cluster->size());
    clusters.push_back(cloud_cluster);
  }

  return clusters;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Publishes the current debug cloud (e.g., g_cloud_filtered) after optional transforms.
 * @param info A string label to accompany the publication.
 */
void cw2::publishCurrentDebugCloud(const std::string &info)
{
  // If g_cloud_filtered is empty, warn and return
  if (!g_cloud_filtered || g_cloud_filtered->empty())
  {
    ROS_WARN_STREAM("publishCurrentDebugCloud: g_cloud_filtered is empty at " << info);
    return;
  }

  // Transform from camera frame => "world"
  std::string sensor_frame = "color";
  std::string target_frame = "world";

  if (!transformCloudUsingTF2<pcl::PointXYZRGBA>(g_cloud_filtered, sensor_frame, target_frame))
  {
    ROS_WARN("Transform failed => skipping debug publish");
    return;
  }

  // Convert and publish
  sensor_msgs::PointCloud2 debug_msg;
  pcl::toROSMsg(*g_cloud_filtered, debug_msg);
  debug_msg.header.frame_id = "world";
  debug_msg.header.stamp    = ros::Time::now();

  debug_pub_.publish(debug_msg);
  ROS_INFO_STREAM("Published debug cloud (" << g_cloud_filtered->size()
                  << " pts) at step: " << info);
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Moves the arm in a path to gather environment data for Task 3.
 */
void cw2::scanEnvironmentForTask3()
{
  // Move to a "reset" or "home" pose
  geometry_msgs::Point reset_point;
  reset_point.x = 0.5;
  reset_point.y = 0.0;
  reset_point.z = 0.5;
  geometry_msgs::Pose reset_pose = point2Pose(reset_point);
  moveArm(reset_pose); // ignoring success check for brevity

  // Set slower speed
  move_group_arm_.setMaxVelocityScalingFactor(0.05);

  // Define corners of the scan area
  geometry_msgs::Point corner1, corner2, corner3, corner4;
  corner1.x = -0.50; corner1.y = -0.40; corner1.z = 0.55;
  corner2.x =  0.50; corner2.y = -0.40; corner2.z = 0.60;
  corner3.x =  0.50; corner3.y =  0.30; corner3.z = 0.55;
  corner4.x = -0.50; corner4.y =  0.30; corner4.z = 0.55;

  std::vector<geometry_msgs::Point> corners{corner1, corner2, corner3, corner4, corner1};

  double angle = -M_1_PI/2; // constant yaw
  int num_steps = 4;

  // Loop corners
  for (int i = 0; i < (int)corners.size() - 1; i++)
  {
    // Move to corner i
    geometry_msgs::Pose pose_i = point2Pose(corners[i], angle);
    moveArm(pose_i);

    // Wait for fresh sensor data
    if (i == 0) task_3_filter = true;
    ros::Duration(1.0).sleep();

    // Step increments between corners i and i+1
    geometry_msgs::Point distance;
    distance.x = corners[i].x - corners[i + 1].x;
    distance.y = corners[i].y - corners[i + 1].y;
    distance.z = 0;

    for (int j = 1; j < num_steps - 1; j++)
    {
      geometry_msgs::Point step;
      step.x = corners[i].x - (j * distance.x / num_steps);
      step.y = corners[i].y - (j * distance.y / num_steps);
      step.z = corners[i].z;

      ROS_INFO("Step: (%f, %f, %f)", step.x, step.y, step.z);

      geometry_msgs::Pose step_pose = point2Pose(step, angle);
      moveArm(step_pose);
      ros::Duration(1.0).sleep(); // wait for fresh sensor data
    }
    task_3_filter = false;
  }

  ros::Duration(1.0).sleep();
  // Reset speed
  move_group_arm_.setMaxVelocityScalingFactor(0.1);
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Publishes a debug cloud (pointXYZ) after transforming to the target frame.
 * @param cloud_in The input cloud pointer.
 * @param sensor_frame Source frame for the cloud.
 * @param target_frame Target frame for publication.
 */
void cw2::publishDebugCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                            const std::string &sensor_frame,
                            const std::string &target_frame)
{
  if (!cloud_in || cloud_in->empty())
  {
    ROS_WARN("publishDebugCloud: cloud_in is empty. Not publishing.");
    return;
  }

  // 1) Copy
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>(*cloud_in));

  // 2) Transform
  if (!transformCloudUsingTF2<pcl::PointXYZ>(transformed, sensor_frame, target_frame))
  {
    ROS_WARN("Could not transform cloud_in => skipping publish");
    return;
  }

  // 3) Convert to ROS msg and publish
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*transformed, cloud_msg);
  cloud_msg.header.frame_id = target_frame;
  cloud_msg.header.stamp    = ros::Time::now();

  debug_pub_cloud_in.publish(cloud_msg);

  ROS_INFO_STREAM("publishDebugCloud: published "
                  << transformed->size() << " points to /debug_cloud_in in frame "
                  << target_frame);
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Classify cluster color (nought/cross/obstacle) by moving camera overhead and checking RGB.
 * @param object_point The cluster centroid in the world.
 * @param dz The cluster's height (for advanced logic if needed).
 * @return "nought", "cross", or "obstacle" (default "cross" on failures).
 */
std::string cw2::classifyCluster(const geometry_msgs::Point &object_point, double dz)
{
  // Move camera overhead
  geometry_msgs::Point image_point = object_point;
  image_point.x -= 0.043;
  image_point.z  = 0.5;

  geometry_msgs::Pose image_pose = point2Pose(image_point);
  bool success = moveArm(image_pose);
  if (!success)
  {
    ROS_WARN("Could not move camera to classify => defaulting to cross?");
    return "cross";
  }

  // Read center pixel
  int redValue   = color_image_data[color_image_midpoint_];
  int greenValue = color_image_data[color_image_midpoint_ + 1];
  int blueValue  = color_image_data[color_image_midpoint_ + 2];

  ROS_INFO("red: %d, green: %d, blue: %d", redValue, greenValue, blueValue);

  if (redValue < 50 && greenValue < 50 && blueValue < 50)
  {
    return "obstacle";
  }
  else if (greenValue > redValue && greenValue > blueValue)
  {
    return "nought";
  }
  else
  {
    return "cross";
  }
}

///////////////////////////////////////////////////////////////////////////////
/**
 * @brief Pick a shape (nought/cross) identified in Task 3 from its occupant-center.
 * @param shape_centroid The shape's centroid in "world" coords.
 * @param shape_label "nought" or "cross".
 * @return True if pick was successful, false otherwise.
 */
bool cw2::pickShapeFromTask3(const geometry_msgs::Point &shape_centroid,
                             const std::string &shape_label)
{
  // Move camera overhead
  if (!moveCameraAbove(shape_centroid, 0.60))
  {
    ROS_ERROR("pickShapeFromTask3: moveCameraAbove failed");
    return false;
  }
  ros::Duration(1.0).sleep();

  // Acquire raw cloud from your T1 pointers
  auto raw_cloud = getObjectCloud(shape_label);
  if (!raw_cloud)
  {
    return false;
  }

  // Compute orientation & pick
  geometry_msgs::Point local_pt = shape_centroid; // copy
  bool success = computeOrientationAndPick(raw_cloud, local_pt, shape_label);
  if (!success)
  {
    ROS_ERROR("pickShapeFromTask3: computeOrientationAndPick => pick failed");
    return false;
  }

  ROS_INFO("pickShapeFromTask3 => success!");
  return true;
}
