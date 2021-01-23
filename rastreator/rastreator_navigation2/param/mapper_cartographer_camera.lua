-- /* Author: Iñaki Lorente */

include "map_builder.lua"
include "trajectory_builder.lua"

--TIME IN SECONDS
--DISTANCE IN METERS

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  --If working with intelrealsense T265 camera use --> t265_imu
  --else --> imu_link
  tracking_frame = "odom",
  published_frame = "odom",
  odom_frame = "odom", 
  --activar si existe odometria adicional
  provide_odom_frame = false, 
  publish_frame_projected_to_2d = true,
  --Usar odometria adicional
  use_odometry = true, 
  use_nav_sat = false,
  use_landmarks = false,
  --si acticado, realiza mapa a partir de laser
  num_laser_scans = 1, 
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  --Si activado, realiza mapa apartir de point cloud
  num_point_clouds = 0, 
  lookup_transform_timeout_sec = 1.0,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-------------------------------------------------------------------
--- PARAMETRIZACION -----------------------------------------------
-------------------------------------------------------------------

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4 --(CORE, Por defecto 4)

TRAJECTORY_BUILDER_2D.min_range = 0.1 --(Por defecto 0.0)
TRAJECTORY_BUILDER_2D.max_range = 20. --(Por defecto 30)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5. --(Por defecto 5)

--Si se usa IMU marcar imu, sino marcar CSM
TRAJECTORY_BUILDER_2D.use_imu_data = false  --(Por defecto true)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true --(Por defecto false)
--CSM
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.20 --(Por defecto 0.1)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.) --(Por defecto math.rad(20.))

--POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e1  --(Por defecto 1.1e4)
--POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e6 --(Por defecto 1e5)
--POSE_GRAPH.optimization_problem.huber_scale = 4e2 --(Por defecto 1e1 )
--POSE_GRAPH.constraint_builder.sampling_ratio = 0.1 --(Por defecto 0.3 )
POSE_GRAPH.constraint_builder.min_score = 0.8 --(Por defecto 0.55 )
POSE_GRAPH.optimize_every_n_nodes = 100 -- 0=NO GLOBAL MAP (Por defecto 90)


TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --(Por defecto 1)
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 30. --(Por defecto 50)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)




return options