add_executable(lunar_visual_odometry lunar_visual_odometry.cpp)
target_link_libraries(
  lunar_visual_odometry
  PRIVATE astro_project_options
          astro_project_warnings
          astro_navigation::core
          astro_navigation::localization
          astro_navigation::crater
          astro_navigation::visualization)

add_executable(stereo_visual_odometry stereo_visual_odometry.cpp)
target_link_libraries(
  stereo_visual_odometry
  PRIVATE astro_project_options
          astro_project_warnings
          astro_navigation::core
          astro_navigation::localization
          astro_navigation::visualization)

add_executable(star_tracker_attitude star_tracker_attitude.cpp)
target_link_libraries(
  star_tracker_attitude
  PRIVATE astro_project_options
          astro_project_warnings
          astro_navigation::core
          astro_navigation::localization)

add_executable(mission_navigation_demo mission_navigation_demo.cpp)
target_link_libraries(
  mission_navigation_demo
  PRIVATE astro_project_options
          astro_project_warnings
          astro_navigation::core
          astro_navigation::localization
          astro_navigation::navigation)

add_executable(hazard_route_demo hazard_route_demo.cpp)
target_link_libraries(
  hazard_route_demo
  PRIVATE astro_project_options
          astro_project_warnings
          astro_navigation::navigation)

add_executable(lost_in_space_pair_id lost_in_space_pair_id.cpp)
target_link_libraries(
  lost_in_space_pair_id
  PRIVATE astro_project_options
          astro_project_warnings
          astro_navigation::localization)

add_executable(build_star_pair_index build_star_pair_index.cpp)
target_link_libraries(
  build_star_pair_index
  PRIVATE astro_project_options
          astro_project_warnings
          Eigen3::Eigen)
