add_library(astro_navigation
  src/star_tracker.cpp
  src/stereo_visual_odometry.cpp
  src/visual_odometry.cpp
  src/pair_index_loader.cpp
  src/pair_id_solver.cpp)
add_library(astro_navigation::localization ALIAS astro_navigation)

target_include_directories(astro_navigation PUBLIC include)
target_link_libraries(
  astro_navigation
  PUBLIC astro_project_options astro_project_warnings astro_navigation::core Eigen3::Eigen ${OpenCV_LIBS})
