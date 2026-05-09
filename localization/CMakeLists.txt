add_library(astro_localization
  src/star_tracker.cpp
  src/stereo_visual_odometry.cpp
  src/visual_odometry.cpp
  src/pair_index_loader.cpp
  src/pair_id_solver.cpp)
add_library(astro_localization::localization ALIAS astro_localization)

target_include_directories(astro_localization PUBLIC include)
target_link_libraries(
  astro_localization
  PUBLIC astro_project_options astro_project_warnings astro_localization::core Eigen3::Eigen ${OpenCV_LIBS})
