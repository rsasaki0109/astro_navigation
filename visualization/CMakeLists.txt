add_library(astro_visualization src/trajectory_io.cpp)
add_library(astro_localization::visualization ALIAS astro_visualization)

target_include_directories(astro_visualization PUBLIC include)
target_link_libraries(
  astro_visualization
  PUBLIC astro_project_options astro_project_warnings astro_localization::core)

