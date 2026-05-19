add_library(astro_nav
  src/hazard_guidance.cpp
  src/pipeline.cpp
  src/state.cpp
  src/state_io.cpp
  src/trn_summary.cpp)
add_library(astro_navigation::navigation ALIAS astro_nav)

target_include_directories(astro_nav PUBLIC include)
target_link_libraries(
  astro_nav
  PUBLIC astro_project_options
         astro_project_warnings
         astro_navigation::core
         astro_navigation::localization
         Eigen3::Eigen)
