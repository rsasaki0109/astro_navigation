add_library(astro_core src/image_sequence.cpp)
add_library(astro_localization::core ALIAS astro_core)

target_include_directories(astro_core PUBLIC include)
target_link_libraries(
  astro_core
  PUBLIC astro_project_options astro_project_warnings Eigen3::Eigen ${OpenCV_LIBS})

