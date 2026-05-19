add_library(astro_terrain src/landmarks.cpp)
add_library(astro_navigation::terrain ALIAS astro_terrain)

target_include_directories(astro_terrain PUBLIC include)
target_link_libraries(
  astro_terrain
  PUBLIC astro_project_options astro_project_warnings astro_navigation::core ${OpenCV_LIBS})

