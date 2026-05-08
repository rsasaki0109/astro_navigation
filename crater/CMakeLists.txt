add_library(astro_crater src/crater_detector.cpp)
add_library(astro_localization::crater ALIAS astro_crater)

target_include_directories(astro_crater PUBLIC include)
target_link_libraries(
  astro_crater
  PUBLIC astro_project_options astro_project_warnings astro_localization::core ${OpenCV_LIBS})

