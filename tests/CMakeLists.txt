add_executable(navigation_state_smoke navigation_state_smoke.cpp)
target_link_libraries(
  navigation_state_smoke
  PRIVATE astro_project_options
          astro_project_warnings
          astro_navigation::navigation)

add_test(
  NAME navigation_state_smoke
  COMMAND
    navigation_state_smoke
    ${CMAKE_SOURCE_DIR}/docs/figures/trn_lro_tycho_terminal/summary.json
    ${CMAKE_BINARY_DIR}/test_outputs/navigation_state_smoke)

add_executable(hazard_guidance_smoke hazard_guidance_smoke.cpp)
target_link_libraries(
  hazard_guidance_smoke
  PRIVATE astro_project_options
          astro_project_warnings
          astro_navigation::navigation)

add_test(NAME hazard_guidance_smoke COMMAND hazard_guidance_smoke)

if(TARGET mission_navigation_demo)
  add_executable(mission_navigation_cli_smoke mission_navigation_cli_smoke.cpp)
  target_link_libraries(
    mission_navigation_cli_smoke
    PRIVATE astro_project_options
            astro_project_warnings
            Eigen3::Eigen)
  add_dependencies(mission_navigation_cli_smoke mission_navigation_demo)

  add_test(
    NAME mission_navigation_cli_smoke
    COMMAND
      mission_navigation_cli_smoke
      $<TARGET_FILE:mission_navigation_demo>
      ${CMAKE_SOURCE_DIR}/docs/figures/trn_lro_tycho_terminal/summary.json
      ${CMAKE_BINARY_DIR}/test_outputs/mission_navigation_cli_smoke)
endif()

if(TARGET hazard_route_demo)
  add_executable(hazard_route_cli_smoke hazard_route_cli_smoke.cpp)
  target_link_libraries(
    hazard_route_cli_smoke
    PRIVATE astro_project_options
            astro_project_warnings)
  add_dependencies(hazard_route_cli_smoke hazard_route_demo)

  add_test(
    NAME hazard_route_cli_smoke
    COMMAND
      hazard_route_cli_smoke
      $<TARGET_FILE:hazard_route_demo>
      ${CMAKE_BINARY_DIR}/test_outputs/hazard_route_cli_smoke)
endif()
