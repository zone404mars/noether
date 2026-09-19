find_gtest()

add_executable(${PROJECT_NAME}_tool_path_modifier_utest tool_path_modifier_utest.cpp)
target_link_libraries(${PROJECT_NAME}_tool_path_modifier_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_tool_path_modifier_utest)
target_compile_definitions(${PROJECT_NAME}_tool_path_modifier_utest PRIVATE PLUGIN_DIR="${CMAKE_CURRENT_BINARY_DIR}/..")

add_executable(${PROJECT_NAME}_mesh_modifier_utest mesh_modifier_utest.cpp)
target_link_libraries(${PROJECT_NAME}_mesh_modifier_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_mesh_modifier_utest)
target_compile_definitions(
  ${PROJECT_NAME}_mesh_modifier_utest PRIVATE MESH_DIR="${CMAKE_CURRENT_SOURCE_DIR}/meshes"
                                              PLUGIN_DIR="${CMAKE_CURRENT_BINARY_DIR}/.."
)

add_executable(${PROJECT_NAME}_face_selection_utest face_selection_utest.cpp)
target_link_libraries(${PROJECT_NAME}_face_selection_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_face_selection_utest)
target_compile_definitions(${PROJECT_NAME}_face_selection_utest PRIVATE MESH_DIR="${CMAKE_CURRENT_SOURCE_DIR}/meshes")

add_executable(${PROJECT_NAME}_tool_path_planner_utest tool_path_planner_utest.cpp)
target_link_libraries(${PROJECT_NAME}_tool_path_planner_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_tool_path_planner_utest)
target_compile_definitions(
  ${PROJECT_NAME}_tool_path_planner_utest PRIVATE MESH_DIR="${CMAKE_CURRENT_SOURCE_DIR}/meshes"
                                                  PLUGIN_DIR="${CMAKE_CURRENT_BINARY_DIR}/.."
)

add_executable(${PROJECT_NAME}_tool_path_recipe_utest tool_path_recipe_utest.cpp)
target_link_libraries(${PROJECT_NAME}_tool_path_recipe_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_tool_path_recipe_utest)
target_compile_definitions(
  ${PROJECT_NAME}_tool_path_recipe_utest PRIVATE TOOL_PATH_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tool_paths"
                                                 TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}"
)

add_executable(${PROJECT_NAME}_occlusion_modifier_utest occlusion_modifier_utest.cpp)
target_link_libraries(${PROJECT_NAME}_occlusion_modifier_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_occlusion_modifier_utest)

add_executable(${PROJECT_NAME}_face_coverage_utest face_coverage_utest.cpp)
target_link_libraries(${PROJECT_NAME}_face_coverage_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_face_coverage_utest)

add_executable(${PROJECT_NAME}_start_end_organization_utest start_end_organization_utest.cpp)
target_link_libraries(${PROJECT_NAME}_start_end_organization_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_start_end_organization_utest)
