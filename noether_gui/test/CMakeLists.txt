find_gtest()

add_executable(${PROJECT_NAME}_path_edit_widget_utest path_edit_widget_utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_path_edit_widget_utest PRIVATE ${PROJECT_NAME} Qt5::Widgets GTest::GTest GTest::Main
)
add_gtest_discover_tests(${PROJECT_NAME}_path_edit_widget_utest)

# Les chemins de reference vivent avec le modele qu'ils oraclent, dans noether_tpp.
target_compile_definitions(
  ${PROJECT_NAME}_path_edit_widget_utest
  PRIVATE TOOL_PATH_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../noether_tpp/test/tool_paths"
)

add_executable(${PROJECT_NAME}_outline_tool_utest outline_tool_utest.cpp)
target_link_libraries(${PROJECT_NAME}_outline_tool_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_outline_tool_utest)

add_executable(${PROJECT_NAME}_pipeline_direction_utest pipeline_direction_utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_pipeline_direction_utest PRIVATE ${PROJECT_NAME} Qt5::Widgets GTest::GTest GTest::Main
)
add_gtest_discover_tests(${PROJECT_NAME}_pipeline_direction_utest)
target_compile_definitions(
  ${PROJECT_NAME}_pipeline_direction_utest
  PRIVATE GUI_PLUGIN_DIR="$<TARGET_FILE_DIR:${PROJECT_NAME}_plugins>"
          TPP_PLUGIN_DIR="$<TARGET_FILE_DIR:noether::noether_tpp_plugins>"
)

add_executable(${PROJECT_NAME}_trace_tool_utest trace_tool_utest.cpp)
target_link_libraries(${PROJECT_NAME}_trace_tool_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_trace_tool_utest)

add_executable(${PROJECT_NAME}_step_import_utest step_import_utest.cpp)
target_link_libraries(${PROJECT_NAME}_step_import_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_step_import_utest)

add_executable(${PROJECT_NAME}_translator_utest translator_utest.cpp)
target_link_libraries(${PROJECT_NAME}_translator_utest PRIVATE ${PROJECT_NAME} Qt5::Widgets GTest::GTest)
add_gtest_discover_tests(${PROJECT_NAME}_translator_utest)

add_executable(${PROJECT_NAME}_pass_surface_utest pass_surface_utest.cpp)
target_link_libraries(${PROJECT_NAME}_pass_surface_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_pass_surface_utest)

add_executable(${PROJECT_NAME}_edited_path_utest edited_path_utest.cpp)
target_link_libraries(${PROJECT_NAME}_edited_path_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_edited_path_utest)

add_executable(${PROJECT_NAME}_point_edit_utest point_edit_utest.cpp)
target_link_libraries(${PROJECT_NAME}_point_edit_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_point_edit_utest)

add_executable(${PROJECT_NAME}_view_cube_utest view_cube_utest.cpp)
target_link_libraries(${PROJECT_NAME}_view_cube_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_view_cube_utest)

add_executable(${PROJECT_NAME}_tpp_widget_utest tpp_widget_utest.cpp)
target_link_libraries(${PROJECT_NAME}_tpp_widget_utest PRIVATE ${PROJECT_NAME} Qt5::Widgets GTest::GTest)
add_gtest_discover_tests(${PROJECT_NAME}_tpp_widget_utest)
