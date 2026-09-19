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
