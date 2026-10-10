find_gtest()
find_package(yaml-cpp REQUIRED)

# Les tests fabriquent leurs STEP avec OpenCASCADE : ils en ont besoin directement
add_executable(${PROJECT_NAME}_step_reader_utest step_reader_utest.cpp)
target_include_directories(${PROJECT_NAME}_step_reader_utest SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(
  ${PROJECT_NAME}_step_reader_utest
  PRIVATE ${PROJECT_NAME}
          TKSTEP
          TKXSBase
          TKBRep
          TKPrim
          TKG3d
          TKGeomBase
          TKTopAlgo
          TKernel
          TKMath
          yaml-cpp
          GTest::GTest
          GTest::Main
)
target_cxx_version(
  ${PROJECT_NAME}_step_reader_utest
  PRIVATE
  VERSION 17
)
add_gtest_discover_tests(${PROJECT_NAME}_step_reader_utest)
