cmake_minimum_required(VERSION 3.5.1)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)

project(
  ${pkg_extracted_name}
  VERSION ${pkg_extracted_version}
  LANGUAGES C CXX
)

find_package(Eigen3 REQUIRED)
# OpenCASCADE fournit son propre fichier de configuration, installe par les paquets libocct-*-dev
find_package(OpenCASCADE REQUIRED)

option(NOETHER_ENABLE_TESTING "Enables compilation with unit tests" OFF)
option(NOETHER_ENABLE_CLANG_TIDY "Enables compilation with clang-tidy" OFF)

# Bibliotheque : l'API publique n'expose aucun type OpenCASCADE
add_library(${PROJECT_NAME} SHARED src/cad_model.cpp src/step_unit.cpp src/step_reader.cpp src/mesh_writer.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include>"
)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PRIVATE TKSTEP
          TKXSBase
          TKBRep
          TKG3d
          TKG2d
          TKMesh
          TKPrim
          TKTopAlgo
          TKGeomAlgo
          TKGeomBase
          TKShHealing
          TKernel
          TKMath
)
target_cxx_version(
  ${PROJECT_NAME}
  PUBLIC
  VERSION 17
)
target_clang_tidy(${PROJECT_NAME} ENABLE ${NOETHER_ENABLE_CLANG_TIDY})

# Ligne de commande : sous-commandes info et mesh
add_executable(${PROJECT_NAME}_app src/app/noether_cad.cpp)
target_link_libraries(${PROJECT_NAME}_app PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_app PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_cxx_version(
  ${PROJECT_NAME}_app
  PUBLIC
  VERSION 17
)

install(DIRECTORY include/ DESTINATION include/)

if(${NOETHER_ENABLE_TESTING})
  enable_testing()
  add_subdirectory(test)
endif()

configure_package(
  NAMESPACE noether
  DEPENDENCIES Eigen3
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_app
)

# L'executable est aussi installe la ou ros2 run le cherche
install(TARGETS ${PROJECT_NAME}_app DESTINATION lib/${PROJECT_NAME})
