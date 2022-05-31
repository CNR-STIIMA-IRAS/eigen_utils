cmake_minimum_required(VERSION 3.9)

project(eigen_matrix_utils)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

cnr_set_flags()

find_package(catkin REQUIRED COMPONENTS)
find_package(Eigen3 REQUIRED COMPONENTS Core Dense)


catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES eigen_matrix_utils
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/eigen_matrix_utils.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
cnr_target_compile_options(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING  PATTERN "*.h"
                  PATTERN "*.hpp"
                  PATTERN ".svn" EXCLUDE
)


