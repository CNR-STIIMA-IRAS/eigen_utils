cmake_minimum_required(VERSION 2.8.3)
project(eigen_matrix_utils)

add_compile_options(-std=c++11 -funroll-loops  -Ofast)

find_package(catkin REQUIRED COMPONENTS
  roscpp rosparam_utilities
)
find_package(Eigen3 REQUIRED COMPONENTS Core Dense)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES eigen_matrix_utils
  CATKIN_DEPENDS roscpp rosparam_utilities
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/eigen_matrix_utils.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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


