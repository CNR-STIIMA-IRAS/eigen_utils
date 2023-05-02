#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.9.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

get_project_name("package.xml" extracted_name extracted_version)

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
  project(${extracted_name})
else()
  project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)
endif()

cnr_set_flags()
#############################
## END - PRELIMINARIES     ##
#############################


#############################
## OPTIONS                 ##
#############################
option(USE_ROS1                 "ROS SUPPORT" ON)
option(ENABLE_TESTING           "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" OFF)
option(COMPILE_EXAMPLE          "COMPILE THE EXAMPLE" OFF)

if(USE_ROS1)
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
endif()
#############################
## END - OPTIONS           ##
#############################


#############################
## DEPENDECIES             ##
#############################
find_package(Eigen3 REQUIRED COMPONENTS Core Dense)

if(USE_ROS1)
  find_package(catkin REQUIRED COMPONENTS)

  catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES eigen_matrix_utils
    DEPENDS )
endif()
#############################
## END DEPENDECIES         ##
#############################


#############################
## TARGETS                 ##
#############################
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/eigen_matrix_utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
  $<INSTALL_INTERFACE:include> 
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
cnr_target_compile_options(${PROJECT_NAME})
#############################
## END TARGETS             ##
#############################


#############################
## INSTALL                 ##
#############################
if(USE_ROS1)
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
else()

  install(
    DIRECTORY include/
    DESTINATION include
  )

  install(
    TARGETS eigen_matrix_utils
    EXPORT export_eigen_matrix_utils
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # export targets
  set(export_targets ${export_targets};eigen_matrix_utils)
  export(EXPORT export_eigen_matrix_utils
    FILE "${PROJECT_BINARY_DIR}/export_eigen_matrix_utils.cmake")

  # Create the eigen_matrix_utilsConfig.cmake
  set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
  configure_file(eigen_matrix_utilsConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/eigen_matrix_utilsConfig.cmake" @ONLY)

  # Create the eigen_matrix_utilsConfigVersion.cmake
  configure_file(eigen_matrix_utilsConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/eigen_matrix_utilsConfigVersion.cmake" @ONLY)

  install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/eigen_matrix_utilsConfig.cmake"
    "${PROJECT_BINARY_DIR}/eigen_matrix_utilsConfigVersion.cmake"
    
    DESTINATION "share/eigen_matrix_utils/cmake" COMPONENT dev)
  install(EXPORT export_eigen_matrix_utils
    DESTINATION "share/eigen_matrix_utils/cmake"
    FILE export_eigen_matrix_utils.cmake
    COMPONENT dev)

endif()
#############################
## END INSTALL             ##
#############################