cmake_minimum_required(VERSION 2.8.7)

project(zed_sync_test)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(OpenCV REQUIRED)

set(DEPENDENCIES
    roscpp
    nodelet    
    image_transport
    message_filters
    sensor_msgs
)

find_package(catkin COMPONENTS
    ${DEPENDENCIES}
)

catkin_package(
    CATKIN_DEPENDS
    ${DEPENDENCIES}
)

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${extra_files})

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
)

###############################################################################
# SOURCES

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/rgbd_test_sync.cpp)

###############################################################################
# nodelet lib
add_definitions(-std=c++11 -Wno-deprecated-declarations)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_library(ZEDSyncTest
    ${SRC}
)
target_link_libraries(ZEDSyncTest
    ${LINK_LIBRARIES}
)
add_dependencies(
    ZEDSyncTest
    ${catkin_EXPORTED_TARGETS}
)

###############################################################################
# INSTALL

install(TARGETS
  ZEDSyncTest
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

