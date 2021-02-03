cmake_minimum_required(VERSION 3.5)
project(rviz_plugin_zed_od)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Remove FindOpenGL warning
cmake_policy(SET CMP0072 NEW)

# Tests currently only run on OS X @ OSRF jenkins
# Enable on Linux by providing a display, enable on Windows via EnableDisplayTests=True
option(EnableDisplayTests "EnableDisplayTests")
set(DisplayTests "False" CACHE STRING "DisplayTestsVariable")

if(DEFINED ENV{DISPLAY})
  set(DISPLAYPRESENT TRUE)
endif()

if(APPLE OR DISPLAYPRESENT OR EnableDisplayTests STREQUAL "True")
  message(STATUS "Enabling tests requiring a display")
else()
  set(SKIP_DISPLAY_TESTS "SKIP_TEST")
endif()

# options and directories for visual tests (see visual_testing_framework documentation)
option(EnableVisualTests "decides whether or not to enable the tests")

# Run visual tests only if "EnableVisualTests=True"
if(EnableVisualTests STREQUAL "True")
  message(STATUS "Enabling visual tests")
else()
  set(SKIP_VISUAL_TESTS "SKIP_TEST")
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(zed_interfaces REQUIRED)

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INCLUDES and LIBS
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/plugin/include
)
###############################################################################
# SOURCES
set(PLUGIN_SRC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/plugin/src/zed_od_display.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/plugin/src/zed_od_info.cpp
)

set(PLUGIN_INC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include/visibility_control.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/plugin/include/zed_od_display.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/plugin/include/zed_od_info.hpp
)

###############################################################################
# BIN & INSTALLATION
add_library(rviz_plugin_zed_od SHARED
  ${PLUGIN_INC}
  ${PLUGIN_SRC}
)

target_include_directories(rviz_plugin_zed_od PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rviz_rendering_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${zed_interfaces_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${resource_retriever_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

#target_link_libraries(rviz_plugin_zed_od
#  resource_retriever::resource_retriever
#  rviz_common::rviz_common
#)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_plugin_zed_od PRIVATE "ZED_OD_PLUGIN_BUILDING_DLL")

# prevent pluginlib from using boost
target_compile_definitions(rviz_plugin_zed_od PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(rviz_plugin_zed_od
  PUBLIC
  zed_interfaces
)

ament_export_include_directories(include)
ament_export_interfaces(rviz_plugin_zed_od HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  rclcpp
  zed_interfaces
)

install(
  TARGETS rviz_plugin_zed_od
  EXPORT rviz_plugin_zed_od
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/src/include/
  DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/src/plugin/include/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_package()

