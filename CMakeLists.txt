# 
#  Copyright (C) 2016 Walkman
#  Author: Enrico Mingo Hoffman
#  email: enrico.mingo@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(trajectory_utils)

FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(kdl_parser REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(roslib QUIET)
FIND_PACKAGE(tf QUIET)
FIND_PACKAGE(nav_msgs QUIET)
FIND_PACKAGE(rviz_visual_tools QUIET)

# compilation flags
option(TRAJECTORY_UTILS_COMPILE_EXAMPLES "Compile trajectory_utils examples" FALSE)
option(TRAJECTORY_UTILS_COMPILE_TESTS "Compile trajectory_utils tests" FALSE)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

INCLUDE_DIRECTORIES(include ${orocos_kdl_DIRS} ${kdl_parser_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS}
                            ${roslib_INCLUDE_DIRS} ${tf_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS}
                            ${rviz_visual_tools_INCLUDE_DIRS})

file(GLOB_RECURSE idynutils_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_utils" *.h*
"${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_utils/paths" *.h*
"${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_utils/utils" *.h*)

set(TRAJECTORY_UTILS_SOURCES src/trajectory_utils.cpp
                             src/paths/path_circle_fix.cpp)
if(${tf_FOUND} AND ${nav_msgs_FOUND} AND ${roslib_FOUND} AND ${rviz_visual_tools_FOUND})
    set(TRAJECTORY_UTILS_SOURCES ${TRAJECTORY_UTILS_SOURCES}
        src/utils/ros_trj_publisher.cpp)
endif()

ADD_LIBRARY(trajectory_utils SHARED ${TRAJECTORY_UTILS_SOURCES}
${trajectory_utils_INCLUDES})
TARGET_LINK_LIBRARIES(trajectory_utils ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES}
${Boost_LIBRARIES} ${rviz_visual_tools_LIBRARIES})

if(${TRAJECTORY_UTILS_COMPILE_EXAMPLES})
    if(${tf_FOUND} AND ${nav_msgs_FOUND} AND ${roslib_FOUND} AND ${rviz_visual_tools_FOUND})
        ADD_EXECUTABLE(exampleTrjPublisher example/exampleTrjPublisher.cpp)
        TARGET_LINK_LIBRARIES(exampleTrjPublisher trajectory_utils ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES}
        ${Boost_LIBRARIES} ${rviz_visual_tools_LIBRARIES})
    endif()
endif()

########################################################################
# use YCM to export idynutils so that it can be found using find_package #
########################################################################

set(VARS_PREFIX "TRAJECTORY_UTILS")

set(TRAJECTORY_UTILS_MAJOR_VERSION 0)
set(TRAJECTORY_UTILS_MINOR_VERSION 0)
set(TRAJECTORY_UTILS_PATCH_VERSION 1)
set(TRAJECTORY_UTILS_VERSION ${TRAJECTORY_UTILS_MAJOR_VERSION}.${TRAJECTORY_UTILS_MINOR_VERSION}.${TRAJECTORY_UTILS_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(trajectory_utils PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                            "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(trajectory_utils PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                SOVERSION ${${VARS_PREFIX}_VERSION})

target_link_libraries(trajectory_utils LINK_PUBLIC )

install(DIRECTORY include/ 
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS trajectory_utils  
        EXPORT trajectory_utils
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)
        
#enabling it will add all idynutils dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS trajectory_utils)


include(InstallBasicPackageFiles)
install_basic_package_files(trajectory_utils   VARS_PREFIX ${VARS_PREFIX}
                                        VERSION ${${VARS_PREFIX}_VERSION}
                                        COMPATIBILITY SameMajorVersion
                                        TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

#######################
# Add Testing target  #
#######################
if(${TRAJECTORY_UTILS_COMPILE_TESTS})
    enable_testing()
    add_subdirectory(tests)
endif()
