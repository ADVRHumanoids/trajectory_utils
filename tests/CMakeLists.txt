cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME tests)
project(${PROJECTNAME})

find_package(GTest)
if (NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} )
enable_testing()

add_custom_command( TARGET trajectory_utils POST_BUILD
               COMMAND ${CMAKE_CTEST_COMMAND}
               MAIN_DEPENDENCY trajectory_utils
               DEPENDS
               testLinearTrj
               testArcTrj
               testLinearPlusArcTrj
               )

SET(TestLibs trajectory_utils ${GTEST_BOTH_LIBRARIES} ${idynutils_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES})

ADD_EXECUTABLE(testLinearTrj     testLinearTrj.cpp)
TARGET_LINK_LIBRARIES(testLinearTrj ${TestLibs})
add_dependencies(testLinearTrj GTest-ext trajectory_utils)

ADD_EXECUTABLE(testArcTrj     testArcTrj.cpp)
TARGET_LINK_LIBRARIES(testArcTrj ${TestLibs})
add_dependencies(testArcTrj GTest-ext trajectory_utils)

ADD_EXECUTABLE(testLinearPlusArcTrj     testLinearPlusArcTrj.cpp)
TARGET_LINK_LIBRARIES(testLinearPlusArcTrj ${TestLibs})
add_dependencies(testLinearPlusArcTrj GTest-ext testLinearPlusArcTrj)

add_test(NAME trajectory_utils_linear_trajectories COMMAND testLinearTrj)
add_test(NAME trajectory_utils_arc_trajectories COMMAND testArcTrj)


