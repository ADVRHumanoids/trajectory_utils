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

find_package(srdfdom_advr)
find_package(iDynTree)
find_package(OpenSoT QUIET)
find_package(test_utils)

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} )
enable_testing()

if(${OpenSoT_FOUND})
    add_custom_command( TARGET trajectory_utils POST_BUILD
               COMMAND ${CMAKE_CTEST_COMMAND}
               MAIN_DEPENDENCY trajectory_utils
               DEPENDS
               testLinearTrj
               testArcTrj
               testLinearPlusArcTrj
               testSplineVelProfile
               testTrjDesigOpenSoT)
else()
    add_custom_command( TARGET trajectory_utils POST_BUILD
               COMMAND ${CMAKE_CTEST_COMMAND}
               MAIN_DEPENDENCY trajectory_utils
               DEPENDS
               testLinearTrj
               testArcTrj
               testLinearPlusArcTrj
               testSplineVelProfile)
endif()



SET(TestLibs trajectory_utils ${GTEST_BOTH_LIBRARIES}
    ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES}
    ${OpenSoT_LIBRARIES} ${test_utils_LIBRARIES})

ADD_EXECUTABLE(testLinearTrj     testLinearTrj.cpp)
TARGET_LINK_LIBRARIES(testLinearTrj ${TestLibs})
add_dependencies(testLinearTrj GTest-ext trajectory_utils)

ADD_EXECUTABLE(testArcTrj     testArcTrj.cpp)
TARGET_LINK_LIBRARIES(testArcTrj ${TestLibs})
add_dependencies(testArcTrj GTest-ext trajectory_utils)

ADD_EXECUTABLE(testLinearPlusArcTrj     testLinearPlusArcTrj.cpp)
TARGET_LINK_LIBRARIES(testLinearPlusArcTrj ${TestLibs})
add_dependencies(testLinearPlusArcTrj GTest-ext testLinearPlusArcTrj)

ADD_EXECUTABLE(testSplineVelProfile     testSplineVelProfile.cpp)
TARGET_LINK_LIBRARIES(testSplineVelProfile ${TestLibs})
add_dependencies(testSplineVelProfile GTest-ext testSplineVelProfile)

if(${OpenSoT_FOUND})
ADD_EXECUTABLE(testTrjDesigOpenSoT     testTrjDesigOpenSoT.cpp)
TARGET_LINK_LIBRARIES(testTrjDesigOpenSoT ${TestLibs})
add_dependencies(testTrjDesigOpenSoT GTest-ext testTrjDesigOpenSoT)
endif()

add_test(NAME trajectory_utils_linear_trajectories COMMAND testLinearTrj)
add_test(NAME trajectory_utils_arc_trajectories COMMAND testArcTrj)
add_test(NAME trajectory_utils_linear_plus_arc_trajectories COMMAND testLinearPlusArcTrj)
add_test(NAME trajectory_utils_spline_vel_profile COMMAND testSplineVelProfile)
if(${OpenSoT_FOUND})
add_test(NAME trajectory_utils_trj_desig_opensot COMMAND testTrjDesigOpenSoT)
endif()
