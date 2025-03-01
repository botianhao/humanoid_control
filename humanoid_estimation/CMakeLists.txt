cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(humanoid_estimation)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        rclcpp
        hardware_interface
        tf2_ros
        nav_msgs
        humanoid_interface
        geometry_msgs
        tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(humanoid_interface REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

###########
## Build ##
###########

# Resolve for the package path at compile time.


set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

# Legged robot interface library
add_library(${PROJECT_NAME}
        src/StateEstimateBase.cpp
        src/FromTopicEstimate.cpp
        src/LinearKalmanFilter.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

#########################
###   CLANG TOOLING   ###
#########################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_legged_robot")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS src include test
            CT_HEADER_DIRS include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY 
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############


ament_package()

