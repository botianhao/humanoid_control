cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(humanoid_wbc)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        humanoid_interface
        qpoases_catkin
)

find_package(ament_cmake REQUIRED)
find_package(humanoid_interface REQUIRED)
find_package(qpoases_catkin REQUIRED)


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
        src/HoQp.cpp
        src/WbcBase.cpp
        src/HierarchicalWbc.cpp
        src/WeightedWbc.cpp
)





target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        
        )

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
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest)

    ament_add_gtest(${PROJECT_NAME}_test
        test/HoQp_test.cpp
    )
    target_include_directories(${PROJECT_NAME}_test PRIVATE
        ${PROJECT_NAME}
        ${pinocchio_LIBRARIES}
        gtest_main
    )
    ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})
endif ()

ament_package()
