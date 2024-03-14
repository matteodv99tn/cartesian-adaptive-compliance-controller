cmake_minimum_required(VERSION 3.14)
project(cartesian_adaptive_compliance_controller)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ENABLE_LOGGING ON CACHE BOOL "Enable logging")

set(CMAKE_CXX_STANDARD 17)
set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_definitions(-DEIGEN_MPL2_ONLY)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cartesian_controller_base REQUIRED)
find_package(cartesian_motion_controller REQUIRED)
find_package(cartesian_force_controller REQUIRED)
find_package(cartesian_compliance_controller REQUIRED)


# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        cartesian_controller_base
        cartesian_motion_controller
        cartesian_force_controller
        cartesian_compliance_controller
        Eigen3
)

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Set the path to the qpOASES library and headers
set(EXTERNALS_DIR ${CMAKE_CURRENT_LIST_DIR}/external)
set(QPOASES_LIB_DIR ${EXTERNALS_DIR}/lib)
set(QPOASES_INCLUDE_DIR ${EXTERNALS_DIR}/include)

# Add the qpOASES library to the project
add_library(qpOASES STATIC IMPORTED)
set_target_properties(qpOASES PROPERTIES IMPORTED_LOCATION ${QPOASES_LIB_DIR}/libqpOASES.so)
target_include_directories(qpOASES INTERFACE ${QPOASES_INCLUDE_DIR})

find_package(fmt)

#--------------------------------------------------------------------------------
# Libraries
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/cartesian_adaptive_compliance_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} qpOASES)

ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

if(ENABLE_LOGGING AND ${fmt_FOUND})
    target_link_libraries(${PROJECT_NAME} fmt::fmt)
    target_compile_definitions(${PROJECT_NAME} PUBLIC LOGGING)
endif()

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#--------------------------------------------------------------------------------
# Install and export
#--------------------------------------------------------------------------------

pluginlib_export_plugin_description_file(controller_interface cartesian_adaptive_compliance_controller_plugin.xml)

# Note: The workflow as described here https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#building-a-library
# does not work for me.
# I'm not sure what's the problem right now, but I need to out-comment the
# lines below to achieve a correct symlink-install.

#ament_export_targets(my_targets_from_this_package HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  #EXPORT my_targets_from_this_package
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  #INCLUDES DESTINATION include
)

# Note: For the target based workflow, they seem to be superfluous.
# But since that doesn't work yet, I'll add them just in case.
# I took the joint_trajectory_controller as inspiration.
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
