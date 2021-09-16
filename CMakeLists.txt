cmake_minimum_required(VERSION 3.0.2)
project(choreonoid_bridge_shm)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  trans_vm)
find_package(choreonoid REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${trans_vm_SOURCE_PREFIX}/include
  )

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)


# add_cnoid_simple_controllerを使いたいが、パスがうまく解決できない
add_library(BridgeShmController SHARED src/BridgeShmController.cpp)

target_link_libraries(BridgeShmController
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  )

set_target_properties(BridgeShmController PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR}/simplecontroller
  LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR}/simplecontroller
  PREFIX "")

install(TARGETS BridgeShmController
  RUNTIME DESTINATION $${CHOREONOID_PLUGIN_DIR}/simplecontroller CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
  LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR}/simplecontroller CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
