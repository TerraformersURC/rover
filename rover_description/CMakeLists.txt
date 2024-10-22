cmake_minimum_required(VERSION 3.8)
project(terryjr_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
