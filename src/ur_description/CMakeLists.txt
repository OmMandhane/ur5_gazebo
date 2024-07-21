cmake_minimum_required(VERSION 3.8)
project(ur_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake and dependencies
find_package(ament_cmake REQUIRED)

# Install launch files, URDFs, and meshes
install(DIRECTORY launch meshes urdf config rviz test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
