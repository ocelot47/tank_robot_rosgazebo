cmake_minimum_required(VERSION 3.5)
project(four_wheeled_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(INSTALL_DIRS
  config
  meshes
  worlds
  maps
  description
  launch
  img
)

foreach(dir IN LISTS INSTALL_DIRS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(
      DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

set(PY_SCRIPTS
  scripts/wasd_teleop
  scripts/set_initial_pose_once
  scripts/custom_planner_server
)

foreach(script IN LISTS PY_SCRIPTS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${script}")
    install(
      PROGRAMS ${script}
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endforeach()

set(PY_MODULES
  scripts/custom_algorithms.py
  scripts/custom_neighbors.py
  scripts/custom_gridviz.py
)

foreach(module IN LISTS PY_MODULES)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${module}")
    install(
      FILES ${module}
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endforeach()

ament_package()
