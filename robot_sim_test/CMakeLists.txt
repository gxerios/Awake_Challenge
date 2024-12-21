cmake_minimum_required(VERSION 3.8)
project(robot_sim_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ign_ros2_control REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY world DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY xacro DESTINATION share/${PROJECT_NAME}/)

# Installe les fichiers .yaml, .dae, etc.
install(
    DIRECTORY maps/  # Remplacez 'maps/' par le dossier contenant vos fichiers
    DESTINATION share/${PROJECT_NAME}/maps
    USE_SOURCE_PERMISSIONS
)

# Installe les fichiers de lancement
install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
