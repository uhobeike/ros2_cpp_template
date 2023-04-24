cmake_minimum_required(VERSION 3.8)
project(ros2_cpp_template)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# template_composition
ament_auto_add_library(template_composition SHARED
  src/composition/template_composition.cpp
)

rclcpp_components_register_node(template_composition
  PLUGIN "Ros2CppTemplate::TemplateComposition"
  EXECUTABLE ${PROJECT_NAME}
)

# template_node
ament_auto_add_executable(template_node
  src/node/template_core.cpp
  src/node/template_node.cpp
)

# template_executor
ament_auto_add_executable(template_executor
  src/node/template_core.cpp
  src/executor/template_executor.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
