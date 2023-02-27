# ros2_cpp_template [![build-test](https://github.com/uhobeike/ros2_cpp_template/actions/workflows/build-test.yaml/badge.svg)](https://github.com/uhobeike/ros2_cpp_template/actions/workflows/build-test.yaml)

## Overview
ROS 2のC++パッケージのテンプレートです。

## Directory Configuration
```
.
├── CMakeLists.txt
├── LICENSE
├── README.md
├── config
│   └── template.param.yaml
├── include
│   └── ros2_cpp_template
│       ├── composition
│       │   └── template_composition.hpp
│       ├── executor
│       │   ├── template_composition.hpp
│       │   └── template_core.hpp
│       └── node
│           └── template_core.hpp
├── launch
│   ├── template_composition.launch.py
│   ├── template_executor.launch.py
│   └── template_node.launch.py
├── package.xml
└── src
    ├── composition
    │   └── template_composition.cpp
    ├── executor
    │   └── template_executor.cpp
    └── node
        ├── template_core.cpp
        └── template_node.cpp

5 directories, 8 files
```

## Build / Install

```
mkdir catkin_ws/src -p
git clone https://github.com/uhobeike/ros2_cpp_template.git catkin_ws/src/ros2_cpp_template
cd catkin_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

### Manual Composition

```
ros2 component standalone ros2_cpp_template Ros2CppTemplate::TemplateComposition -p printf_string:="ROS 3のテンプレートも欲しい：" -p printf_hz:=10.0
```

### Composition Using Launch Actions
```
ros2 launch ros2_cpp_template template_composition.launch.py
```

### Node Using Launch Actions
```
ros2 launch ros2_cpp_template template_node.launch.py
```

### Executor Using Launch Actions
```
ros2 launch ros2_cpp_template template_executor.launch.py
```

## [C++ code format](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html#contributors-guidelines-formatting)
```
ament_uncrustify --reformat src/ include/
```