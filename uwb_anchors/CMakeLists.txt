cmake_minimum_required(VERSION 3.0.2)
project(uwb_anchors)

# Поиск зависимостей
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
)

find_package(gazebo REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Создание плагина
add_library(gazebo_ros_uwb_anchor SHARED
  src/gazebo_ros_uwb_anchor.cpp
)

# Линковка
target_link_libraries(gazebo_ros_uwb_anchor
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Установка
install(TARGETS gazebo_ros_uwb_anchor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)