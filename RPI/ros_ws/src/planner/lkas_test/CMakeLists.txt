cmake_minimum_required(VERSION 3.8)
project(lkas_test)

set(TENSORFLOW_SOURCE_DIR /root/tensorflow-2.15.1)
option(TFLITE_ENABLE_INSTALL "Enable install rule" OFF)
option(TFLITE_ENABLE_LABEL_IMAGE "Enable label_image example" OFF)
option(TFLITE_ENABLE_BENCHMARK_MODEL "Enable the benchmark_model tool" OFF)
option(TFLITE_ENABLE_XNNPACK "Enable XNNPACK" ON) # <-- 성능을 위해 켜는 것을 권장
option(TFLITE_ENABLE_RUY "Enable RUY" ON)

add_subdirectory(
  "${TENSORFLOW_SOURCE_DIR}/tensorflow/lite"
  "${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite" EXCLUDE_FROM_ALL
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ssv_interfaces REQUIRED)
find_package(planner_manager REQUIRED)
find_package(can_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(lkas_node src/lkas_test.cpp)
ament_target_dependencies(lkas_node rclcpp std_msgs sensor_msgs message_filters cv_bridge image_transport ssv_interfaces planner_manager)

add_executable(lkas_node2 src/lkas_test2.cpp)
ament_target_dependencies(lkas_node2 rclcpp std_msgs sensor_msgs message_filters cv_bridge image_transport ssv_interfaces planner_manager)

add_executable(tf2can_node src/tf2can_node.cpp)
ament_target_dependencies(tf2can_node rclcpp std_msgs sensor_msgs can_msgs)

# find_package로 찾은 TFLite 타겟(tensorflow-lite)을 링크합니다.
target_link_libraries(lkas_node
	${OpenCV_LIBRARIES}
	tensorflow-lite
)

target_link_libraries(lkas_node2
	${OpenCV_LIBRARIES}
	tensorflow-lite
)


install(TARGETS
	lkas_node
	lkas_node2
	tf2can_node
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME})

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
