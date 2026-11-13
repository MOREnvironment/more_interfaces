cmake_minimum_required(VERSION 3.8...3.31)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

set(msg_files
  "msg/AbortEvent.msg"
  "msg/AcousticLinkEvent.msg"
  "msg/AllocatorResponse.msg"
  "msg/BridgeLinkStatus.msg"
  "msg/CommandAuth.msg"
  "msg/CommandRefused.msg"
  "msg/CommandResult.msg"
  "msg/ConsumerLatency.msg"
  "msg/FinsResponse.msg"
  "msg/FleetRegistry.msg"
  "msg/FleetSystem.msg"
  "msg/IMU.msg"
  "msg/Limits.msg"
  "msg/Maneuver.msg"
  "msg/PartAction.msg"
  "msg/PartState.msg"
  "msg/PartStateArray.msg"
  "msg/PayloadState.msg"
  "msg/PlanList.msg"
  "msg/PlanState.msg"
  "msg/PlanSummary.msg"
  "msg/PlanTransition.msg"
  "msg/QuaternionNed.msg"
  "msg/RangeMeasurement.msg"
  "msg/RangeInterval.msg"
  "msg/RuntimeStats.msg"
  "msg/Setting.msg"
  "msg/SonarInfo.msg"
  "msg/SwathFootprint.msg"
  "msg/SyncAck.msg"
  "msg/TextMessage.msg"
  "msg/ThrusterResponse.msg"
  "msg/TxStatus.msg"
  "msg/UsblFix.msg"
  "msg/VehicleMode.msg"
  "msg/WaterColumn.msg"
)

set(srv_files
  "srv/Abort.srv"
  "srv/QueryPlan.srv"
  "srv/SendText.srv"
  "srv/SetCourseSpeed.srv"
  "srv/SetLeg.srv"
  "srv/SetPayload.srv"
  "srv/SetStation.srv"
  "srv/SetStationRadii.srv"
  "srv/StartPlan.srv"
  "srv/StopPlan.srv"
  "srv/StorePlan.srv"
)

set(action_files
  "action/CalibrateIMU.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    sensor_msgs
    geographic_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_interfaces test/test_interfaces.py)
endif()

ament_export_dependencies(
  rosidl_default_runtime
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
  geographic_msgs
)

ament_package()
