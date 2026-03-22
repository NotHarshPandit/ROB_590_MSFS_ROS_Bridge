# cmake_minimum_required(VERSION 3.8)
# project(msfs_sensor_node)

# # Use C++17
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

# # --- SimConnect SDK paths (adjust if different) ---
# set(SIMCONNECT_SDK_ROOT "C:/MSFS 2024 SDK/SimConnect SDK" CACHE PATH "SimConnect SDK root")

# set(SIMCONNECT_INCLUDE_DIR "${SIMCONNECT_SDK_ROOT}/include")
# set(SIMCONNECT_LIB_DIR     "${SIMCONNECT_SDK_ROOT}/lib")

# if(NOT EXISTS "${SIMCONNECT_INCLUDE_DIR}/SimConnect.h")
#   message(FATAL_ERROR "SimConnect.h not found in ${SIMCONNECT_INCLUDE_DIR}")
# endif()

# if(NOT EXISTS "${SIMCONNECT_LIB_DIR}/SimConnect.lib")
#   message(FATAL_ERROR "SimConnect.lib not found in ${SIMCONNECT_LIB_DIR}")
# endif()

# include_directories(
#   include
#   ${SIMCONNECT_INCLUDE_DIR}
# )

# link_directories(${SIMCONNECT_LIB_DIR})

# add_executable(msfs_sensor_node src/msfs_sensor_node.cpp)
# add_executable(msfs_control_node src/msfs_control_node.cpp)


# ament_target_dependencies(msfs_sensor_node
#   rclcpp
#   std_msgs
# )
# ament_target_dependencies(msfs_control_node
#   rclcpp
#   std_msgs
#   geometry_msgs
# )

# target_link_libraries(msfs_control_node
#   SimConnect  # path to SimConnect.lib in your toolchain
# )
# target_link_libraries(msfs_sensor_node
#   SimConnect
# )

# install(TARGETS
#   msfs_sensor_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS
#   msfs_control_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()
cmake_minimum_required(VERSION 3.8)
project(msfs_sensor_node)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- SimConnect SDK paths (adjust if needed) ---
set(SIMCONNECT_SDK_ROOT "C:/MSFS 2024 SDK/SimConnect SDK" CACHE PATH "SimConnect SDK root")
set(SIMCONNECT_INCLUDE_DIR "${SIMCONNECT_SDK_ROOT}/include")
set(SIMCONNECT_LIB_DIR     "${SIMCONNECT_SDK_ROOT}/lib")

if(NOT EXISTS "${SIMCONNECT_INCLUDE_DIR}/SimConnect.h")
  message(FATAL_ERROR "SimConnect.h not found in ${SIMCONNECT_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${SIMCONNECT_LIB_DIR}/SimConnect.lib")
  message(FATAL_ERROR "SimConnect.lib not found in ${SIMCONNECT_LIB_DIR}")
endif()

include_directories(
  include
  ${SIMCONNECT_INCLUDE_DIR}
)

link_directories(${SIMCONNECT_LIB_DIR})

# --- Sensor node (publishes readings as String) ---
add_executable(msfs_sensor_node src/msfs_sensor_node.cpp)

ament_target_dependencies(msfs_sensor_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(msfs_sensor_node
  SimConnect
)

# --- Control node (subscribes to control topics) ---
add_executable(msfs_control_node src/msfs_control_node.cpp)

ament_target_dependencies(msfs_control_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(msfs_control_node
  SimConnect
)

install(TARGETS
  msfs_sensor_node
  msfs_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()