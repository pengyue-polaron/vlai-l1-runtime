cmake_minimum_required(VERSION 3.16)
project(vlai_l1_xair_sidecar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(VLAI_L1_SAFETY_ONLY "Build only the hardware-free runtime safety tests" OFF)

add_library(vlai_l1_runtime_safety STATIC runtime_safety.cpp)
target_compile_options(vlai_l1_runtime_safety PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(vlai_l1_runtime_safety PUBLIC dl pthread)

include(CTest)
if(BUILD_TESTING)
  add_executable(vlai_l1_runtime_safety_test runtime_safety_test.cpp)
  target_compile_options(vlai_l1_runtime_safety_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_link_libraries(vlai_l1_runtime_safety_test PRIVATE vlai_l1_runtime_safety)
  add_test(NAME vlai_l1_runtime_safety COMMAND vlai_l1_runtime_safety_test)
endif()

if(VLAI_L1_SAFETY_ONLY)
  return()
endif()

if(NOT DEFINED X_AIR_SDK_ROOT)
  message(FATAL_ERROR "X_AIR_SDK_ROOT must point to the pinned x_air SDK checkout")
endif()

set(XAIR_TELEOP_ROOT "${X_AIR_SDK_ROOT}/publish/modules/src/xarm_teleop")
set(XAIR_CAN_ROOT "${X_AIR_SDK_ROOT}/publish/xarm_can/package")
set(XAIR_TELEOP_LIB
    "${XAIR_TELEOP_ROOT}/prebuilt/xarm_teleop/lib/${CMAKE_SYSTEM_PROCESSOR}/libxarm_teleop_lib.so")
set(XAIR_CAN_LIB
    "${XAIR_CAN_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}/libxarm_can_sdk.so")

foreach(required_path IN ITEMS
    "${XAIR_TELEOP_ROOT}/include/xarm_teleop_sdk.h"
    "${XAIR_TELEOP_LIB}"
    "${XAIR_CAN_LIB}")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR "Required x_air SDK artifact is missing: ${required_path}")
  endif()
endforeach()

add_library(xair_teleop SHARED IMPORTED)
set_target_properties(xair_teleop PROPERTIES IMPORTED_LOCATION "${XAIR_TELEOP_LIB}")
add_library(xair_can SHARED IMPORTED)
set_target_properties(xair_can PROPERTIES IMPORTED_LOCATION "${XAIR_CAN_LIB}")

# The published library declares SONAME libxarm_can_sdk.so.1 but its
# architecture directory contains only libxarm_can_sdk.so. Keep the pinned
# checkout immutable and satisfy the loader from this disposable build tree.
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E create_symlink
          "${XAIR_CAN_LIB}" "${CMAKE_CURRENT_BINARY_DIR}/libxarm_can_sdk.so.1"
  RESULT_VARIABLE XAIR_CAN_LINK_RESULT
)
if(NOT XAIR_CAN_LINK_RESULT EQUAL 0)
  message(FATAL_ERROR "Could not stage libxarm_can_sdk.so.1 in the build tree")
endif()

add_executable(vlai_l1_xair_sidecar main.cpp)
target_compile_options(vlai_l1_xair_sidecar PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(vlai_l1_xair_sidecar PRIVATE "${XAIR_TELEOP_ROOT}/include")
target_link_options(vlai_l1_xair_sidecar PRIVATE -Wl,--no-as-needed)
target_link_libraries(vlai_l1_xair_sidecar PRIVATE vlai_l1_runtime_safety xair_teleop xair_can)
set_target_properties(vlai_l1_xair_sidecar PROPERTIES
  ENABLE_EXPORTS ON
  BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR};${XAIR_TELEOP_ROOT}/prebuilt/xarm_teleop/lib/${CMAKE_SYSTEM_PROCESSOR};${XAIR_CAN_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}"
)
