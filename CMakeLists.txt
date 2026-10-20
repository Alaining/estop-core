# estop-core: an ESP-IDF component under idf.py, a host test build anywhere else.

if(ESP_PLATFORM)
  idf_component_register(
    SRCS "src/core/node_logic.cpp" "src/core/latch_logic.cpp"
         "src/esp/edge_input.cpp" "src/esp/node.cpp" "src/esp/latch.cpp"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src/esp"
    REQUIRES cdr rammp_rtps_messages esp_driver_gpio freertos
    PRIV_REQUIRES rtps esp_timer log)
  return()
endif()

# Host build: the pure core and its tests, with ASan/UBSan.
#   cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure
# Local checkouts instead of fetching: -DFETCHCONTENT_SOURCE_DIR_RAMMP_RTPS=<path>,
# -DFETCHCONTENT_SOURCE_DIR_CDR=<path>.
cmake_minimum_required(VERSION 3.24)
project(estop_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
# The versions pace-hmi-fw builds with: rammp-rtps dev-estop, and the cdr that espp 1.3.2
# vendors (components/cdr/detail/cdr).
FetchContent_Declare(rammp_rtps
  GIT_REPOSITORY https://github.com/rammp-org/rammp-rtps.git
  GIT_TAG 3390723cbad6f7a00bd7d5f0ca65fe69635e2122
  SOURCE_SUBDIR do-not-configure)
FetchContent_Declare(cdr
  GIT_REPOSITORY https://github.com/finger563/cdr.git
  GIT_TAG 15d5aac857997024bc743f5b785cf9c4567f5ebc)
set(CDR_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(rammp_rtps cdr)

add_library(estop_core STATIC src/core/node_logic.cpp src/core/latch_logic.cpp)
target_include_directories(estop_core PUBLIC include
  ${rammp_rtps_SOURCE_DIR}/components/rammp_rtps_messages/include)
target_link_libraries(estop_core PUBLIC cdr::cdr)

set(ESTOP_SANITIZE -fsanitize=address,undefined -fno-sanitize-recover=all)
target_compile_options(estop_core PRIVATE -Wall -Wextra -Werror ${ESTOP_SANITIZE})

enable_testing()
foreach(name codec node latch race)
  add_executable(test_${name} test/test_${name}.cpp)
  target_link_libraries(test_${name} PRIVATE estop_core)
  target_compile_options(test_${name} PRIVATE -Wall -Wextra -Werror -g ${ESTOP_SANITIZE})
  target_link_options(test_${name} PRIVATE ${ESTOP_SANITIZE})
  add_test(NAME ${name} COMMAND test_${name})
endforeach()
