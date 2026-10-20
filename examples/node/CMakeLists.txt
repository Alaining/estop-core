cmake_minimum_required(VERSION 3.22)

# IDF names a local component after its directory, and the repo's is estop-core: a link
# named estop_core lets the build find it as a firmware would (managed_components/estop_core).
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/components)
file(CREATE_LINK ${CMAKE_CURRENT_LIST_DIR}/../.. ${CMAKE_BINARY_DIR}/components/estop_core SYMBOLIC)
set(EXTRA_COMPONENT_DIRS
  ${CMAKE_BINARY_DIR}/components/estop_core
  $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(estop_node_example)
