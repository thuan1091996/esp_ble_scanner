# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Actor Framework ====================================================================================
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/actor-framework"
                         "${CMAKE_CURRENT_LIST_DIR}/actor-framework/osal"
                         "${CMAKE_CURRENT_LIST_DIR}/actor-framework/3rd-party/lwmem/lwmem" )
# ====================================================================================================

project(ble_scanner)
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)

