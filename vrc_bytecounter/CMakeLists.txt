cmake_minimum_required(VERSION 2.6)
project(vrc_bytecounter)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")


find_package(LibPCAP REQUIRED)
include_directories(${PCAP_INCLUDE_DIRS})
set(LIBS ${LIBS} ${PCAP_LIBRARIES} hiredis)

add_subdirectory(src)
