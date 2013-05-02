cmake_minimum_required(VERSION 2.6)
project(vrc_bytecounter)

add_executable(vrc_bytecounter byteCounter.c)
target_link_libraries(vrc_bytecounter ${LIBS})
install(TARGETS vrc_bytecounter RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)