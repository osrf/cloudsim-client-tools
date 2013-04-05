cmake_minimum_required(VERSION 2.6)
project(DarpaPacketCounter)


add_executable(vrc_bitcounter bitCounter.c)
target_link_libraries(vrc_bitcounter ${LIBS})