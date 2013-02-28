

cmake_minimum_required(VERSION 2.8)

project(cloudsim-client-tools)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
set(MAJOR_VERSION 0)
set(MINOR_VERSION 9)
set(PATCH_VERSION 4)

set(VERSION ${MAJOR_VERSION}.${MINOR_VERSION})
set(VERSION_FULL ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

message(STATUS "${PROJECT_NAME} version ${VERSION_FULL}")

add_executable(hello hello.cc)
install(TARGETS hello RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(PROGRAMS tc/init_tc.py tc/configure_tc.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# Install Traffic shaper utilities:
# ts_sniffer: Sniffer to measure latency between router and COU
# ts_controller: Open loop controller to adjust latency to a target values  
# pid: Module that contains a PID template that can be used by the controller
install(PROGRAMS ts/ts_sniffer.py ts/ts_controller.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(PROGRAMS ts/cloudsim/pid.py ts/cloudsim/__init__.py DESTINATION /usr/lib/python2.7/cloudsim)
install(PROGRAMS ts/conf/ts_sniffer_outbound.conf ts/conf/ts_controller_outbound.conf ts/conf/ts_controller_inbound.conf  DESTINATION /etc/init)

# Install VRC bandwidth utilities:
# VRC_bandwidth: Count and logging bandwidth when a specific start/stop ros topic is received
install(PROGRAMS vrc_bandwidth/vrc_bandwidth.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(PROGRAMS vrc_bandwidth/conf/vrc_bandwidth.conf DESTINATION /etc/init)

########################################
# Package Creation: 
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Infrastructure for launching robotics simulations in the cloud")
set(CPACK_PACKAGE_VENDOR "Open Source Robotics Foundation, Inc.")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR ${MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${PATCH_VERSION})
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME_LOWER}-${VERSION_FULL}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "${PROJECT_NAME_LOWER}")
set(CPACK_SOURCE_GENERATOR "TBZ2")
list(APPEND CPACK_SOURCE_IGNORE_FILES ";Ogre.log;~$;TODO;/.hg/;.swp$;/build/;.hgtags;.hgignore")
include(CPack)

