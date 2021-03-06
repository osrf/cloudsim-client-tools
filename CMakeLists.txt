cmake_minimum_required(VERSION 2.8)

project(cloudsim-client-tools)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
set(MAJOR_VERSION 0)
set(MINOR_VERSION 11)
set(PATCH_VERSION 5)

set(VERSION ${MAJOR_VERSION}.${MINOR_VERSION})
set(VERSION_FULL ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

message(STATUS "${PROJECT_NAME} version ${VERSION_FULL}")

# Install vrc_bytecounter for counting network usage over the network
add_subdirectory(vrc_bytecounter)

# Install vrc_wrapper.sh that allows to source conf files and run jobs
install(PROGRAMS vrc_wrapper/vrc_wrapper.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# Install Traffic shaper utilities:
# vrc_init_tc: Initialize tc commands for shaping
# vrc_configure_tc: Injects extra latency commands for shaping
# vrc_sniffer: Sniffer to measure latency between router and COU
# vrc_controller: Open loop controller to adjust latency to a target values  
install(PROGRAMS vrc_shaping/tc/vrc_init_tc.py vrc_shaping/tc/vrc_configure_tc.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(PROGRAMS vrc_shaping/tc/init_tc.py vrc_shaping/tc/configure_tc.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(PROGRAMS vrc_shaping/vrc_sniffer.py vrc_shaping/vrc_controller.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# Install VRC bandwidth utilities:
# VRC_netwatcher: Count and logging bandwidth when a specific start/stop ros topic is received
install(PROGRAMS vrc_netwatcher/vrc_netwatcher.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

install(PROGRAMS vrc_monitor/vrc_monitor.py DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(PROGRAMS vrc_monitor/vrc_monitor.conf DESTINATION /etc/init)

# Helper script to locally cache gazebo_models and to check out vrc_arenas
# (needs appropriate ssh key)
install(PROGRAMS tools/set_vrc_private.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

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

