# Copyright: (C) 2020 RobotCub Consortium
# Authors: Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname egocentricAudioCropper)


file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.xml)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})

yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
