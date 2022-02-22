# Copyright: (C) 2018 RobotCub Consortium
# Authors: Gonzalez Jonas
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
SET(KEYWORD "ColorSegmentation")
set(appname ${KEYWORD})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.xml model colormap)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)


yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})

yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

