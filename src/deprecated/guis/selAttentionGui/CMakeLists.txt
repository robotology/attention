# Copyright (C) 2014  iCub Facility, Istituto Italiano di Tecnologia
# Author: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

#include(iCubUseQt5)


set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

include_directories(${YARP_INCLUDE_DIRS}
                    ${attentionDynLib_INCLUDE_DIRS})

set(attentionManagerGui_qt_SRCS main.cpp
                           loadingwidget.cpp
                           mainwindow.cpp
                           portthread.cpp
                           qcustomplot.cpp)

set(attentionManagerGui_qt_HDRS loadingwidget.h
                           mainwindow.h
                           portthread.h
                           qcustomplot.h)

set(attentionManagerGui_qt_QRC_FILES res.qrc)

set(attentionManagerGui_qt_UI_FILES loadingwidget.ui
                               mainwindow.ui)

qt5_add_resources(attentionManagerGui_qt_QRC_GEN_SRCS ${attentionManagerGui_qt_QRC_FILES})
qt5_wrap_ui(attentionManagerGui_qt_UI_GEN_SRCS ${attentionManagerGui_qt_UI_FILES})

source_group("Source Files" FILES ${attentionManagerGui_qt_SRCS})
source_group("Header Files" FILES ${attentionManagerGui_qt_HDRS})
source_group("Resources Files" FILES ${attentionManagerGui_qt_QRC_FILES})
source_group("UI Files" FILES ${attentionManagerGui_qt_UI_FILES})
source_group("Generated Files" FILES ${attentionManagerGui_qt_QRC_GEN_SRCS}
                                     ${attentionManagerGui_qt_UI_GEN_SRCS})

add_executable(attentionManagerGui-qt WIN32 ${attentionManagerGui_qt_SRCS}
                                       ${attentionManagerGui_qt_HDRS}
                                       ${attentionManagerGui_qt_QRC_GEN_SRCS}
                                       ${attentionManagerGui_qt_UI_GEN_SRCS})
target_link_libraries(attentionManagerGui-qt YARP::YARP_OS
                                        YARP::YARP_init
                                        YARP::YARP_sig
                                        )

qticub_use_modules(attentionManagerGui-qt Widgets
                                     PrintSupport)
qticub_deprecate_with_cmake_version(2.8.11)
if(WIN32 AND CMAKE_VERSION VERSION_LESS 2.8.11)
  target_link_libraries(attentionManagerGui-qt Qt5::WinMain)
endif()

install(TARGETS attentionManagerGui-qt COMPONENT utilities DESTINATION ${CMAKE_INSTALL_BINDIR})

if(NOT YARP_DEFAULT_GTK)
  set_target_properties(attentionManagerGui-qt PROPERTIES OUTPUT_NAME attentionManagerGui)
endif()
