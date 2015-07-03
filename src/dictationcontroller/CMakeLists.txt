CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(dictationcontroller)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(YARP)
find_package(ICUB)
find_package(OpenCV)

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    )
add_executable(dictationcontroller main.cpp)

target_link_libraries(dictationcontroller icubmod ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS dictationcontroller RUNTIME DESTINATION bin)
