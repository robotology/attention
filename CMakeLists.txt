cmake_minimum_required(VERSION 2.6)
PROJECT (logpolarAttention)
# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.


#################################

find_package(ICUB)
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

include(iCubOptions)
include(iCubHelpers)

# default prefix for applications
SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

### Go through single folder
add_subdirectory(app)
add_subdirectory(src)

