# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /usr/local/src/robot/attention/src/dictationcontroller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /usr/local/src/robot/attention/src/dictationcontroller/build

# Include any dependencies generated for this target.
include CMakeFiles/dictationcontroller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dictationcontroller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dictationcontroller.dir/flags.make

CMakeFiles/dictationcontroller.dir/main.cpp.o: CMakeFiles/dictationcontroller.dir/flags.make
CMakeFiles/dictationcontroller.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /usr/local/src/robot/attention/src/dictationcontroller/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dictationcontroller.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dictationcontroller.dir/main.cpp.o -c /usr/local/src/robot/attention/src/dictationcontroller/main.cpp

CMakeFiles/dictationcontroller.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dictationcontroller.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /usr/local/src/robot/attention/src/dictationcontroller/main.cpp > CMakeFiles/dictationcontroller.dir/main.cpp.i

CMakeFiles/dictationcontroller.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dictationcontroller.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /usr/local/src/robot/attention/src/dictationcontroller/main.cpp -o CMakeFiles/dictationcontroller.dir/main.cpp.s

CMakeFiles/dictationcontroller.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/dictationcontroller.dir/main.cpp.o.requires

CMakeFiles/dictationcontroller.dir/main.cpp.o.provides: CMakeFiles/dictationcontroller.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/dictationcontroller.dir/build.make CMakeFiles/dictationcontroller.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/dictationcontroller.dir/main.cpp.o.provides

CMakeFiles/dictationcontroller.dir/main.cpp.o.provides.build: CMakeFiles/dictationcontroller.dir/main.cpp.o

# Object files for target dictationcontroller
dictationcontroller_OBJECTS = \
"CMakeFiles/dictationcontroller.dir/main.cpp.o"

# External object files for target dictationcontroller
dictationcontroller_EXTERNAL_OBJECTS =

dictationcontroller: CMakeFiles/dictationcontroller.dir/main.cpp.o
dictationcontroller: CMakeFiles/dictationcontroller.dir/build.make
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/libicubmod.a
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_OS.so
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_sig.so
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_math.so
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_dev.so
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_name.so
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_init.so
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_videostab.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_video.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_ts.a
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_superres.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_stitching.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_photo.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_ocl.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_objdetect.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_nonfree.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_ml.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_legacy.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_imgproc.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_highgui.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_gpu.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_flann.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_features2d.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_core.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_contrib.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_calib3d.so.2.4.8
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/libcartesiancontrollerserver.a
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/libcartesiancontrollerclient.a
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/libiKin.a
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/libctrlLib.a
dictationcontroller: /usr/lib/libgsl.so
dictationcontroller: /usr/lib/libgslcblas.so
dictationcontroller: /usr/local/src/robot/ipopt/Ipopt-3.10.4/build/lib/libipopt.so
dictationcontroller: /usr/local/src/robot/ipopt/Ipopt-3.10.4/build/lib/libcoinmumps.so
dictationcontroller: /usr/local/src/robot/ipopt/Ipopt-3.10.4/build/lib/libcoinlapack.so
dictationcontroller: /usr/local/src/robot/ipopt/Ipopt-3.10.4/build/lib/libcoinmetis.so
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/libgazecontrollerclient.a
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/liblogpolarclient.a
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/libiCubDev.a
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/liblogpolargrabber.a
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/liblogpolar.a
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/libdebugInterfaceClient.a
dictationcontroller: /usr/local/src/robot/icub-main/build-x86_64/lib/libdebugInterfaceWrapper.a
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_math.so
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_dev.so
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_sig.so
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_name.so
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_init.so
dictationcontroller: /usr/local/src/robot/yarp/build-x86_64/lib/libYARP_OS.so
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_nonfree.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_ocl.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_gpu.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_photo.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_objdetect.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_legacy.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_video.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_ml.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_calib3d.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_features2d.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_highgui.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_imgproc.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_flann.so.2.4.8
dictationcontroller: /usr/local/src/robot/opencv-2.4.8/build/lib/libopencv_core.so.2.4.8
dictationcontroller: CMakeFiles/dictationcontroller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable dictationcontroller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dictationcontroller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dictationcontroller.dir/build: dictationcontroller
.PHONY : CMakeFiles/dictationcontroller.dir/build

CMakeFiles/dictationcontroller.dir/requires: CMakeFiles/dictationcontroller.dir/main.cpp.o.requires
.PHONY : CMakeFiles/dictationcontroller.dir/requires

CMakeFiles/dictationcontroller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dictationcontroller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dictationcontroller.dir/clean

CMakeFiles/dictationcontroller.dir/depend:
	cd /usr/local/src/robot/attention/src/dictationcontroller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/local/src/robot/attention/src/dictationcontroller /usr/local/src/robot/attention/src/dictationcontroller /usr/local/src/robot/attention/src/dictationcontroller/build /usr/local/src/robot/attention/src/dictationcontroller/build /usr/local/src/robot/attention/src/dictationcontroller/build/CMakeFiles/dictationcontroller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dictationcontroller.dir/depend

