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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bigrig/Sentinel/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bigrig/Sentinel/build

# Include any dependencies generated for this target.
include image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/depend.make

# Include the progress variables for this target.
include image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/progress.make

# Include the compile flags for this target's objects.
include image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/flags.make

image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o: image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/flags.make
image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o: /home/bigrig/Sentinel/src/image_pipeline/image_rotate/src/node/image_rotate.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bigrig/Sentinel/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o"
	cd /home/bigrig/Sentinel/build/image_pipeline/image_rotate && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o -c /home/bigrig/Sentinel/src/image_pipeline/image_rotate/src/node/image_rotate.cpp

image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.i"
	cd /home/bigrig/Sentinel/build/image_pipeline/image_rotate && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bigrig/Sentinel/src/image_pipeline/image_rotate/src/node/image_rotate.cpp > CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.i

image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.s"
	cd /home/bigrig/Sentinel/build/image_pipeline/image_rotate && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bigrig/Sentinel/src/image_pipeline/image_rotate/src/node/image_rotate.cpp -o CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.s

image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o.requires:
.PHONY : image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o.requires

image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o.provides: image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o.requires
	$(MAKE) -f image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/build.make image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o.provides.build
.PHONY : image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o.provides

image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o.provides.build: image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o

# Object files for target image_rotate_exe
image_rotate_exe_OBJECTS = \
"CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o"

# External object files for target image_rotate_exe
image_rotate_exe_EXTERNAL_OBJECTS =

/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/build.make
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libcv_bridge.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libdynamic_reconfigure_config_init_mutex.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libeigen_conversions.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libimage_transport.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libnodeletlib.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libbondcpp.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libclass_loader.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/libPocoFoundation.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libdl.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libroslib.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/liborocos-kdl.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/liborocos-kdl.so.1.3.0
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libtf2_ros.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libactionlib.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libmessage_filters.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libroscpp.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/librosconsole.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/librosconsole_log4cxx.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/librosconsole_backend_interface.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/liblog4cxx.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libxmlrpcpp.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libtf2.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libroscpp_serialization.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/librostime.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /opt/ros/jade/lib/libcpp_common.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate: image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate"
	cd /home/bigrig/Sentinel/build/image_pipeline/image_rotate && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/image_rotate_exe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/build: /home/bigrig/Sentinel/devel/lib/image_rotate/image_rotate
.PHONY : image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/build

image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/requires: image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/src/node/image_rotate.cpp.o.requires
.PHONY : image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/requires

image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/clean:
	cd /home/bigrig/Sentinel/build/image_pipeline/image_rotate && $(CMAKE_COMMAND) -P CMakeFiles/image_rotate_exe.dir/cmake_clean.cmake
.PHONY : image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/clean

image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/depend:
	cd /home/bigrig/Sentinel/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bigrig/Sentinel/src /home/bigrig/Sentinel/src/image_pipeline/image_rotate /home/bigrig/Sentinel/build /home/bigrig/Sentinel/build/image_pipeline/image_rotate /home/bigrig/Sentinel/build/image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : image_pipeline/image_rotate/CMakeFiles/image_rotate_exe.dir/depend

