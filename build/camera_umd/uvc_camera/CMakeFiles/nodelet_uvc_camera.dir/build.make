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
include camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/depend.make

# Include the progress variables for this target.
include camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/progress.make

# Include the compile flags for this target's objects.
include camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/flags.make

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/flags.make
camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o: /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/nodelets.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bigrig/Sentinel/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o -c /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/nodelets.cpp

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.i"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/nodelets.cpp > CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.i

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.s"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/nodelets.cpp -o CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.s

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o.requires:
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o.requires

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o.provides: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o.requires
	$(MAKE) -f camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/build.make camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o.provides.build
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o.provides

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o.provides.build: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/flags.make
camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o: /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/camera.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bigrig/Sentinel/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o -c /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/camera.cpp

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.i"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/camera.cpp > CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.i

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.s"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/camera.cpp -o CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.s

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o.requires:
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o.requires

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o.provides: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o.requires
	$(MAKE) -f camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/build.make camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o.provides.build
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o.provides

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o.provides.build: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/flags.make
camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o: /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/stereo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bigrig/Sentinel/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o -c /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/stereo.cpp

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.i"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/stereo.cpp > CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.i

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.s"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/stereo.cpp -o CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.s

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o.requires:
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o.requires

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o.provides: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o.requires
	$(MAKE) -f camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/build.make camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o.provides.build
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o.provides

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o.provides.build: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/flags.make
camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o: /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/uvc_cam.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bigrig/Sentinel/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o -c /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/uvc_cam.cpp

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.i"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/uvc_cam.cpp > CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.i

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.s"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bigrig/Sentinel/src/camera_umd/uvc_camera/src/uvc_cam.cpp -o CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.s

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o.requires:
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o.requires

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o.provides: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o.requires
	$(MAKE) -f camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/build.make camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o.provides.build
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o.provides

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o.provides.build: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o

# Object files for target nodelet_uvc_camera
nodelet_uvc_camera_OBJECTS = \
"CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o" \
"CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o" \
"CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o" \
"CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o"

# External object files for target nodelet_uvc_camera
nodelet_uvc_camera_EXTERNAL_OBJECTS =

/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/build.make
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libcamera_info_manager.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libimage_transport.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libmessage_filters.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libnodeletlib.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libbondcpp.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libclass_loader.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/libPocoFoundation.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libroslib.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libroscpp.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/librosconsole.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/librosconsole_log4cxx.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/librosconsole_backend_interface.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/liblog4cxx.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libxmlrpcpp.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libroscpp_serialization.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/librostime.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libcpp_common.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libcamera_info_manager.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libimage_transport.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libmessage_filters.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libnodeletlib.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libbondcpp.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libclass_loader.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/libPocoFoundation.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libroslib.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libroscpp.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/librosconsole.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/librosconsole_log4cxx.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/librosconsole_backend_interface.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/liblog4cxx.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libxmlrpcpp.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libroscpp_serialization.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/librostime.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /opt/ros/jade/lib/libcpp_common.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so"
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nodelet_uvc_camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/build: /home/bigrig/Sentinel/devel/lib/libnodelet_uvc_camera.so
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/build

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/requires: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/nodelets.cpp.o.requires
camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/requires: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/camera.cpp.o.requires
camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/requires: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/stereo.cpp.o.requires
camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/requires: camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/src/uvc_cam.cpp.o.requires
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/requires

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/clean:
	cd /home/bigrig/Sentinel/build/camera_umd/uvc_camera && $(CMAKE_COMMAND) -P CMakeFiles/nodelet_uvc_camera.dir/cmake_clean.cmake
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/clean

camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/depend:
	cd /home/bigrig/Sentinel/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bigrig/Sentinel/src /home/bigrig/Sentinel/src/camera_umd/uvc_camera /home/bigrig/Sentinel/build /home/bigrig/Sentinel/build/camera_umd/uvc_camera /home/bigrig/Sentinel/build/camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : camera_umd/uvc_camera/CMakeFiles/nodelet_uvc_camera.dir/depend

