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

# Utility rule file for roslint_cv_camera.

# Include the progress variables for this target.
include cv_camera/CMakeFiles/roslint_cv_camera.dir/progress.make

cv_camera/CMakeFiles/roslint_cv_camera:

roslint_cv_camera: cv_camera/CMakeFiles/roslint_cv_camera
roslint_cv_camera: cv_camera/CMakeFiles/roslint_cv_camera.dir/build.make
	cd /home/bigrig/Sentinel/src/cv_camera && /opt/ros/jade/share/roslint/cmake/../../../lib/roslint/cpplint --filter=-runtime/references /home/bigrig/Sentinel/src/cv_camera/src/cv_camera_nodelet.cpp /home/bigrig/Sentinel/src/cv_camera/src/driver.cpp /home/bigrig/Sentinel/src/cv_camera/src/capture.cpp /home/bigrig/Sentinel/src/cv_camera/src/cv_camera_node.cpp /home/bigrig/Sentinel/src/cv_camera/test/test_cv_camera.cpp /home/bigrig/Sentinel/src/cv_camera/test/test_cv_camera_no_yaml.cpp /home/bigrig/Sentinel/src/cv_camera/include/cv_camera/exception.h /home/bigrig/Sentinel/src/cv_camera/include/cv_camera/capture.h /home/bigrig/Sentinel/src/cv_camera/include/cv_camera/driver.h
	cd /home/bigrig/Sentinel/src/cv_camera && /opt/ros/jade/share/roslint/cmake/../../../lib/roslint/cpplint --filter=-runtime/references src/capture.cpp src/cv_camera_node.cpp src/cv_camera_nodelet.cpp src/driver.cpp
.PHONY : roslint_cv_camera

# Rule to build all files generated by this target.
cv_camera/CMakeFiles/roslint_cv_camera.dir/build: roslint_cv_camera
.PHONY : cv_camera/CMakeFiles/roslint_cv_camera.dir/build

cv_camera/CMakeFiles/roslint_cv_camera.dir/clean:
	cd /home/bigrig/Sentinel/build/cv_camera && $(CMAKE_COMMAND) -P CMakeFiles/roslint_cv_camera.dir/cmake_clean.cmake
.PHONY : cv_camera/CMakeFiles/roslint_cv_camera.dir/clean

cv_camera/CMakeFiles/roslint_cv_camera.dir/depend:
	cd /home/bigrig/Sentinel/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bigrig/Sentinel/src /home/bigrig/Sentinel/src/cv_camera /home/bigrig/Sentinel/build /home/bigrig/Sentinel/build/cv_camera /home/bigrig/Sentinel/build/cv_camera/CMakeFiles/roslint_cv_camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cv_camera/CMakeFiles/roslint_cv_camera.dir/depend

