# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /home/nuc/Downloads/clion-2017.1.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/nuc/Downloads/clion-2017.1.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/realsense-grabbox.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/realsense-grabbox.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/realsense-grabbox.dir/flags.make

CMakeFiles/realsense-grabbox.dir/main.cpp.o: CMakeFiles/realsense-grabbox.dir/flags.make
CMakeFiles/realsense-grabbox.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/realsense-grabbox.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/realsense-grabbox.dir/main.cpp.o -c /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/main.cpp

CMakeFiles/realsense-grabbox.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/realsense-grabbox.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/main.cpp > CMakeFiles/realsense-grabbox.dir/main.cpp.i

CMakeFiles/realsense-grabbox.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/realsense-grabbox.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/main.cpp -o CMakeFiles/realsense-grabbox.dir/main.cpp.s

CMakeFiles/realsense-grabbox.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/realsense-grabbox.dir/main.cpp.o.requires

CMakeFiles/realsense-grabbox.dir/main.cpp.o.provides: CMakeFiles/realsense-grabbox.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/realsense-grabbox.dir/build.make CMakeFiles/realsense-grabbox.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/realsense-grabbox.dir/main.cpp.o.provides

CMakeFiles/realsense-grabbox.dir/main.cpp.o.provides.build: CMakeFiles/realsense-grabbox.dir/main.cpp.o


# Object files for target realsense-grabbox
realsense__grabbox_OBJECTS = \
"CMakeFiles/realsense-grabbox.dir/main.cpp.o"

# External object files for target realsense-grabbox
realsense__grabbox_EXTERNAL_OBJECTS =

realsense-grabbox: CMakeFiles/realsense-grabbox.dir/main.cpp.o
realsense-grabbox: CMakeFiles/realsense-grabbox.dir/build.make
realsense-grabbox: /usr/local/lib/libopencv_videostab.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_ts.a
realsense-grabbox: /usr/local/lib/libopencv_superres.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_stitching.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_contrib.so.2.4.13
realsense-grabbox: source/libsource_lib.a
realsense-grabbox: /usr/local/lib/libopencv_nonfree.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_ocl.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_gpu.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_photo.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_objdetect.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_legacy.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_video.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_ml.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_calib3d.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_features2d.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_highgui.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_imgproc.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_flann.so.2.4.13
realsense-grabbox: /usr/local/lib/libopencv_core.so.2.4.13
realsense-grabbox: CMakeFiles/realsense-grabbox.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable realsense-grabbox"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/realsense-grabbox.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/realsense-grabbox.dir/build: realsense-grabbox

.PHONY : CMakeFiles/realsense-grabbox.dir/build

CMakeFiles/realsense-grabbox.dir/requires: CMakeFiles/realsense-grabbox.dir/main.cpp.o.requires

.PHONY : CMakeFiles/realsense-grabbox.dir/requires

CMakeFiles/realsense-grabbox.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/realsense-grabbox.dir/cmake_clean.cmake
.PHONY : CMakeFiles/realsense-grabbox.dir/clean

CMakeFiles/realsense-grabbox.dir/depend:
	cd /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/cmake-build-debug /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/cmake-build-debug /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/cmake-build-debug/CMakeFiles/realsense-grabbox.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/realsense-grabbox.dir/depend

