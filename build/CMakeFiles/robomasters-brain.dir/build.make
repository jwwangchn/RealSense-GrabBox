# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/build

# Include any dependencies generated for this target.
include CMakeFiles/robomasters-brain.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/robomasters-brain.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/robomasters-brain.dir/flags.make

CMakeFiles/robomasters-brain.dir/realsence.cpp.o: CMakeFiles/robomasters-brain.dir/flags.make
CMakeFiles/robomasters-brain.dir/realsence.cpp.o: ../realsence.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/robomasters-brain.dir/realsence.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robomasters-brain.dir/realsence.cpp.o -c /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/realsence.cpp

CMakeFiles/robomasters-brain.dir/realsence.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robomasters-brain.dir/realsence.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/realsence.cpp > CMakeFiles/robomasters-brain.dir/realsence.cpp.i

CMakeFiles/robomasters-brain.dir/realsence.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robomasters-brain.dir/realsence.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/realsence.cpp -o CMakeFiles/robomasters-brain.dir/realsence.cpp.s

CMakeFiles/robomasters-brain.dir/realsence.cpp.o.requires:

.PHONY : CMakeFiles/robomasters-brain.dir/realsence.cpp.o.requires

CMakeFiles/robomasters-brain.dir/realsence.cpp.o.provides: CMakeFiles/robomasters-brain.dir/realsence.cpp.o.requires
	$(MAKE) -f CMakeFiles/robomasters-brain.dir/build.make CMakeFiles/robomasters-brain.dir/realsence.cpp.o.provides.build
.PHONY : CMakeFiles/robomasters-brain.dir/realsence.cpp.o.provides

CMakeFiles/robomasters-brain.dir/realsence.cpp.o.provides.build: CMakeFiles/robomasters-brain.dir/realsence.cpp.o


CMakeFiles/robomasters-brain.dir/main.cpp.o: CMakeFiles/robomasters-brain.dir/flags.make
CMakeFiles/robomasters-brain.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/robomasters-brain.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robomasters-brain.dir/main.cpp.o -c /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/main.cpp

CMakeFiles/robomasters-brain.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robomasters-brain.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/main.cpp > CMakeFiles/robomasters-brain.dir/main.cpp.i

CMakeFiles/robomasters-brain.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robomasters-brain.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/main.cpp -o CMakeFiles/robomasters-brain.dir/main.cpp.s

CMakeFiles/robomasters-brain.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/robomasters-brain.dir/main.cpp.o.requires

CMakeFiles/robomasters-brain.dir/main.cpp.o.provides: CMakeFiles/robomasters-brain.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/robomasters-brain.dir/build.make CMakeFiles/robomasters-brain.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/robomasters-brain.dir/main.cpp.o.provides

CMakeFiles/robomasters-brain.dir/main.cpp.o.provides.build: CMakeFiles/robomasters-brain.dir/main.cpp.o


# Object files for target robomasters-brain
robomasters__brain_OBJECTS = \
"CMakeFiles/robomasters-brain.dir/realsence.cpp.o" \
"CMakeFiles/robomasters-brain.dir/main.cpp.o"

# External object files for target robomasters-brain
robomasters__brain_EXTERNAL_OBJECTS =

robomasters-brain: CMakeFiles/robomasters-brain.dir/realsence.cpp.o
robomasters-brain: CMakeFiles/robomasters-brain.dir/main.cpp.o
robomasters-brain: CMakeFiles/robomasters-brain.dir/build.make
robomasters-brain: /usr/local/lib/libopencv_videostab.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_ts.a
robomasters-brain: /usr/local/lib/libopencv_superres.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_stitching.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_contrib.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_nonfree.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_ocl.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_gpu.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_photo.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_objdetect.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_legacy.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_video.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_ml.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_calib3d.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_features2d.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_highgui.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_imgproc.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_flann.so.2.4.13
robomasters-brain: /usr/local/lib/libopencv_core.so.2.4.13
robomasters-brain: CMakeFiles/robomasters-brain.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable robomasters-brain"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robomasters-brain.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/robomasters-brain.dir/build: robomasters-brain

.PHONY : CMakeFiles/robomasters-brain.dir/build

CMakeFiles/robomasters-brain.dir/requires: CMakeFiles/robomasters-brain.dir/realsence.cpp.o.requires
CMakeFiles/robomasters-brain.dir/requires: CMakeFiles/robomasters-brain.dir/main.cpp.o.requires

.PHONY : CMakeFiles/robomasters-brain.dir/requires

CMakeFiles/robomasters-brain.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/robomasters-brain.dir/cmake_clean.cmake
.PHONY : CMakeFiles/robomasters-brain.dir/clean

CMakeFiles/robomasters-brain.dir/depend:
	cd /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/build /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/build /home/nuc/Documents/RoboMasters/RealSense/RealSense-GrabBox/build/CMakeFiles/robomasters-brain.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/robomasters-brain.dir/depend

