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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/youbot/uva_at_work_catkin/src/speech_recognizer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/youbot/uva_at_work_catkin/src/qtc/speech_recogniser

# Utility rule file for lcmgen_python.

# Include the progress variables for this target.
include CMakeFiles/lcmgen_python.dir/progress.make

CMakeFiles/lcmgen_python:
	sh -c '/usr/local/bin/lcm-gen --lazy -p /home/youbot/uva_at_work_catkin/src/speech_recognizer/lcmtypes/h2sl_xml_string_t.lcm --ppath /home/youbot/uva_at_work_catkin/src/speech_recognizer/lcmtypes/python'

lcmgen_python: CMakeFiles/lcmgen_python
lcmgen_python: CMakeFiles/lcmgen_python.dir/build.make
.PHONY : lcmgen_python

# Rule to build all files generated by this target.
CMakeFiles/lcmgen_python.dir/build: lcmgen_python
.PHONY : CMakeFiles/lcmgen_python.dir/build

CMakeFiles/lcmgen_python.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lcmgen_python.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lcmgen_python.dir/clean

CMakeFiles/lcmgen_python.dir/depend:
	cd /home/youbot/uva_at_work_catkin/src/qtc/speech_recogniser && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/youbot/uva_at_work_catkin/src/speech_recognizer /home/youbot/uva_at_work_catkin/src/speech_recognizer /home/youbot/uva_at_work_catkin/src/qtc/speech_recogniser /home/youbot/uva_at_work_catkin/src/qtc/speech_recogniser /home/youbot/uva_at_work_catkin/src/qtc/speech_recogniser/CMakeFiles/lcmgen_python.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lcmgen_python.dir/depend

