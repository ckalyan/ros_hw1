# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.4

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
.SUFFIXES:

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /afs/ir/users/d/p/dpursell/my_ros-pkg/pkgs/hw1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /afs/ir/users/d/p/dpursell/my_ros-pkg/pkgs/hw1/build

# Utility rule file for gcoverage.

CMakeFiles/gcoverage:
	rosgcov_summarize /afs/ir/users/d/p/dpursell/my_ros-pkg/pkgs/hw1 /afs/ir/users/d/p/dpursell/my_ros-pkg/pkgs/hw1/.rosgcov_files

gcoverage: CMakeFiles/gcoverage.dir/build.make

# Rule to build all files generated by this target.
CMakeFiles/gcoverage.dir/build: gcoverage
CMakeFiles/gcoverage.dir/build: CMakeFiles/gcoverage

CMakeFiles/gcoverage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gcoverage.dir/cmake_clean.cmake
