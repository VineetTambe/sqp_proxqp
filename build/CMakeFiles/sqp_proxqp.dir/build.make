# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/vrex/.local/lib/python3.10/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/vrex/.local/lib/python3.10/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vrex/symForce/symforce_ws/sqp_proxqp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vrex/symForce/symforce_ws/sqp_proxqp/build

# Include any dependencies generated for this target.
include CMakeFiles/sqp_proxqp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/sqp_proxqp.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/sqp_proxqp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sqp_proxqp.dir/flags.make

CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.o: CMakeFiles/sqp_proxqp.dir/flags.make
CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.o: /home/vrex/symForce/symforce_ws/sqp_proxqp/src/sqp_proxqp.cpp
CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.o: CMakeFiles/sqp_proxqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vrex/symForce/symforce_ws/sqp_proxqp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.o -MF CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.o.d -o CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.o -c /home/vrex/symForce/symforce_ws/sqp_proxqp/src/sqp_proxqp.cpp

CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vrex/symForce/symforce_ws/sqp_proxqp/src/sqp_proxqp.cpp > CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.i

CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vrex/symForce/symforce_ws/sqp_proxqp/src/sqp_proxqp.cpp -o CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.s

# Object files for target sqp_proxqp
sqp_proxqp_OBJECTS = \
"CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.o"

# External object files for target sqp_proxqp
sqp_proxqp_EXTERNAL_OBJECTS =

libsqp_proxqp.a: CMakeFiles/sqp_proxqp.dir/src/sqp_proxqp.cpp.o
libsqp_proxqp.a: CMakeFiles/sqp_proxqp.dir/build.make
libsqp_proxqp.a: CMakeFiles/sqp_proxqp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vrex/symForce/symforce_ws/sqp_proxqp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libsqp_proxqp.a"
	$(CMAKE_COMMAND) -P CMakeFiles/sqp_proxqp.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sqp_proxqp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sqp_proxqp.dir/build: libsqp_proxqp.a
.PHONY : CMakeFiles/sqp_proxqp.dir/build

CMakeFiles/sqp_proxqp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sqp_proxqp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sqp_proxqp.dir/clean

CMakeFiles/sqp_proxqp.dir/depend:
	cd /home/vrex/symForce/symforce_ws/sqp_proxqp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vrex/symForce/symforce_ws/sqp_proxqp /home/vrex/symForce/symforce_ws/sqp_proxqp /home/vrex/symForce/symforce_ws/sqp_proxqp/build /home/vrex/symForce/symforce_ws/sqp_proxqp/build /home/vrex/symForce/symforce_ws/sqp_proxqp/build/CMakeFiles/sqp_proxqp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sqp_proxqp.dir/depend

