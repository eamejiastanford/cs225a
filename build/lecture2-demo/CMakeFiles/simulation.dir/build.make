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
CMAKE_SOURCE_DIR = /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build

# Include any dependencies generated for this target.
include lecture2-demo/CMakeFiles/simulation.dir/depend.make

# Include the progress variables for this target.
include lecture2-demo/CMakeFiles/simulation.dir/progress.make

# Include the compile flags for this target's objects.
include lecture2-demo/CMakeFiles/simulation.dir/flags.make

lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o: lecture2-demo/CMakeFiles/simulation.dir/flags.make
lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o: ../lecture2-demo/simulation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o"
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/lecture2-demo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simulation.dir/simulation.cpp.o -c /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/lecture2-demo/simulation.cpp

lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simulation.dir/simulation.cpp.i"
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/lecture2-demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/lecture2-demo/simulation.cpp > CMakeFiles/simulation.dir/simulation.cpp.i

lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simulation.dir/simulation.cpp.s"
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/lecture2-demo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/lecture2-demo/simulation.cpp -o CMakeFiles/simulation.dir/simulation.cpp.s

lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o.requires:

.PHONY : lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o.requires

lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o.provides: lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o.requires
	$(MAKE) -f lecture2-demo/CMakeFiles/simulation.dir/build.make lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o.provides.build
.PHONY : lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o.provides

lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o.provides.build: lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o


# Object files for target simulation
simulation_OBJECTS = \
"CMakeFiles/simulation.dir/simulation.cpp.o"

# External object files for target simulation
simulation_EXTERNAL_OBJECTS =

../bin/lecture2-demo/simulation: lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o
../bin/lecture2-demo/simulation: lecture2-demo/CMakeFiles/simulation.dir/build.make
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-common/build/libsai2-common.a
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/chai3d/build/libchai3d.a
../bin/lecture2-demo/simulation: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/lecture2-demo/simulation: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-simulation/build/libsai2-simulation.a
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-model/build/libsai2-model.a
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-model/rbdl/build/librbdl.so
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-graphics/build/libsai2-graphics.a
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/chai3d/build/libchai3d.a
../bin/lecture2-demo/simulation: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/lecture2-demo/simulation: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/lecture2-demo/simulation: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/lecture2-demo/simulation: /usr/local/lib/libglfw.so
../bin/lecture2-demo/simulation: /home/cs225a/Codes/sai2.0/core/sai2-model/rbdl/build/librbdl.so
../bin/lecture2-demo/simulation: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/lecture2-demo/simulation: /usr/local/lib/libglfw.so
../bin/lecture2-demo/simulation: lecture2-demo/CMakeFiles/simulation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/lecture2-demo/simulation"
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/lecture2-demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simulation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lecture2-demo/CMakeFiles/simulation.dir/build: ../bin/lecture2-demo/simulation

.PHONY : lecture2-demo/CMakeFiles/simulation.dir/build

lecture2-demo/CMakeFiles/simulation.dir/requires: lecture2-demo/CMakeFiles/simulation.dir/simulation.cpp.o.requires

.PHONY : lecture2-demo/CMakeFiles/simulation.dir/requires

lecture2-demo/CMakeFiles/simulation.dir/clean:
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/lecture2-demo && $(CMAKE_COMMAND) -P CMakeFiles/simulation.dir/cmake_clean.cmake
.PHONY : lecture2-demo/CMakeFiles/simulation.dir/clean

lecture2-demo/CMakeFiles/simulation.dir/depend:
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/lecture2-demo /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/lecture2-demo /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/lecture2-demo/CMakeFiles/simulation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lecture2-demo/CMakeFiles/simulation.dir/depend

