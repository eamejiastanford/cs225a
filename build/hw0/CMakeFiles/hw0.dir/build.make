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
include hw0/CMakeFiles/hw0.dir/depend.make

# Include the progress variables for this target.
include hw0/CMakeFiles/hw0.dir/progress.make

# Include the compile flags for this target's objects.
include hw0/CMakeFiles/hw0.dir/flags.make

hw0/CMakeFiles/hw0.dir/hw0.cpp.o: hw0/CMakeFiles/hw0.dir/flags.make
hw0/CMakeFiles/hw0.dir/hw0.cpp.o: ../hw0/hw0.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object hw0/CMakeFiles/hw0.dir/hw0.cpp.o"
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw0 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hw0.dir/hw0.cpp.o -c /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/hw0/hw0.cpp

hw0/CMakeFiles/hw0.dir/hw0.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hw0.dir/hw0.cpp.i"
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw0 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/hw0/hw0.cpp > CMakeFiles/hw0.dir/hw0.cpp.i

hw0/CMakeFiles/hw0.dir/hw0.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hw0.dir/hw0.cpp.s"
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw0 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/hw0/hw0.cpp -o CMakeFiles/hw0.dir/hw0.cpp.s

hw0/CMakeFiles/hw0.dir/hw0.cpp.o.requires:

.PHONY : hw0/CMakeFiles/hw0.dir/hw0.cpp.o.requires

hw0/CMakeFiles/hw0.dir/hw0.cpp.o.provides: hw0/CMakeFiles/hw0.dir/hw0.cpp.o.requires
	$(MAKE) -f hw0/CMakeFiles/hw0.dir/build.make hw0/CMakeFiles/hw0.dir/hw0.cpp.o.provides.build
.PHONY : hw0/CMakeFiles/hw0.dir/hw0.cpp.o.provides

hw0/CMakeFiles/hw0.dir/hw0.cpp.o.provides.build: hw0/CMakeFiles/hw0.dir/hw0.cpp.o


# Object files for target hw0
hw0_OBJECTS = \
"CMakeFiles/hw0.dir/hw0.cpp.o"

# External object files for target hw0
hw0_EXTERNAL_OBJECTS =

../bin/hw0/hw0: hw0/CMakeFiles/hw0.dir/hw0.cpp.o
../bin/hw0/hw0: hw0/CMakeFiles/hw0.dir/build.make
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-common/build/libsai2-common.a
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/chai3d/build/libchai3d.a
../bin/hw0/hw0: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/hw0/hw0: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-simulation/build/libsai2-simulation.a
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-model/build/libsai2-model.a
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-model/rbdl/build/librbdl.so
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-graphics/build/libsai2-graphics.a
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/build/libsai2-urdf.a
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-urdfreader/tinyxml2/build/libtinyxml2.so
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/chai3d/build/libchai3d.a
../bin/hw0/hw0: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/hw0/hw0: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/hw0/hw0: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/hw0/hw0: /usr/local/lib/libglfw.so
../bin/hw0/hw0: /home/cs225a/Codes/sai2.0/core/sai2-model/rbdl/build/librbdl.so
../bin/hw0/hw0: /usr/lib/x86_64-linux-gnu/libhiredis.so
../bin/hw0/hw0: /usr/local/lib/libglfw.so
../bin/hw0/hw0: hw0/CMakeFiles/hw0.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/hw0/hw0"
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw0 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hw0.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
hw0/CMakeFiles/hw0.dir/build: ../bin/hw0/hw0

.PHONY : hw0/CMakeFiles/hw0.dir/build

hw0/CMakeFiles/hw0.dir/requires: hw0/CMakeFiles/hw0.dir/hw0.cpp.o.requires

.PHONY : hw0/CMakeFiles/hw0.dir/requires

hw0/CMakeFiles/hw0.dir/clean:
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw0 && $(CMAKE_COMMAND) -P CMakeFiles/hw0.dir/cmake_clean.cmake
.PHONY : hw0/CMakeFiles/hw0.dir/clean

hw0/CMakeFiles/hw0.dir/depend:
	cd /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/hw0 /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw0 /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw0/CMakeFiles/hw0.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : hw0/CMakeFiles/hw0.dir/depend

