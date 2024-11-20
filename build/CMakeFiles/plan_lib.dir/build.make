# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/trec/WorkRaj/raisim_legged/QPVCC

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/trec/WorkRaj/raisim_legged/QPVCC/build

# Include any dependencies generated for this target.
include CMakeFiles/plan_lib.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/plan_lib.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/plan_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/plan_lib.dir/flags.make

CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.o: CMakeFiles/plan_lib.dir/flags.make
CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.o: ../MP_and_Con/src/ContactEst.cpp
CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.o: CMakeFiles/plan_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.o"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.o -MF CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.o.d -o CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.o -c /home/trec/WorkRaj/raisim_legged/QPVCC/MP_and_Con/src/ContactEst.cpp

CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.i"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/trec/WorkRaj/raisim_legged/QPVCC/MP_and_Con/src/ContactEst.cpp > CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.i

CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.s"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/trec/WorkRaj/raisim_legged/QPVCC/MP_and_Con/src/ContactEst.cpp -o CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.s

CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.o: CMakeFiles/plan_lib.dir/flags.make
CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.o: ../MP_and_Con/src/MotionPlanner.cpp
CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.o: CMakeFiles/plan_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.o"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.o -MF CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.o.d -o CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.o -c /home/trec/WorkRaj/raisim_legged/QPVCC/MP_and_Con/src/MotionPlanner.cpp

CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.i"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/trec/WorkRaj/raisim_legged/QPVCC/MP_and_Con/src/MotionPlanner.cpp > CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.i

CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.s"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/trec/WorkRaj/raisim_legged/QPVCC/MP_and_Con/src/MotionPlanner.cpp -o CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.s

# Object files for target plan_lib
plan_lib_OBJECTS = \
"CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.o" \
"CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.o"

# External object files for target plan_lib
plan_lib_EXTERNAL_OBJECTS =

libplan_lib.a: CMakeFiles/plan_lib.dir/MP_and_Con/src/ContactEst.cpp.o
libplan_lib.a: CMakeFiles/plan_lib.dir/MP_and_Con/src/MotionPlanner.cpp.o
libplan_lib.a: CMakeFiles/plan_lib.dir/build.make
libplan_lib.a: CMakeFiles/plan_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libplan_lib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/plan_lib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/plan_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/plan_lib.dir/build: libplan_lib.a
.PHONY : CMakeFiles/plan_lib.dir/build

CMakeFiles/plan_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/plan_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/plan_lib.dir/clean

CMakeFiles/plan_lib.dir/depend:
	cd /home/trec/WorkRaj/raisim_legged/QPVCC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/trec/WorkRaj/raisim_legged/QPVCC /home/trec/WorkRaj/raisim_legged/QPVCC /home/trec/WorkRaj/raisim_legged/QPVCC/build /home/trec/WorkRaj/raisim_legged/QPVCC/build /home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles/plan_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/plan_lib.dir/depend
