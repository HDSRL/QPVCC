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
include CMakeFiles/optim_lib.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/optim_lib.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/optim_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/optim_lib.dir/flags.make

CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.o: CMakeFiles/optim_lib.dir/flags.make
CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.o: ../optimization/iSWIFT/src/Auxilary.c
CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.o: CMakeFiles/optim_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.o"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.o -MF CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.o.d -o CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.o -c /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/src/Auxilary.c

CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.i"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/src/Auxilary.c > CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.i

CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.s"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/src/Auxilary.c -o CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.s

CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.o: CMakeFiles/optim_lib.dir/flags.make
CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.o: ../optimization/iSWIFT/src/Prime.c
CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.o: CMakeFiles/optim_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.o"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.o -MF CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.o.d -o CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.o -c /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/src/Prime.c

CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.i"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/src/Prime.c > CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.i

CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.s"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/src/Prime.c -o CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.s

CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.o: CMakeFiles/optim_lib.dir/flags.make
CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.o: ../optimization/iSWIFT/src/timer.c
CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.o: CMakeFiles/optim_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.o"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.o -MF CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.o.d -o CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.o -c /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/src/timer.c

CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.i"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/src/timer.c > CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.i

CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.s"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/src/timer.c -o CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.s

CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.o: CMakeFiles/optim_lib.dir/flags.make
CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.o: ../optimization/iSWIFT/ldl/src/ldl.c
CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.o: CMakeFiles/optim_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.o"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.o -MF CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.o.d -o CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.o -c /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/ldl/src/ldl.c

CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.i"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/ldl/src/ldl.c > CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.i

CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.s"
	/usr/bin/gcc-12 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/ldl/src/ldl.c -o CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.s

CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.o: CMakeFiles/optim_lib.dir/flags.make
CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.o: ../optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp
CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.o: CMakeFiles/optim_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.o"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.o -MF CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.o.d -o CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.o -c /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp

CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.i"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp > CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.i

CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.s"
	/usr/bin/g++-8 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/trec/WorkRaj/raisim_legged/QPVCC/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp -o CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.s

# Object files for target optim_lib
optim_lib_OBJECTS = \
"CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.o" \
"CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.o" \
"CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.o" \
"CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.o" \
"CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.o"

# External object files for target optim_lib
optim_lib_EXTERNAL_OBJECTS =

liboptim_lib.a: CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Auxilary.c.o
liboptim_lib.a: CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/Prime.c.o
liboptim_lib.a: CMakeFiles/optim_lib.dir/optimization/iSWIFT/src/timer.c.o
liboptim_lib.a: CMakeFiles/optim_lib.dir/optimization/iSWIFT/ldl/src/ldl.c.o
liboptim_lib.a: CMakeFiles/optim_lib.dir/optimization/iSWIFT/cpp_wrapper/iswift_qp.cpp.o
liboptim_lib.a: CMakeFiles/optim_lib.dir/build.make
liboptim_lib.a: CMakeFiles/optim_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library liboptim_lib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/optim_lib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optim_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/optim_lib.dir/build: liboptim_lib.a
.PHONY : CMakeFiles/optim_lib.dir/build

CMakeFiles/optim_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/optim_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/optim_lib.dir/clean

CMakeFiles/optim_lib.dir/depend:
	cd /home/trec/WorkRaj/raisim_legged/QPVCC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/trec/WorkRaj/raisim_legged/QPVCC /home/trec/WorkRaj/raisim_legged/QPVCC /home/trec/WorkRaj/raisim_legged/QPVCC/build /home/trec/WorkRaj/raisim_legged/QPVCC/build /home/trec/WorkRaj/raisim_legged/QPVCC/build/CMakeFiles/optim_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/optim_lib.dir/depend
