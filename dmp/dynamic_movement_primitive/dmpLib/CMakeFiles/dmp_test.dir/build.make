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
CMAKE_SOURCE_DIR = /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib

# Include any dependencies generated for this target.
include CMakeFiles/dmp_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dmp_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dmp_test.dir/flags.make

CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o: CMakeFiles/dmp_test.dir/flags.make
CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o: test_dmp/test_dynamic_movement_primitive.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o -c /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/test_dynamic_movement_primitive.cpp

CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/test_dynamic_movement_primitive.cpp > CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.i

CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/test_dynamic_movement_primitive.cpp -o CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.s

CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o.requires:
.PHONY : CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o.requires

CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o.provides: CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o.requires
	$(MAKE) -f CMakeFiles/dmp_test.dir/build.make CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o.provides.build
.PHONY : CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o.provides

CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o.provides.build: CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o

CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o: CMakeFiles/dmp_test.dir/flags.make
CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o: test_dmp/test_trajectory.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o -c /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/test_trajectory.cpp

CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/test_trajectory.cpp > CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.i

CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/test_trajectory.cpp -o CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.s

CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o.requires:
.PHONY : CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o.requires

CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o.provides: CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o.requires
	$(MAKE) -f CMakeFiles/dmp_test.dir/build.make CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o.provides.build
.PHONY : CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o.provides

CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o.provides.build: CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o

CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o: CMakeFiles/dmp_test.dir/flags.make
CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o: test_dmp/test_data.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o -c /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/test_data.cpp

CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/test_data.cpp > CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.i

CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/test_data.cpp -o CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.s

CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o.requires:
.PHONY : CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o.requires

CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o.provides: CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o.requires
	$(MAKE) -f CMakeFiles/dmp_test.dir/build.make CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o.provides.build
.PHONY : CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o.provides

CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o.provides.build: CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o

CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o: CMakeFiles/dmp_test.dir/flags.make
CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o: test_dmp/nc2010_test.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o -c /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/nc2010_test.cpp

CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/nc2010_test.cpp > CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.i

CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_dmp/nc2010_test.cpp -o CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.s

CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o.requires:
.PHONY : CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o.requires

CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o.provides: CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o.requires
	$(MAKE) -f CMakeFiles/dmp_test.dir/build.make CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o.provides.build
.PHONY : CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o.provides

CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o.provides.build: CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o

# Object files for target dmp_test
dmp_test_OBJECTS = \
"CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o" \
"CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o" \
"CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o" \
"CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o"

# External object files for target dmp_test
dmp_test_EXTERNAL_OBJECTS =

devel/lib/dynamic_movement_primitive/dmp_test: CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o
devel/lib/dynamic_movement_primitive/dmp_test: CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o
devel/lib/dynamic_movement_primitive/dmp_test: CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o
devel/lib/dynamic_movement_primitive/dmp_test: CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o
devel/lib/dynamic_movement_primitive/dmp_test: CMakeFiles/dmp_test.dir/build.make
devel/lib/dynamic_movement_primitive/dmp_test: devel/lib/libdmp++.so
devel/lib/dynamic_movement_primitive/dmp_test: devel/lib/liblwr.so
devel/lib/dynamic_movement_primitive/dmp_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/dynamic_movement_primitive/dmp_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/dynamic_movement_primitive/dmp_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/dynamic_movement_primitive/dmp_test: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/dynamic_movement_primitive/dmp_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/dynamic_movement_primitive/dmp_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/dynamic_movement_primitive/dmp_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/dynamic_movement_primitive/dmp_test: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/dynamic_movement_primitive/dmp_test: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/dynamic_movement_primitive/dmp_test: CMakeFiles/dmp_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable devel/lib/dynamic_movement_primitive/dmp_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dmp_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dmp_test.dir/build: devel/lib/dynamic_movement_primitive/dmp_test
.PHONY : CMakeFiles/dmp_test.dir/build

CMakeFiles/dmp_test.dir/requires: CMakeFiles/dmp_test.dir/test_dmp/test_dynamic_movement_primitive.cpp.o.requires
CMakeFiles/dmp_test.dir/requires: CMakeFiles/dmp_test.dir/test_dmp/test_trajectory.cpp.o.requires
CMakeFiles/dmp_test.dir/requires: CMakeFiles/dmp_test.dir/test_dmp/test_data.cpp.o.requires
CMakeFiles/dmp_test.dir/requires: CMakeFiles/dmp_test.dir/test_dmp/nc2010_test.cpp.o.requires
.PHONY : CMakeFiles/dmp_test.dir/requires

CMakeFiles/dmp_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dmp_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dmp_test.dir/clean

CMakeFiles/dmp_test.dir/depend:
	cd /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/CMakeFiles/dmp_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dmp_test.dir/depend

