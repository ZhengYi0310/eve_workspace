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
CMAKE_SOURCE_DIR = /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/build

# Include any dependencies generated for this target.
include CMakeFiles/lwr.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lwr.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lwr.dir/flags.make

CMakeFiles/lwr.dir/src/lwrLib/lwr.o: CMakeFiles/lwr.dir/flags.make
CMakeFiles/lwr.dir/src/lwrLib/lwr.o: ../src/lwrLib/lwr.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/lwr.dir/src/lwrLib/lwr.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/lwr.dir/src/lwrLib/lwr.o -c /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/src/lwrLib/lwr.cpp

CMakeFiles/lwr.dir/src/lwrLib/lwr.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lwr.dir/src/lwrLib/lwr.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/src/lwrLib/lwr.cpp > CMakeFiles/lwr.dir/src/lwrLib/lwr.i

CMakeFiles/lwr.dir/src/lwrLib/lwr.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lwr.dir/src/lwrLib/lwr.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/src/lwrLib/lwr.cpp -o CMakeFiles/lwr.dir/src/lwrLib/lwr.s

CMakeFiles/lwr.dir/src/lwrLib/lwr.o.requires:
.PHONY : CMakeFiles/lwr.dir/src/lwrLib/lwr.o.requires

CMakeFiles/lwr.dir/src/lwrLib/lwr.o.provides: CMakeFiles/lwr.dir/src/lwrLib/lwr.o.requires
	$(MAKE) -f CMakeFiles/lwr.dir/build.make CMakeFiles/lwr.dir/src/lwrLib/lwr.o.provides.build
.PHONY : CMakeFiles/lwr.dir/src/lwrLib/lwr.o.provides

CMakeFiles/lwr.dir/src/lwrLib/lwr.o.provides.build: CMakeFiles/lwr.dir/src/lwrLib/lwr.o

CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o: CMakeFiles/lwr.dir/flags.make
CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o: ../src/lwrLib/lwr_parameters.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o -c /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/src/lwrLib/lwr_parameters.cpp

CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/src/lwrLib/lwr_parameters.cpp > CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.i

CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/src/lwrLib/lwr_parameters.cpp -o CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.s

CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o.requires:
.PHONY : CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o.requires

CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o.provides: CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o.requires
	$(MAKE) -f CMakeFiles/lwr.dir/build.make CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o.provides.build
.PHONY : CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o.provides

CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o.provides.build: CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o

CMakeFiles/lwr.dir/src/lwrLib/logger.o: CMakeFiles/lwr.dir/flags.make
CMakeFiles/lwr.dir/src/lwrLib/logger.o: ../src/lwrLib/logger.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/lwr.dir/src/lwrLib/logger.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/lwr.dir/src/lwrLib/logger.o -c /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/src/lwrLib/logger.cpp

CMakeFiles/lwr.dir/src/lwrLib/logger.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lwr.dir/src/lwrLib/logger.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/src/lwrLib/logger.cpp > CMakeFiles/lwr.dir/src/lwrLib/logger.i

CMakeFiles/lwr.dir/src/lwrLib/logger.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lwr.dir/src/lwrLib/logger.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/src/lwrLib/logger.cpp -o CMakeFiles/lwr.dir/src/lwrLib/logger.s

CMakeFiles/lwr.dir/src/lwrLib/logger.o.requires:
.PHONY : CMakeFiles/lwr.dir/src/lwrLib/logger.o.requires

CMakeFiles/lwr.dir/src/lwrLib/logger.o.provides: CMakeFiles/lwr.dir/src/lwrLib/logger.o.requires
	$(MAKE) -f CMakeFiles/lwr.dir/build.make CMakeFiles/lwr.dir/src/lwrLib/logger.o.provides.build
.PHONY : CMakeFiles/lwr.dir/src/lwrLib/logger.o.provides

CMakeFiles/lwr.dir/src/lwrLib/logger.o.provides.build: CMakeFiles/lwr.dir/src/lwrLib/logger.o

# Object files for target lwr
lwr_OBJECTS = \
"CMakeFiles/lwr.dir/src/lwrLib/lwr.o" \
"CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o" \
"CMakeFiles/lwr.dir/src/lwrLib/logger.o"

# External object files for target lwr
lwr_EXTERNAL_OBJECTS =

../lib/liblwr.a: CMakeFiles/lwr.dir/src/lwrLib/lwr.o
../lib/liblwr.a: CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o
../lib/liblwr.a: CMakeFiles/lwr.dir/src/lwrLib/logger.o
../lib/liblwr.a: CMakeFiles/lwr.dir/build.make
../lib/liblwr.a: CMakeFiles/lwr.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ../lib/liblwr.a"
	$(CMAKE_COMMAND) -P CMakeFiles/lwr.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lwr.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lwr.dir/build: ../lib/liblwr.a
.PHONY : CMakeFiles/lwr.dir/build

CMakeFiles/lwr.dir/requires: CMakeFiles/lwr.dir/src/lwrLib/lwr.o.requires
CMakeFiles/lwr.dir/requires: CMakeFiles/lwr.dir/src/lwrLib/lwr_parameters.o.requires
CMakeFiles/lwr.dir/requires: CMakeFiles/lwr.dir/src/lwrLib/logger.o.requires
.PHONY : CMakeFiles/lwr.dir/requires

CMakeFiles/lwr.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lwr.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lwr.dir/clean

CMakeFiles/lwr.dir/depend:
	cd /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/build /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/build /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/build/CMakeFiles/lwr.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lwr.dir/depend
