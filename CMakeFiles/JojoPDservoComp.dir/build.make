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
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wu/src/HRP3.1x/myPDservo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wu/src/HRP3.1x/myPDservo

# Include any dependencies generated for this target.
include CMakeFiles/JojoPDservoComp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/JojoPDservoComp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/JojoPDservoComp.dir/flags.make

CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o: CMakeFiles/JojoPDservoComp.dir/flags.make
CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o: JojoPDservoComp.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/wu/src/HRP3.1x/myPDservo/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o -c /home/wu/src/HRP3.1x/myPDservo/JojoPDservoComp.cpp

CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/wu/src/HRP3.1x/myPDservo/JojoPDservoComp.cpp > CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.i

CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/wu/src/HRP3.1x/myPDservo/JojoPDservoComp.cpp -o CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.s

CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o.requires:
.PHONY : CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o.requires

CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o.provides: CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o.requires
	$(MAKE) -f CMakeFiles/JojoPDservoComp.dir/build.make CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o.provides.build
.PHONY : CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o.provides

CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o.provides.build: CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o

CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o: CMakeFiles/JojoPDservoComp.dir/flags.make
CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o: JojoPDservo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/wu/src/HRP3.1x/myPDservo/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o -c /home/wu/src/HRP3.1x/myPDservo/JojoPDservo.cpp

CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/wu/src/HRP3.1x/myPDservo/JojoPDservo.cpp > CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.i

CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/wu/src/HRP3.1x/myPDservo/JojoPDservo.cpp -o CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.s

CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o.requires:
.PHONY : CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o.requires

CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o.provides: CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o.requires
	$(MAKE) -f CMakeFiles/JojoPDservoComp.dir/build.make CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o.provides.build
.PHONY : CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o.provides

CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o.provides.build: CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o

# Object files for target JojoPDservoComp
JojoPDservoComp_OBJECTS = \
"CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o" \
"CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o"

# External object files for target JojoPDservoComp
JojoPDservoComp_EXTERNAL_OBJECTS =

JojoPDservoComp: CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o
JojoPDservoComp: CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o
JojoPDservoComp: CMakeFiles/JojoPDservoComp.dir/build.make
JojoPDservoComp: CMakeFiles/JojoPDservoComp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable JojoPDservoComp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/JojoPDservoComp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/JojoPDservoComp.dir/build: JojoPDservoComp
.PHONY : CMakeFiles/JojoPDservoComp.dir/build

CMakeFiles/JojoPDservoComp.dir/requires: CMakeFiles/JojoPDservoComp.dir/JojoPDservoComp.cpp.o.requires
CMakeFiles/JojoPDservoComp.dir/requires: CMakeFiles/JojoPDservoComp.dir/JojoPDservo.cpp.o.requires
.PHONY : CMakeFiles/JojoPDservoComp.dir/requires

CMakeFiles/JojoPDservoComp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/JojoPDservoComp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/JojoPDservoComp.dir/clean

CMakeFiles/JojoPDservoComp.dir/depend:
	cd /home/wu/src/HRP3.1x/myPDservo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wu/src/HRP3.1x/myPDservo /home/wu/src/HRP3.1x/myPDservo /home/wu/src/HRP3.1x/myPDservo /home/wu/src/HRP3.1x/myPDservo /home/wu/src/HRP3.1x/myPDservo/CMakeFiles/JojoPDservoComp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/JojoPDservoComp.dir/depend

