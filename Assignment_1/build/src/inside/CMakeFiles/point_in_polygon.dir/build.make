# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.21.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.21.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build

# Include any dependencies generated for this target.
include src/inside/CMakeFiles/point_in_polygon.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/inside/CMakeFiles/point_in_polygon.dir/compiler_depend.make

# Include the progress variables for this target.
include src/inside/CMakeFiles/point_in_polygon.dir/progress.make

# Include the compile flags for this target's objects.
include src/inside/CMakeFiles/point_in_polygon.dir/flags.make

src/inside/CMakeFiles/point_in_polygon.dir/main.cpp.o: src/inside/CMakeFiles/point_in_polygon.dir/flags.make
src/inside/CMakeFiles/point_in_polygon.dir/main.cpp.o: ../src/inside/main.cpp
src/inside/CMakeFiles/point_in_polygon.dir/main.cpp.o: src/inside/CMakeFiles/point_in_polygon.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/inside/CMakeFiles/point_in_polygon.dir/main.cpp.o"
	cd /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build/src/inside && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/inside/CMakeFiles/point_in_polygon.dir/main.cpp.o -MF CMakeFiles/point_in_polygon.dir/main.cpp.o.d -o CMakeFiles/point_in_polygon.dir/main.cpp.o -c /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/src/inside/main.cpp

src/inside/CMakeFiles/point_in_polygon.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/point_in_polygon.dir/main.cpp.i"
	cd /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build/src/inside && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/src/inside/main.cpp > CMakeFiles/point_in_polygon.dir/main.cpp.i

src/inside/CMakeFiles/point_in_polygon.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/point_in_polygon.dir/main.cpp.s"
	cd /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build/src/inside && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/src/inside/main.cpp -o CMakeFiles/point_in_polygon.dir/main.cpp.s

# Object files for target point_in_polygon
point_in_polygon_OBJECTS = \
"CMakeFiles/point_in_polygon.dir/main.cpp.o"

# External object files for target point_in_polygon
point_in_polygon_EXTERNAL_OBJECTS =

point_in_polygon: src/inside/CMakeFiles/point_in_polygon.dir/main.cpp.o
point_in_polygon: src/inside/CMakeFiles/point_in_polygon.dir/build.make
point_in_polygon: src/inside/CMakeFiles/point_in_polygon.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../point_in_polygon"
	cd /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build/src/inside && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/point_in_polygon.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/inside/CMakeFiles/point_in_polygon.dir/build: point_in_polygon
.PHONY : src/inside/CMakeFiles/point_in_polygon.dir/build

src/inside/CMakeFiles/point_in_polygon.dir/clean:
	cd /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build/src/inside && $(CMAKE_COMMAND) -P CMakeFiles/point_in_polygon.dir/cmake_clean.cmake
.PHONY : src/inside/CMakeFiles/point_in_polygon.dir/clean

src/inside/CMakeFiles/point_in_polygon.dir/depend:
	cd /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1 /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/src/inside /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build/src/inside /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_1/build/src/inside/CMakeFiles/point_in_polygon.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/inside/CMakeFiles/point_in_polygon.dir/depend

