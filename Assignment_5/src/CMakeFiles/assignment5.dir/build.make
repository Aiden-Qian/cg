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
CMAKE_SOURCE_DIR = /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src

# Include any dependencies generated for this target.
include CMakeFiles/assignment5.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/assignment5.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/assignment5.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/assignment5.dir/flags.make

CMakeFiles/assignment5.dir/main.cpp.o: CMakeFiles/assignment5.dir/flags.make
CMakeFiles/assignment5.dir/main.cpp.o: main.cpp
CMakeFiles/assignment5.dir/main.cpp.o: CMakeFiles/assignment5.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/assignment5.dir/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/assignment5.dir/main.cpp.o -MF CMakeFiles/assignment5.dir/main.cpp.o.d -o CMakeFiles/assignment5.dir/main.cpp.o -c /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src/main.cpp

CMakeFiles/assignment5.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/assignment5.dir/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src/main.cpp > CMakeFiles/assignment5.dir/main.cpp.i

CMakeFiles/assignment5.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/assignment5.dir/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src/main.cpp -o CMakeFiles/assignment5.dir/main.cpp.s

CMakeFiles/assignment5.dir/raster.cpp.o: CMakeFiles/assignment5.dir/flags.make
CMakeFiles/assignment5.dir/raster.cpp.o: raster.cpp
CMakeFiles/assignment5.dir/raster.cpp.o: CMakeFiles/assignment5.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/assignment5.dir/raster.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/assignment5.dir/raster.cpp.o -MF CMakeFiles/assignment5.dir/raster.cpp.o.d -o CMakeFiles/assignment5.dir/raster.cpp.o -c /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src/raster.cpp

CMakeFiles/assignment5.dir/raster.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/assignment5.dir/raster.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src/raster.cpp > CMakeFiles/assignment5.dir/raster.cpp.i

CMakeFiles/assignment5.dir/raster.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/assignment5.dir/raster.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src/raster.cpp -o CMakeFiles/assignment5.dir/raster.cpp.s

# Object files for target assignment5
assignment5_OBJECTS = \
"CMakeFiles/assignment5.dir/main.cpp.o" \
"CMakeFiles/assignment5.dir/raster.cpp.o"

# External object files for target assignment5
assignment5_EXTERNAL_OBJECTS =

assignment5: CMakeFiles/assignment5.dir/main.cpp.o
assignment5: CMakeFiles/assignment5.dir/raster.cpp.o
assignment5: CMakeFiles/assignment5.dir/build.make
assignment5: CMakeFiles/assignment5.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable assignment5"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/assignment5.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/assignment5.dir/build: assignment5
.PHONY : CMakeFiles/assignment5.dir/build

CMakeFiles/assignment5.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/assignment5.dir/cmake_clean.cmake
.PHONY : CMakeFiles/assignment5.dir/clean

CMakeFiles/assignment5.dir/depend:
	cd /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5 /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5 /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_5/src/CMakeFiles/assignment5.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/assignment5.dir/depend

