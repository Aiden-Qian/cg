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
CMAKE_SOURCE_DIR = /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build

# Include any dependencies generated for this target.
include CMakeFiles/RasterViewer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/RasterViewer.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/RasterViewer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RasterViewer.dir/flags.make

CMakeFiles/RasterViewer.dir/src/raster.cpp.o: CMakeFiles/RasterViewer.dir/flags.make
CMakeFiles/RasterViewer.dir/src/raster.cpp.o: ../src/raster.cpp
CMakeFiles/RasterViewer.dir/src/raster.cpp.o: CMakeFiles/RasterViewer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RasterViewer.dir/src/raster.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RasterViewer.dir/src/raster.cpp.o -MF CMakeFiles/RasterViewer.dir/src/raster.cpp.o.d -o CMakeFiles/RasterViewer.dir/src/raster.cpp.o -c /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/src/raster.cpp

CMakeFiles/RasterViewer.dir/src/raster.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RasterViewer.dir/src/raster.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/src/raster.cpp > CMakeFiles/RasterViewer.dir/src/raster.cpp.i

CMakeFiles/RasterViewer.dir/src/raster.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RasterViewer.dir/src/raster.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/src/raster.cpp -o CMakeFiles/RasterViewer.dir/src/raster.cpp.s

CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.o: CMakeFiles/RasterViewer.dir/flags.make
CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.o: ../src/RasterViewer.cpp
CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.o: CMakeFiles/RasterViewer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.o -MF CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.o.d -o CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.o -c /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/src/RasterViewer.cpp

CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/src/RasterViewer.cpp > CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.i

CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/src/RasterViewer.cpp -o CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.s

CMakeFiles/RasterViewer.dir/src/Triangle.cpp.o: CMakeFiles/RasterViewer.dir/flags.make
CMakeFiles/RasterViewer.dir/src/Triangle.cpp.o: ../src/Triangle.cpp
CMakeFiles/RasterViewer.dir/src/Triangle.cpp.o: CMakeFiles/RasterViewer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/RasterViewer.dir/src/Triangle.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RasterViewer.dir/src/Triangle.cpp.o -MF CMakeFiles/RasterViewer.dir/src/Triangle.cpp.o.d -o CMakeFiles/RasterViewer.dir/src/Triangle.cpp.o -c /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/src/Triangle.cpp

CMakeFiles/RasterViewer.dir/src/Triangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RasterViewer.dir/src/Triangle.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/src/Triangle.cpp > CMakeFiles/RasterViewer.dir/src/Triangle.cpp.i

CMakeFiles/RasterViewer.dir/src/Triangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RasterViewer.dir/src/Triangle.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/src/Triangle.cpp -o CMakeFiles/RasterViewer.dir/src/Triangle.cpp.s

# Object files for target RasterViewer
RasterViewer_OBJECTS = \
"CMakeFiles/RasterViewer.dir/src/raster.cpp.o" \
"CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.o" \
"CMakeFiles/RasterViewer.dir/src/Triangle.cpp.o"

# External object files for target RasterViewer
RasterViewer_EXTERNAL_OBJECTS =

RasterViewer: CMakeFiles/RasterViewer.dir/src/raster.cpp.o
RasterViewer: CMakeFiles/RasterViewer.dir/src/RasterViewer.cpp.o
RasterViewer: CMakeFiles/RasterViewer.dir/src/Triangle.cpp.o
RasterViewer: CMakeFiles/RasterViewer.dir/build.make
RasterViewer: libAssignment_6.a
RasterViewer: SDL-build/libSDL2.a
RasterViewer: CMakeFiles/RasterViewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable RasterViewer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RasterViewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RasterViewer.dir/build: RasterViewer
.PHONY : CMakeFiles/RasterViewer.dir/build

CMakeFiles/RasterViewer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RasterViewer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RasterViewer.dir/clean

CMakeFiles/RasterViewer.dir/depend:
	cd /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6 /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6 /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/CMakeFiles/RasterViewer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RasterViewer.dir/depend

