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
CMAKE_SOURCE_DIR = /home/jean/Programming/BioDynamical/Arduino

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jean/Programming/BioDynamical/Arduino/cmake

# Include any dependencies generated for this target.
include CMakeFiles/MMA7361.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MMA7361.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MMA7361.dir/flags.make

CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj: CMakeFiles/MMA7361.dir/flags.make
CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj: ../src/MMA7361.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jean/Programming/BioDynamical/Arduino/cmake/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj"
	/usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_FLAGS) -DF_CPU=16000000L -DARDUINO=100 -mmcu=atmega2560 -I"/usr/share/arduino/hardware/arduino/cores/arduino" -I"/usr/share/arduino/libraries" -I"/usr/share/arduino/hardware/arduino/variants/mega"   -o CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj -c /home/jean/Programming/BioDynamical/Arduino/src/MMA7361.cpp

CMakeFiles/MMA7361.dir/src/MMA7361.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MMA7361.dir/src/MMA7361.cpp.i"
	/usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_FLAGS) -DF_CPU=16000000L -DARDUINO=100 -mmcu=atmega2560 -I"/usr/share/arduino/hardware/arduino/cores/arduino" -I"/usr/share/arduino/libraries" -I"/usr/share/arduino/hardware/arduino/variants/mega"   -E /home/jean/Programming/BioDynamical/Arduino/src/MMA7361.cpp > CMakeFiles/MMA7361.dir/src/MMA7361.cpp.i

CMakeFiles/MMA7361.dir/src/MMA7361.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MMA7361.dir/src/MMA7361.cpp.s"
	/usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_FLAGS) -DF_CPU=16000000L -DARDUINO=100 -mmcu=atmega2560 -I"/usr/share/arduino/hardware/arduino/cores/arduino" -I"/usr/share/arduino/libraries" -I"/usr/share/arduino/hardware/arduino/variants/mega"   -S /home/jean/Programming/BioDynamical/Arduino/src/MMA7361.cpp -o CMakeFiles/MMA7361.dir/src/MMA7361.cpp.s

CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj.requires:
.PHONY : CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj.requires

CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj.provides: CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj.requires
	$(MAKE) -f CMakeFiles/MMA7361.dir/build.make CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj.provides.build
.PHONY : CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj.provides

CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj.provides.build: CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj

# Object files for target MMA7361
MMA7361_OBJECTS = \
"CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj"

# External object files for target MMA7361
MMA7361_EXTERNAL_OBJECTS =

libMMA7361.a: CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj
libMMA7361.a: CMakeFiles/MMA7361.dir/build.make
libMMA7361.a: CMakeFiles/MMA7361.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libMMA7361.a"
	$(CMAKE_COMMAND) -P CMakeFiles/MMA7361.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MMA7361.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MMA7361.dir/build: libMMA7361.a
.PHONY : CMakeFiles/MMA7361.dir/build

CMakeFiles/MMA7361.dir/requires: CMakeFiles/MMA7361.dir/src/MMA7361.cpp.obj.requires
.PHONY : CMakeFiles/MMA7361.dir/requires

CMakeFiles/MMA7361.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MMA7361.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MMA7361.dir/clean

CMakeFiles/MMA7361.dir/depend:
	cd /home/jean/Programming/BioDynamical/Arduino/cmake && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jean/Programming/BioDynamical/Arduino /home/jean/Programming/BioDynamical/Arduino /home/jean/Programming/BioDynamical/Arduino/cmake /home/jean/Programming/BioDynamical/Arduino/cmake /home/jean/Programming/BioDynamical/Arduino/cmake/CMakeFiles/MMA7361.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MMA7361.dir/depend

