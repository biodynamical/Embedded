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

# Utility rule file for Arduino-upload.

# Include the progress variables for this target.
include CMakeFiles/Arduino-upload.dir/progress.make

CMakeFiles/Arduino-upload: Arduino.elf
	/usr/share/arduino/hardware/tools/avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -patmega2560 -cwiring -b115200 -P/dev/ttyACM0 -D -V -Uflash:w:Arduino.hex

Arduino-upload: CMakeFiles/Arduino-upload
Arduino-upload: CMakeFiles/Arduino-upload.dir/build.make
.PHONY : Arduino-upload

# Rule to build all files generated by this target.
CMakeFiles/Arduino-upload.dir/build: Arduino-upload
.PHONY : CMakeFiles/Arduino-upload.dir/build

CMakeFiles/Arduino-upload.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Arduino-upload.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Arduino-upload.dir/clean

CMakeFiles/Arduino-upload.dir/depend:
	cd /home/jean/Programming/BioDynamical/Arduino/cmake && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jean/Programming/BioDynamical/Arduino /home/jean/Programming/BioDynamical/Arduino /home/jean/Programming/BioDynamical/Arduino/cmake /home/jean/Programming/BioDynamical/Arduino/cmake /home/jean/Programming/BioDynamical/Arduino/cmake/CMakeFiles/Arduino-upload.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Arduino-upload.dir/depend

