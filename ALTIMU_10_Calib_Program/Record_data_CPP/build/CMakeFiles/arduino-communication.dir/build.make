# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP/build"

# Include any dependencies generated for this target.
include CMakeFiles/arduino-communication.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/arduino-communication.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/arduino-communication.dir/flags.make

CMakeFiles/arduino-communication.dir/test.cpp.o: CMakeFiles/arduino-communication.dir/flags.make
CMakeFiles/arduino-communication.dir/test.cpp.o: ../test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/arduino-communication.dir/test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/arduino-communication.dir/test.cpp.o -c "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP/test.cpp"

CMakeFiles/arduino-communication.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/arduino-communication.dir/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP/test.cpp" > CMakeFiles/arduino-communication.dir/test.cpp.i

CMakeFiles/arduino-communication.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/arduino-communication.dir/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP/test.cpp" -o CMakeFiles/arduino-communication.dir/test.cpp.s

# Object files for target arduino-communication
arduino__communication_OBJECTS = \
"CMakeFiles/arduino-communication.dir/test.cpp.o"

# External object files for target arduino-communication
arduino__communication_EXTERNAL_OBJECTS =

arduino-communication: CMakeFiles/arduino-communication.dir/test.cpp.o
arduino-communication: CMakeFiles/arduino-communication.dir/build.make
arduino-communication: libserial.a
arduino-communication: CMakeFiles/arduino-communication.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable arduino-communication"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/arduino-communication.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/arduino-communication.dir/build: arduino-communication

.PHONY : CMakeFiles/arduino-communication.dir/build

CMakeFiles/arduino-communication.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/arduino-communication.dir/cmake_clean.cmake
.PHONY : CMakeFiles/arduino-communication.dir/clean

CMakeFiles/arduino-communication.dir/depend:
	cd "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP" "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP" "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP/build" "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP/build" "/home/kiqmc/Dropbox/Proyecto de Tesis MAC/ALTIMU_10_Calib_Program/Record_data_CPP/build/CMakeFiles/arduino-communication.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/arduino-communication.dir/depend

