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
CMAKE_SOURCE_DIR = /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/build/ros2_orb_slam

# Include any dependencies generated for this target.
include CMakeFiles/mono.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mono.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mono.dir/flags.make

CMakeFiles/mono.dir/src/mono.cpp.o: CMakeFiles/mono.dir/flags.make
CMakeFiles/mono.dir/src/mono.cpp.o: ../../src/mono.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/build/ros2_orb_slam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mono.dir/src/mono.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mono.dir/src/mono.cpp.o -c /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/src/mono.cpp

CMakeFiles/mono.dir/src/mono.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono.dir/src/mono.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/src/mono.cpp > CMakeFiles/mono.dir/src/mono.cpp.i

CMakeFiles/mono.dir/src/mono.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono.dir/src/mono.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/src/mono.cpp -o CMakeFiles/mono.dir/src/mono.cpp.s

CMakeFiles/mono.dir/src/mono-slam-node.cpp.o: CMakeFiles/mono.dir/flags.make
CMakeFiles/mono.dir/src/mono-slam-node.cpp.o: ../../src/mono-slam-node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/build/ros2_orb_slam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/mono.dir/src/mono-slam-node.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mono.dir/src/mono-slam-node.cpp.o -c /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/src/mono-slam-node.cpp

CMakeFiles/mono.dir/src/mono-slam-node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mono.dir/src/mono-slam-node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/src/mono-slam-node.cpp > CMakeFiles/mono.dir/src/mono-slam-node.cpp.i

CMakeFiles/mono.dir/src/mono-slam-node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mono.dir/src/mono-slam-node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/src/mono-slam-node.cpp -o CMakeFiles/mono.dir/src/mono-slam-node.cpp.s

# Object files for target mono
mono_OBJECTS = \
"CMakeFiles/mono.dir/src/mono.cpp.o" \
"CMakeFiles/mono.dir/src/mono-slam-node.cpp.o"

# External object files for target mono
mono_EXTERNAL_OBJECTS =

mono: CMakeFiles/mono.dir/src/mono.cpp.o
mono: CMakeFiles/mono.dir/src/mono-slam-node.cpp.o
mono: CMakeFiles/mono.dir/build.make
mono: /opt/ros/foxy/lib/libimage_transport.so
mono: /opt/ros/foxy/lib/libmessage_filters.so
mono: /opt/ros/foxy/lib/libmessage_filters.so
mono: /opt/ros/foxy/lib/librclcpp.so
mono: /opt/ros/foxy/lib/libmessage_filters.so
mono: /opt/ros/foxy/lib/librclcpp.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libament_index_cpp.so
mono: /opt/ros/foxy/lib/libclass_loader.so
mono: /opt/ros/foxy/lib/librcutils.so
mono: /opt/ros/foxy/lib/librcpputils.so
mono: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
mono: /opt/ros/foxy/lib/libtf2_ros.so
mono: /opt/ros/foxy/lib/libstatic_transform_broadcaster_node.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libmessage_filters.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libmessage_filters.so
mono: /opt/ros/foxy/lib/librclcpp.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librcl.so
mono: /opt/ros/foxy/lib/librcutils.so
mono: /opt/ros/foxy/lib/librmw.so
mono: /opt/ros/foxy/lib/librcl_action.so
mono: /opt/ros/foxy/lib/librosidl_runtime_c.so
mono: /opt/ros/foxy/lib/librclcpp_action.so
mono: /opt/ros/foxy/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librcutils.so
mono: /opt/ros/foxy/lib/libtf2.so
mono: /opt/ros/foxy/lib/librosidl_runtime_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librosidl_runtime_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librcutils.so
mono: /opt/ros/foxy/lib/librcpputils.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librosidl_runtime_c.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librclcpp.so
mono: /opt/ros/foxy/lib/libcomponent_manager.so
mono: /opt/ros/foxy/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
mono: /opt/ros/foxy/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librcutils.so
mono: /opt/ros/foxy/lib/libtf2.so
mono: ../../ORB_SLAM3/lib/libORB_SLAM3.so
mono: ../../ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
mono: ../../ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libstatic_transform_broadcaster_node.so
mono: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
mono: /opt/ros/foxy/lib/libcv_bridge.so
mono: /opt/ros/foxy/lib/libimage_transport.so
mono: /opt/ros/foxy/lib/libmessage_filters.so
mono: /opt/ros/foxy/lib/librclcpp.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libament_index_cpp.so
mono: /opt/ros/foxy/lib/libclass_loader.so
mono: /opt/ros/foxy/lib/librcutils.so
mono: /opt/ros/foxy/lib/librcpputils.so
mono: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
mono: /home/mantus/Pangolin/build/libpango_glgeometry.so
mono: /home/mantus/Pangolin/build/libpango_python.so
mono: /home/mantus/Pangolin/build/libpango_scene.so
mono: /home/mantus/Pangolin/build/libpango_tools.so
mono: /home/mantus/Pangolin/build/libpango_video.so
mono: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
mono: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
mono: /home/mantus/Pangolin/build/libpango_geometry.so
mono: /home/mantus/Pangolin/build/libtinyobj.so
mono: /home/mantus/Pangolin/build/libpango_plot.so
mono: /home/mantus/Pangolin/build/libpango_display.so
mono: /home/mantus/Pangolin/build/libpango_vars.so
mono: /home/mantus/Pangolin/build/libpango_windowing.so
mono: /home/mantus/Pangolin/build/libpango_opengl.so
mono: /usr/lib/x86_64-linux-gnu/libGLEW.so
mono: /usr/lib/x86_64-linux-gnu/libOpenGL.so
mono: /usr/lib/x86_64-linux-gnu/libGLX.so
mono: /usr/lib/x86_64-linux-gnu/libGLU.so
mono: /home/mantus/Pangolin/build/libpango_image.so
mono: /home/mantus/Pangolin/build/libpango_packetstream.so
mono: /home/mantus/Pangolin/build/libpango_core.so
mono: /opt/ros/foxy/lib/libtf2_ros.so
mono: /opt/ros/foxy/lib/libstatic_transform_broadcaster_node.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librcl.so
mono: /opt/ros/foxy/lib/librmw.so
mono: /opt/ros/foxy/lib/librcl_action.so
mono: /opt/ros/foxy/lib/librosidl_runtime_c.so
mono: /opt/ros/foxy/lib/librclcpp_action.so
mono: /opt/ros/foxy/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
mono: /opt/ros/foxy/lib/libtf2.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libcomponent_manager.so
mono: ../../ORB_SLAM3/lib/libORB_SLAM3.so
mono: ../../ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
mono: ../../ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
mono: /opt/ros/foxy/lib/libcv_bridge.so
mono: /opt/ros/foxy/lib/libtf2_ros.so
mono: /opt/ros/foxy/lib/libmessage_filters.so
mono: /opt/ros/foxy/lib/libtf2.so
mono: /opt/ros/foxy/lib/librclcpp_action.so
mono: /opt/ros/foxy/lib/librcl_action.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libtf2_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libcomponent_manager.so
mono: /opt/ros/foxy/lib/librclcpp.so
mono: /opt/ros/foxy/lib/liblibstatistics_collector.so
mono: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librcl.so
mono: /opt/ros/foxy/lib/librmw_implementation.so
mono: /opt/ros/foxy/lib/librcl_logging_spdlog.so
mono: /usr/lib/x86_64-linux-gnu/libspdlog.so.1.5.0
mono: /opt/ros/foxy/lib/librmw.so
mono: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
mono: /opt/ros/foxy/lib/libyaml.so
mono: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libtracetools.so
mono: /opt/ros/foxy/lib/libament_index_cpp.so
mono: /opt/ros/foxy/lib/libclass_loader.so
mono: /opt/ros/foxy/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
mono: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
mono: /opt/ros/foxy/lib/librosidl_typesupport_c.so
mono: /opt/ros/foxy/lib/librcpputils.so
mono: /opt/ros/foxy/lib/librosidl_runtime_c.so
mono: /opt/ros/foxy/lib/librcutils.so
mono: CMakeFiles/mono.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/build/ros2_orb_slam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable mono"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mono.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mono.dir/build: mono

.PHONY : CMakeFiles/mono.dir/build

CMakeFiles/mono.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mono.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mono.dir/clean

CMakeFiles/mono.dir/depend:
	cd /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/build/ros2_orb_slam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/build/ros2_orb_slam /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/build/ros2_orb_slam /home/mantus/kol_bot/ros2_orb_slam/ros2_orb_slam/build/ros2_orb_slam/CMakeFiles/mono.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mono.dir/depend
