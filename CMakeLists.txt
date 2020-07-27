cmake_minimum_required(VERSION 2.8.3)
project(unreal_airsim)
add_definitions(-std=c++17)

# Setup Airsim
# TODO(schmluk): Maybe there's a better way to do this, maybe catkinize
set(AIRSIM_ROOT /home/lukas/programs/AirSim)   # setup your Airsim directory here

# We add all these sources so they can be compiled with e.g. gcc, since the Airsim(Plugin) binaries need to be compiled
# with clang for compatibility with unreal engine
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)
set(RPC_LIB_INCLUDES "${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${AIRSIM_ROOT}/AirLib/deps/eigen3
        ${AIRSIM_ROOT}/AirLib/include
        ${RPC_LIB_INCLUDES}
        ${AIRSIM_ROOT}/MavLinkCom/include
        ${AIRSIM_ROOT}/MavLinkCom/common_utils
        ${OpenCV_INCLUDE_DIRS}
)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        # Modules
        src/frame_converter.cpp
        src/online_simulator/simulator.cpp
        src/online_simulator/sensor_timer.cpp
        src/simulator_processing/processor_factory.cpp
        src/simulator_processing/depth_to_pointcloud.cpp
        src/simulator_processing/infrared_id_compensation.cpp
        )

###############
# Executables #
###############
cs_add_executable(airsim_simulator_node
        app/airsim_simulator_node.cpp
        )
target_link_libraries(airsim_simulator_node ${PROJECT_NAME} ${catkin_LIBRARIES} AirLib ${RPC_LIB})


