# Define the minimum CMake version that this project was developed for
cmake_minimum_required(VERSION 2.8)

# Set the project and build type
project(kollmorgen)

# cross compiling
set(cmake_system_name Linux)
set(CMAKE_C_COMPILER "arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
set(CMAKE_FIND_ROOT_PATH  /home/thy/app/gcc-arm-linux/libc)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


# Include the header files
set(CMAKE_EIGEN_PATH  /home/thy/app/eigen-eigen-3)
include_directories(${CMAKE_EIGEN_PATH})
include_directories(. ./canopen ./canopen/include ./canopen/rgm6 ./canopen/tcp ./canopen/include/unix ./canopen/include/timers_unix)
include_directories(./orocos_kinematics_alter/src ./orocos_kinematics_alter/models ./orocos_kinematics_alter/src/utilities ./orocos_kinematics_alter/examples)

#setting for path
set(DIR_ORO_UTI ${PROJECT_SOURCE_DIR}/orocos_kinematics_alter/src/utilities)
set(DIR_ORO_SRC ${PROJECT_SOURCE_DIR}/orocos_kinematics_alter/src)
set(DIR_ORO_MODEL ${PROJECT_SOURCE_DIR}/orocos_kinematics_alter/models)
set(DIR_ORO_EXAM ${PROJECT_SOURCE_DIR}/orocos_kinematics_alter/examples)

#setting path for exec
set(UR_TEST ${PROJECT_SOURCE_DIR}/orocos_kinematics_alter)
#add subdirectory exec 
add_subdirectory(${PROJECT_SOURCE_DIR}/canopen)
add_subdirectory(${DIR_ORO_SRC})
add_subdirectory(${DIR_ORO_UTI})
add_subdirectory(${DIR_ORO_MODEL})
add_subdirectory(${DIR_ORO_EXAM})

# link libaries
link_libraries(canfestival)
link_libraries(canfestival_unix)
link_libraries(rgm6)
link_libraries(tcp)
link_libraries(canfestival_unix_socket)


link_libraries(orocos_src)
link_libraries(orocos_model)
link_libraries(orocos_uti)


# root directory
aux_source_directory(. DIR_SRCS)
add_executable(kollmorgen ${DIR_SRCS})
# add_executable(ur5_test ${PROJECT_SOURCE_DIR}/test/model_test.cpp)

# link libaries
TARGET_LINK_LIBRARIES(kollmorgen canfestival canfestival_unix canfestival_unix_socket rgm6 tcp orocos_model orocos_src orocos_uti rt dl)
# TARGET_LINK_LIBRARIES(ur5_test orocos_model orocos_src orocos_uti)



set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(kollmorgen Threads::Threads)


set(CMAKE_EXE_LINKER_FLAGS "-lpthread -lrt -ldl")


