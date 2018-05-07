# Define the minimum CMake version that this project was developed for
cmake_minimum_required(VERSION 2.8)

# Set the project and build type
project(kdl_simulation)
#set(CMAKE_C_COMPILER "g++")

# #add path
# add_subdirectory(src)
# add_subdirectory(include)
# add_subdirectory(models)

#setting for path
set(DIR_SRCS ./examples/RGMtrajTest0.c)
set(DIR_ORO_UTI ${PROJECT_SOURCE_DIR}/src/utilities)
set(DIR_ORO_SRC ${PROJECT_SOURCE_DIR}/src)
set(DIR_ORO_MODEL ${PROJECT_SOURCE_DIR}/models)
set(DIR_ORO_EIGEN_PATH /usr/include/eigen3)
set(DIR_ORO_ULI_PATH /usr/include/)

#set(DIR_INCLUDE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/utilities)
# # #set(DIR_LIB ${PROJECT_SOURCE_DIR}/lib)
# # # link lib directory
# # # LINK_DIRECTORIES(${DIR_LIB})

# # # find_library(KDL ${DIR_LIB}/orocos-kdl)
# # # find_library(KDL_MODEL ${DIR_LIB}/orocos-kdl-models)

# cross compiling
# set(cmake_system_name Linux)

# set(CMAKE_C_COMPILER "arm-linux-gnueabihf-gcc")
# set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
# set(CMAKE_FIND_ROOT_PATH  /home/tronlong/tl/gcc-linaro/arm-linux-gnueabihf/libc)
# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# include_directories(${CMAKE_FIND_ROOT_PATH}/eigen3)
include_directories(${DIR_ORO_EIGEN_PATH})



# Include the header files
include_directories(./src ./models ./src/utilities ${DIR_ORO_ULI_PATH})

#
#add_subdirectory
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/utilities)
add_subdirectory(${PROJECT_SOURCE_DIR}/models)

# link libaries
link_libraries(orocos_src)

link_libraries(orocos_model)

link_libraries(orocos_uti)

# root directory
add_executable(kdl_simulation ${DIR_SRCS})

# link libaries

TARGET_LINK_LIBRARIES(kdl_simulation orocos_model orocos_src orocos_uti)


#set(CMAKE_EXE_LINKER_FLAGS "-lpthread -lrt -ldl")

