#############################################################################
# which version are we using

cmake_minimum_required(VERSION 2.8)

#############################################################################
# include common cmake components

set(NAME barrett)

project(${NAME})

#############################################################################
# user defined cmake components

include(add_common_sources)


# Matlab shared memory Interface
# compiles mex functions and copies them to the build folder
# see Readme.txt in IAS/matlab folder for details on how to use the interface
include(add_matlab_shared_mem)


include(sl_task_auto_detect)

#TODO fix this one
add_subdirectory(${CMAKE_SOURCE_DIR}/IAS/src/gen_tasks/ ${CMAKE_CURRENT_BINARY_DIR}/src/gen_tasks/)

# for executables
set(SRCS_XTASK  ${SRCS_XTASK}
)

# for graphics
set(SRCS_OPENGL ${SRCS_OPENGL}
)

include(add_common_targets)

# Matlab shared lib Interface
# compiles mex functions and copies them to the build folder
# see Readme.txt in IAS/MexLib folder for details on how to use the interface
include(add_shared_lib_interface)

