cmake_minimum_required(VERSION 2.8)

set(TASKNAME beerpong)

project(${TASKNAME})

message( STATUS "Adding ${TASKNAME} task for ${NAME}" )

required_matlab_shared_mem()

include_directories(
    ${CMAKE_SOURCE_DIR}/IAS/include
)

# for executables
set(ADD_SRCS_TASK
    beerpong_sim.c
    beerpong_task.c
    ${CMAKE_SOURCE_DIR}/IAS/src/ddmp.c
)

# for graphics
set(ADD_SRCS_OPENGL
    beerpong_graphics.c
)

finalize_auto_add_task()


#TODO Move the file to a proper location...
install(CODE 
"EXECUTE_PROCESS(COMMAND ln -sf \"${CMAKE_CURRENT_SOURCE_DIR}/w_beerpong.txt\" \"${CMAKE_CURRENT_BINARY_DIR}/../../\")"
)
