cmake_minimum_required(VERSION 2.8)

set(TASKNAME ballonbeam)

project(${TASKNAME})

message( STATUS "Adding ${TASKNAME} task for ${NAME}" )

required_matlab_shared_mem()

include_directories(
    ${CMAKE_SOURCE_DIR}/IAS/include
)

# for executables
set(ADD_SRCS_TASK
    ballonbeam_sim.c
    ballonbeam_task.c
)

# for graphics
set(ADD_SRCS_OPENGL
    ballonbeam_graphics.c
)

finalize_auto_add_task()


