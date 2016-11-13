cmake_minimum_required(VERSION 2.8)
set(TASKNAME barebone)
project(${TASKNAME})
message( STATUS "Adding ${TASKNAME} task for ${NAME}" )

set(ADD_SRCS_TASK
    barebone_task.c
    barebone_sim.c
)

set(ADD_SRCS_OPENGL
    barebone_graphics.c
)

finalize_auto_add_task()
