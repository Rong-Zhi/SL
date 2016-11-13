cmake_minimum_required(VERSION 2.8)

set(TASKNAME ball_following)

project(${TASKNAME})

message( STATUS "Adding ${TASKNAME} task for ${NAME}" )


include_directories(
    ${CMAKE_SOURCE_DIR}/IAS/include
)

# for executables
set(ADD_SRCS_TASK
    ball_following_task.c
)

finalize_auto_add_task()




