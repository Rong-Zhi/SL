cmake_minimum_required(VERSION 2.8)

if ( BUILD_${NAME} )
    set(TASKNAME gen_matlab)

    project(${TASKNAME})

    message( STATUS "Adding ${TASKNAME} task for ${NAME}" )

    required_matlab_shared_mem()

    include_directories(
        ${CMAKE_SOURCE_DIR}/IAS/include
        )

    # for executables
    set(ADD_SRCS_TASK
        ${CMAKE_SOURCE_DIR}/IAS/src/gen_tasks/gen_matlab_task.c
        )

    finalize_auto_add_task()

endif()
