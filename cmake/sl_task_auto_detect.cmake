message ( STATUS "" )
message ( STATUS " _..::: Autodetecting tasks for ${NAME} :::.._ " )
message ( STATUS "" )


function(auto_add_task_err_chk)

    set(TASK_ADD_FUNC_FOUND FALSE )
    foreach ( CFILE ${ADD_SRCS_TASK} )

        file ( READ ${CFILE} FileData )
        string ( REGEX MATCH "add_${TASKNAME}_task" TaskDef ${FileData} )

        if ( NOT "A${TaskDef}A" STREQUAL "AA" )
            set(TASK_ADD_FUNC_FOUND TRUE)
        endif ()

    endforeach()

    if ( NOT TASK_ADD_FUNC_FOUND )
        message ( WARNING "Warning: Cound not find add_${TASKNAME}_task!")
    endif ()

endfunction()


function(auto_add_graph_err_chk)

    set(GRAPH_ADD_FUNC_FOUND FALSE )

    if ( DEFINED ADD_SRCS_OPENGL )
        foreach ( CFILE ${ADD_SRCS_OPENGL} )

            file ( READ ${CFILE} FileData )
            string ( REGEX MATCH "add_${TASKNAME}_graphics" TaskDef ${FileData} )

            if ( NOT "A${TaskDef}A" STREQUAL "AA" )
                set(GRAPH_ADD_FUNC_FOUND TRUE)
            endif ()

        endforeach()

        if ( NOT GRAPH_ADD_FUNC_FOUND )
            message ( FATAL_ERROR "Cound not find add_${TASKNAME}_graphics, but ADD_SRCS_OPENGL is defined!
            The addtional opengl functions will not be visible from the task!")
        endif ()

    endif()

    set(GRAPH_ADD_FUNC_FOUND ${GRAPH_ADD_FUNC_FOUND} PARENT_SCOPE)

endfunction()







macro(finalize_auto_add_task )

    if ( NOT DEFINED ADD_SRCS_TASK )

        message ( SEND_ERROR "ADD_SRCS_TASK in REQUIRED to be defined in ${NAME}_${TASKNAME} CMakeLists! Skipping Generation." )
        return()

    endif()

    set ( ${NAME}_${TASKNAME} ON CACHE BOOL
        "Build ${TASKNAME} for ${NAME}" )

    if ( ${NAME}_${TASKNAME} )

        auto_add_task_err_chk()
        add_library( "${NAME}_${TASKNAME}TASK"  STATIC ${ADD_SRCS_TASK} )
	
        set ( SLTASK_LIBS ${NAME}_${TASKNAME}TASK ${LIBS_XTASK} ${SLTASK_LIBS} PARENT_SCOPE)
	
        file ( APPEND ${SLAUTO_TASK_FILE} "
        extern void add_${TASKNAME}_task( void );
        add_${TASKNAME}_task();
        " )

        auto_add_graph_err_chk()
        if ( GRAPH_ADD_FUNC_FOUND )

            add_library( "${NAME}_${TASKNAME}OPENGL" STATIC ${ADD_SRCS_OPENGL} )
            set ( SLOPENGK_LIBS ${SLOPENGK_LIBS} ${NAME}_${TASKNAME}OPENGL PARENT_SCOPE)

            file ( APPEND ${SLAUTO_OPENGL_FILE} "
            extern void add_${TASKNAME}_graphics( void );
            add_${TASKNAME}_graphics();
            " )

        endif()

    endif()

endmacro()


if ( BUILD_${NAME} )

    add_definitions(-D_CMAKE_SL_TASK_AUTO_ADD_)
    set( SLAUTO_TASK_FILE "${CMAKE_CURRENT_SOURCE_DIR}/include/user_tasks_autodetect.h" )
    set( SLAUTO_OPENGL_FILE "${CMAKE_CURRENT_SOURCE_DIR}/include/user_graphics_autodetect.h" )

    file ( WRITE ${SLAUTO_TASK_FILE} "")
    file ( WRITE ${SLAUTO_OPENGL_FILE} "")

    file(GLOB_RECURSE SRC_AUTO_TASKS    "./src/*CMakeLists.txt")


    foreach ( CTASK ${SRC_AUTO_TASKS} )
        string( REPLACE "CMakeLists.txt" "" CTASK_DIR ${CTASK}  )
        add_subdirectory(${CTASK_DIR})
    endforeach()

    message ( STATUS "" )


endif()

