cmake_minimum_required(VERSION 2.8)

if ( BUILD_${NAME} )

    set ( BUILD_MATLAB_INTERFACE_${NAME} OFF CACHE BOOL
        "Add SM interface for Matlab SL communication for ${NAME}" )

    IF ( BUILD_MATLAB_INTERFACE_${NAME} )

        include(find_matlab)

        if(MATLAB_ROOT )
            message ( STATUS " _..::: Adding Shared Mem MATLAB Communication for ${NAME} :::.._ " )
            if ( NOT DEFINED BUILD_MATLAB_INTERFACE_${NAME} )
                set ( BUILD_MATLAB_INTERFACE_${NAME} ON CACHE BOOL
                    "Add SM interface for Matlab SL communication for ${NAME}" )
            endif()
        elseif(NOT DEFINED BUILD_MATLAB_INTERFACE_${NAME} )
            message ( WARNING "MATLAB not found, MEX interfaces for ${NAME} will not be built" )
            set ( BUILD_MATLAB_INTERFACE_${NAME} OFF CACHE BOOL
                "Add SM interface for Matlab SL communication for ${NAME}" )
        endif()

    #############################################################################

        # Fix on OS X 10.11.4 with CMake 3.5.1
        if(APPLE)
            set(CMAKE_MACOSX_RPATH 1)
        endif(APPLE)

        include(build_mex_macro)

        SET( MATLAB_SM_INTERFACE_COMMON ${CMAKE_SOURCE_DIR}/IAS/src/sharedmemory.c
            ${CMAKE_SOURCE_DIR}/IAS/src/SL_episodic_communication.c
            ${CMAKE_SOURCE_DIR}/IAS/src/sem_timedwait.c
            )

        BuildMex(
            MEXNAME 
            SLGetEpisodeMex_${NAME}
            TARGETDIR
            ${CMAKE_CURRENT_BINARY_DIR}
            SOURCE
            ${CMAKE_SOURCE_DIR}/IAS/matlab/SLGetEpisodeMex.c
            ${MATLAB_SM_INTERFACE_COMMON}
            LIBRARIES
            #		${NAME} SLcommon utility readline pthread rt
            )

        BuildMex(
            MEXNAME 
            SLSendTrajectoryMex_${NAME}
            TARGETDIR
            ${CMAKE_CURRENT_BINARY_DIR}
            SOURCE
            ${CMAKE_SOURCE_DIR}/IAS/matlab/SLSendTrajectoryMex.c
            ${MATLAB_SM_INTERFACE_COMMON}
            LIBRARIES
            #		${NAME} SLcommon utility readline pthread rt
            )

        BuildMex(
            MEXNAME 
            SLGetInfoMex_${NAME}
            TARGETDIR
            ${CMAKE_CURRENT_BINARY_DIR}
            SOURCE
            ${CMAKE_SOURCE_DIR}/IAS/matlab/SLGetInfoMex.c
            ${MATLAB_SM_INTERFACE_COMMON}
            LIBRARIES
            #		${NAME} SLcommon utility readline pthread rt
            )

        set_target_properties(SLGetEpisodeMex_${NAME} PROPERTIES OUTPUT_NAME SLGetEpisodeMex PREFIX "" )
        set_target_properties(SLSendTrajectoryMex_${NAME} PROPERTIES OUTPUT_NAME SLSendTrajectoryMex PREFIX "")
        set_target_properties(SLGetInfoMex_${NAME} PROPERTIES OUTPUT_NAME SLGetInfoMex PREFIX "")

        #Adding common sources in the task
        set(SRCS_XTASK  ${SRCS_XTASK}
            ${CMAKE_SOURCE_DIR}/IAS/src/SL_episodic_communication.c
            ${CMAKE_SOURCE_DIR}/IAS/src/SL_episodic_comm_utils.c
            ${CMAKE_SOURCE_DIR}/IAS/src/sharedmemory.c
            ${CMAKE_SOURCE_DIR}/IAS/src/sem_timedwait.c
            )

    endif( )

endif( )

macro(required_matlab_shared_mem)

    if (NOT BUILD_MATLAB_INTERFACE_${NAME} )
        message (STATUS "Skipping task ${TASKNAME}, since MATLAB bindings for ${NAME} are required")
        unset(${NAME}_${TASKNAME} CACHE) #Cleaning cache entry from autodetect
        return()
    endif()

endmacro()
