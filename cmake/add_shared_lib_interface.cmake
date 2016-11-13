cmake_minimum_required(VERSION 2.8)

if ( BUILD_${NAME} )

    set ( BUILD_MATLAB_BINDINGS_${NAME} OFF CACHE BOOL
        "Add MATLAB bindings for ${NAME}" )

    if ( BUILD_MATLAB_BINDINGS_${NAME} )

        include(find_matlab)

        if(MATLAB_ROOT)
            message ( STATUS " _..::: Adding MATLAB to SL Bindings for ${NAME} :::.._ " )
            if ( NOT DEFINED BUILD_MATLAB_BINDINGS_${NAME} )
                set ( BUILD_MATLAB_BINDINGS_${NAME} ON CACHE BOOL
                    "Add MATLAB bindings for ${NAME}" )
            endif()
        elseif(NOT DEFINED BUILD_MATLAB_BINDINGS_${NAME} )
            message ( WARNING "MATLAB not found, SL Bindings for ${NAME} will not be build" )
            set ( BUILD_MATLAB_BINDINGS_${NAME} OFF CACHE BOOL
                "Add MATLAB bindings for ${NAME}" )
        endif()


    #############################################################################


        # Fix on OS X 10.11.4 with CMake 3.5.1
        if(APPLE)
            set(CMAKE_MACOSX_RPATH 1)
        endif(APPLE)

        include(build_mex_macro)

        include_directories(BEFORE ${CMAKE_SOURCE_DIR}/IAS/MexLib/)

        SET(MEX_SOURCE_FILES "MexComputeEFTraj;MexComputeJacobian;MexFwdDyn;MexSimControls;MexSimTraj")

        FOREACH(MEX_SOURCE_FILE ${MEX_SOURCE_FILES})
            BuildMex(
                MEXNAME 
                ${MEX_SOURCE_FILE}${NAME}
                TARGETDIR
                ${CMAKE_CURRENT_BINARY_DIR}
                SOURCE
                ${CMAKE_SOURCE_DIR}/IAS/MexLib/SharedLibInterfaces/${MEX_SOURCE_FILE}.cpp
                LIBRARIES
                ${NAME} SLcommon utility readline pthread
                )
        ENDFOREACH()

    ENDIF()

ENDIF()
