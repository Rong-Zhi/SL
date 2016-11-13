# which cmake version do we require?
cmake_minimum_required(VERSION 2.8)


if ( BUILD_${NAME} )

message(STATUS "Building ${NAME}")

add_executable("${NAME}_xtask" ${SRCS_XTASK})
target_link_libraries("${NAME}_xtask" 
        ${SLTASK_LIBS}  SLtask SLcommon 
        ${NAME}_task ${NAME} 
        utility SLopenGL ${LAB_STD_LIBS}
)


add_executable("${NAME}_xopengl" ${SRCS_XOPENGL})
target_link_libraries("${NAME}_xopengl" 
        ${SLOPENGK_LIBS} SLopenGL SLcommon 
        ${NAME}_openGL ${NAME} 
        utility ${LAB_STD_LIBS}
)


add_executable("${NAME}_xsimulation" ${SRCS_XSIM})
target_link_libraries("${NAME}_xsimulation" 
        SLsimulation SLcommon 
        ${NAME}_simulation ${NAME} 
        utility ${LAB_STD_LIBS}
)


add_executable("${NAME}_xvision" ${SRCS_XVISION})
target_link_libraries("${NAME}_xvision" 
        SLvision SLcommon 
        lwpr utility ${LAB_STD_LIBS}
)

if(${BUILD_SL_ROS})
    add_executable("${NAME}_xros" ${SRCS_XROS})
    target_link_libraries("${NAME}_xros"
                            SLros SLcommon
                            ${NAME} utility
                            ${LAB_STD_LIBS} ${EXT_ROS_LIBS})
endif()


add_executable("x${NAME}" ${SRCS_XMAIN})
target_link_libraries("x${NAME}" SLcommon utility ${LAB_STD_LIBS})

if(BUILD_${NAME}_RR)
    add_executable("xr${NAME}" ${SRCS_XRMAIN})
    target_link_libraries("xr${NAME}" SLcommon utility ${LAB_STD_LIBS})

    if ( BUILD_${NAME}_XROBOT )
        add_executable("${NAME}_xrobot" ${SRCS_XROBOT})
        target_link_libraries("${NAME}_xrobot" SLcommon utility ${LAB_STD_LIBS})
    endif()
endif()

add_executable("${NAME}_xpest" ${SRCS_XPEST})
target_link_libraries("${NAME}_xpest" 
        SLcommon utility ${LAB_STD_LIBS}
)


add_executable("${NAME}_xmotor" ${SRCS_XMOTOR})
target_link_libraries("${NAME}_xmotor" 
        SLmotor SLcommon 
        utility ${LAB_STD_LIBS}
)


add_library("${NAME}" ${SRCS_COMMON})
add_library("${NAME}_openGL" ${SRCS_OPENGL})
add_library("${NAME}_task" ${SRCS_TASK})
add_library("${NAME}_simulation" ${SRCS_SIMULATION})

install(CODE 
"EXECUTE_PROCESS(COMMAND ln -sf \"${CMAKE_CURRENT_SOURCE_DIR}/config\" \"${CMAKE_CURRENT_BINARY_DIR}\")"
)
install(CODE 
"EXECUTE_PROCESS(COMMAND ln -sf \"${CMAKE_CURRENT_SOURCE_DIR}/prefs\" \"${CMAKE_CURRENT_BINARY_DIR}\")"
)

add_custom_target(install_${PROJECT_NAME}
    $(MAKE) install
    COMMENT "Installing ${PROJECT_NAME}"
)

else()
    #message("Skipping ${NAME}")
endif()

