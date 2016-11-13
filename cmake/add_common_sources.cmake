#############################################################################
# which version are we using

cmake_minimum_required(VERSION 2.8)

add_definitions(-D_SL_ROBOT_NAME_="${NAME}")

#############################################################################
message ( STATUS "" )
message ( STATUS " _..::: Configuring ${NAME} :::.._ " )
message ( STATUS "" )

include_directories(BEFORE include)
include_directories(BEFORE math)
include_directories(BEFORE ${CMAKE_SOURCE_DIR}/utilities/include/)
include_directories(BEFORE ${CMAKE_SOURCE_DIR}/SL/include/)
include_directories(${CMAKE_SOURCE_DIR}/IAS/include)

set ( BUILD_${NAME} OFF CACHE BOOL
		        "Add targets for ${NAME}" )

#Real robot compilation
if ( BUILD_${NAME} )
    option(BUILD_${NAME}_RR "Compile real-robot servo for ${NAME}" OFF )
    option(BUILD_${NAME}_XROBOT "Enable xrobot target ${NAME}" ON )
    mark_as_advanced(BUILD_${NAME}_XROBOT)
endif()

set(SRCS_COMMON
	${CMAKE_SOURCE_DIR}/SL/srcUserTemplates/SL_user_commands.c
	src/SL_user_common.c
        ${CMAKE_SOURCE_DIR}/SL/src/SL_kinematics.c
        ${CMAKE_SOURCE_DIR}/SL/src/SL_dynamics.c
        ${CMAKE_SOURCE_DIR}/SL/src/SL_invDynNE.cpp
        ${CMAKE_SOURCE_DIR}/SL/src/SL_invDynArt.cpp
        ${CMAKE_SOURCE_DIR}/SL/src/SL_forDynComp.cpp
        ${CMAKE_SOURCE_DIR}/SL/src/SL_forDynArt.cpp
        ${CMAKE_SOURCE_DIR}/IAS/src/sharedmemory.c
        ${CMAKE_SOURCE_DIR}/IAS/src/sem_timedwait.c
)


set(SRCS_XMAIN
	${CMAKE_SOURCE_DIR}/SL/src/SL_main.c
	src/SL_user_common.c
)

set(SRCS_XRMAIN
	${CMAKE_SOURCE_DIR}/SL/src/SL_rmain.c
    src/SL_user_common.c
)

set(SRCS_XVISION
	${CMAKE_SOURCE_DIR}/SL/srcUserTemplates/SL_user_vision.c
	${CMAKE_SOURCE_DIR}/SL/src/SL_vision_proc.c
	${SRCS_COMMON}
)

if ( BUILD_${NAME} )
	option(BUILD_${NAME}_VISION_OPTITRACK "Compile optitrack sources for ${NAME}" OFF )
	if ( BUILD_${NAME}_VISION_OPTITRACK )
		include(${CMAKE_SOURCE_DIR}/IAS/vision/optitrack/CMakeLists.txt)
	endif()
endif()

set(SRCS_XPEST
        ${CMAKE_SOURCE_DIR}/SL/src/SL_parm_estimate.c
        ${SRCS_COMMON}
)

set(SRCS_XMOTOR
        ${CMAKE_SOURCE_DIR}/SL/srcUserTemplates/SL_user_motor.c
        ${CMAKE_SOURCE_DIR}/SL/srcUserTemplates/SL_user_sensor_proc_unix.c
        ${SRCS_COMMON}
)

set(SRCS_XROBOT
    ${SRCS_COMMON}
)

# for libraries
set(SRCS_OPENGL 	src/SL_user_openGL.c)
set(SRCS_TASK   	${CMAKE_SOURCE_DIR}/SL/srcUserTemplates/SL_user_task.c)
set(SRCS_SIMULATION 	${CMAKE_SOURCE_DIR}/SL/srcUserTemplates/SL_user_simulation.c)

# for executables
set(SRCS_XTASK
        ${CMAKE_SOURCE_DIR}/IAS/src/ias_utilities.c
        ${CMAKE_SOURCE_DIR}/IAS/src/ias_motor_utilities.c
        ${CMAKE_SOURCE_DIR}/IAS/src/ias_matrix_utilities.c
        ${CMAKE_SOURCE_DIR}/IAS/src/quat_tools.c
        ${CMAKE_SOURCE_DIR}/IAS/src/ext_vision.c
        src/initUserTasks.c
)

set(SRCS_XOPENGL src/initUserGraphics.c)
set(SRCS_XSIM src/initUserSimulation.c)
set(SRCS_XROS ${CMAKE_SOURCE_DIR}/SL/srcUserTemplates/SL_user_ros.c)
