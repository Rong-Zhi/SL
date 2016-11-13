#############################################################################
#############################################################################
#
#  This is a CMakeList.txt file originally programmed for the CLMC/AMD labs
#  at the University of Southern California and the Max-Planck-Institute for
#  Intelligent Systems. We use a mixutre of explicit makefiles and cmake, but 
#  primarily we relay on cmake for all major compile dependencies. All our
#  software is provided under a slightly modified version of the LGPL license
#  to be found at http://www-clmc.usc.edu/software/license.
#
#  Copyright by Stefan Schaal, 2014
#
#############################################################################
#############################################################################
# which version are we using

cmake_minimum_required(VERSION 2.8)

#############################################################################
# include common cmake components

#include($ENV{LAB_ROOT}/config/cmake/LAB.cmake)

#############################################################################
# user defined cmake components


# local defines
include_directories(BEFORE ../include)
include_directories(BEFORE ${CMAKE_SOURCE_DIR}/utilities/include)

set(SRCS_LIB 
	lwpr.c 
	lwpr_script.c 
	)

set(SRCS_A2M 
	ascii2mrdplot.c 
	)

set(SRCS_XLWPR 
	lwpr_test.c 
	lwpr.c 
	lwpr_main.c 
	lwpr_script.c 
	)

set(SRCS_STAT
	lwpr_stat.c 
	lwpr.c 
	lwpr_main.c 
	lwpr_script.c 
	)

set(HEADERS
	../include/lwpr.h
	)

add_library(lwpr ${SRCS_LIB})
#install(TARGETS lwpr ARCHIVE DESTINATION ${LAB_LIBDIR})

add_executable(xlwpr ${SRCS_XLWPR})
target_link_libraries(xlwpr utility ${LAB_STD_LIBS})
#install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/xlwpr DESTINATION ${LAB_BINDIR})

add_executable(xlwprstat ${SRCS_STAT})
target_link_libraries(xlwprstat utility ${LAB_STD_LIBS})
#install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/xlwprstat DESTINATION ${LAB_BINDIR})

add_executable(xascii2clmcplot ${SRCS_A2M})
target_link_libraries(xascii2clmcplot utility ${LAB_STD_LIBS})
#install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/xascii2clmcplot DESTINATION ${LAB_BINDIR})

#install(FILES ${HEADERS} DESTINATION ${LAB_INCLUDES})
