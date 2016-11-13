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

project(utilities)
#include($ENV{LAB_ROOT}/config/cmake/LAB.cmake)

#############################################################################
# user defined cmake components


# local defines
include_directories(BEFORE ../include)

set(SOURCES 
	brent.c 
	control.c 
	dbrent.c  
	df1dim.c  
	dfpmin.c  
	distributions.c 
	dlinmin.c  
	f1dim.c  
	frprmn.c 
	j-fixed.c 
	ldltinv.c 
	linmin.c  
	lubksb.c  
	ludcmp.c  
	mnbrak.c  
	numerics_package.c  
	powell.c  
	stability.c  
	svbksb.c  
	svdcmp.c  
	parm_opt.c 
	lookup_table.c 
	kdtree.c 
	pythag.c 
	choldc.c 
	statistics.c 
	data_sets.c
        balanc.c 
	hqr.c 
	elmhes.c 
	utility.c )

set(HEADERS
	../include/utility.h
	../include/data_sets.h
	../include/nrutil.h 
	../include/statistics.h 
	../include/utility_macros.h
	../include/mdefs.h )

add_library(utility ${SOURCES})
#install(FILES ${HEADERS} DESTINATION ${LAB_INCLUDES})
#install(TARGETS utility ARCHIVE DESTINATION ${LAB_LIBDIR})
