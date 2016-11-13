
cmake_minimum_required(VERSION 2.8) 

project(robolab)

include(./cmake/ias_common.cmake)

file(GLOB SL_SUB_PROJECTS    "*/CMakeLists.txt")


foreach(CPROJECT ${SL_SUB_PROJECTS})
    string( REPLACE "CMakeLists.txt" "" CPROJECT_DIR ${CPROJECT}  )
    add_subdirectory(${CPROJECT_DIR})
endforeach()

# Warning if one is set it can not be changed from the command line anymore
#set(CMAKE_BUILD_TYPE RelWithDebInfo) # Optimization with debugging info
#set(CMAKE_BUILD_TYPE Release)       # Optimization 
#set(CMAKE_BUILD_TYPE Debug)         # Debug

