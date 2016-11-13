# BuildMex.cmake
# Author: Kent Williams norman-k-williams at uiowa.edu

# Comment: We prefer this macro as it reads everything from 
# the mex script itselft instead of setting it to the default
# values from find_matlab script!
# -Alex
include(CMakeParseArguments)

if(NOT MATLAB_ROOT)
    #Skip dreadful system scanner, use own -Alex
    #find_package(MATLAB REQUIRED)
    include(find_matlab)
endif()
# CMake 2.8.12 & earlier apparently don't define the
# Mex script path, so find it.
if(NOT MATLAB_MEX_PATH)
  find_program( MATLAB_MEX_PATH ./mex
    HINTS ${MATLAB_ROOT}/bin
    PATHS ${MATLAB_ROOT}/bin
    DOC "The mex program path"
    )
  if(NOT MATLAB_MEX_PATH)
      message("BuildMexMacro: Can't find Matlab MEX compiler")
  endif()
endif()

if ( MATLAB_ROOT AND MATLAB_MEX_PATH )

include_directories(${MATLAB_INCLUDE_DIR})

# mex -v outputs all the settings used for building MEX files, so it
# we can use it to grab the important variables needed to generate
# a well formed mex file.
execute_process(COMMAND ${MATLAB_ROOT}/bin/mex -v
  OUTPUT_VARIABLE mexOut
  ERROR_VARIABLE mexErr
  WORKING_DIRECTORY ${MATLAB_ROOT}/bin
)

# parse mex output line by line by turning file into CMake list of lines
string(REGEX REPLACE "\r?\n" ";" _mexOut "${mexOut}")
foreach(line ${_mexOut})  
  if("${line}" MATCHES " CXXFLAGS *=")
    string(REGEX REPLACE " *CXXFLAGS *= *" "" mexCxxFlags "${line}")
  elseif("${line}" MATCHES " CXXLIBS *=")
    string(REGEX REPLACE " *CXXLIBS *= *" "" mexCxxLibs "${line}")
  elseif("${line}" MATCHES " LDFLAGS *=")
    string(REGEX REPLACE " *LDFLAGS *= *" "" mexLdFlags "${line}")
  elseif("${line}" MATCHES " *LDEXTENSION *=")
    string(REGEX REPLACE " *LDEXTENSION *= *" "" mexLdExtension "${line}")
  endif()
endforeach()

if ( "${mexLdExtension}" STREQUAL "" )
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set ( mexLdExtension ".mexglx" )
    else()
        set (mexLdExtension ".mexa64" )
    endif()
endif()

set ( MATLAB_MEX_EXTENSION ${mexLdExtension} CACHE STRING 
    "Extension for mex files, depends of system Arch" )

list(APPEND mexCxxFlags "-DMATLAB_MEX_FILE")

# BuildMex -- arguments
# MEXNAME = root of mex library name
# TARGETDIR = location for the mex library files to be created
# SOURCE = list of source files
# LIBRARIES = libraries needed to link mex library
macro(BuildMex)
  set(oneValueArgs MEXNAME TARGETDIR)
  set(multiValueArgs SOURCE LIBRARIES)
  cmake_parse_arguments(BuildMex "" "${oneValueArgs}" "${multiValueArgs}"
${ARGN})
  set_source_files_properties(${BuildMex_SOURCE}    COMPILE_DEFINITIONS
${mexCxxFlags}
    )
  add_library(${BuildMex_MEXNAME} SHARED ${BuildMex_SOURCE})
  set_target_properties(${BuildMex_MEXNAME} PROPERTIES
    SUFFIX "${MATLAB_MEX_EXTENSION}"
    RUNTIME_OUTPUT_DIRECTORY "${BuildMex_TARGETDIR}"
    ARCHIVE_OUTPUT_DIRECTORY "${BuildMex_TARGETDIR}"
    LIBRARY_OUTPUT_DIRECTORY "${BuildMex_TARGETDIR}"
    )
  target_link_libraries(${BuildMex_MEXNAME} ${BuildMex_LIBRARIES}
${mexCxxLibs})
endmacro(BuildMex)

endif()
