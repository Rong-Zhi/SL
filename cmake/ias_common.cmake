
cmake_minimum_required(VERSION 2.8)

message ( STATUS "" )
message ( STATUS " _..::: Configuring Core :::.._ " )
message ( STATUS "" )

set(CMAKE_C_FLAGS "-Wall -Wno-unused -Wno-strict-aliasing ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wall -Wno-unused -Wno-strict-aliasing ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)
include(detect_machtype)

# set MACHTYPE
detect_machtype()
message(STATUS  "Detected MACHTYPE= ${MACHTYPE}")
message(STATUS  "Detected HOSTNAME= ${HOST_NAME}")

add_definitions(-DUNIX)
add_definitions(-D${MACHTYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CPP_FLAGS} -fPIC")

if (${MACHTYPE} STREQUAL "x86_64mac")

  message(STATUS "Detected MACHTYPE=x86_64mac")
  include_directories(BEFORE /opt/local/include /sw/include /usr/X11/include /opt/X11/include /usr/include)
  link_directories(/opt/local/lib /usr/lib ${CMAKE_LIBRARY_PATH})
  set(LAB_STD_LIBS edit curses glut GL GLU X11 m)

  add_definitions(-D__APPLE__)

elseif (${MACHTYPE} STREQUAL "x86_64xeno" )

  message(STATUS "Detected MACHTYPE=x86_64xeno")

  set(XENOMAI_ROOT /usr/xenomai)

  exec_program(${XENOMAI_ROOT}/bin/xeno-config ARGS "--skin=native --cflags" 
  						  OUTPUT_VARIABLE XENOMAI_C_FLAGS)  
  exec_program(${XENOMAI_ROOT}/bin/xeno-config ARGS "--skin=native --ldflags" 
  						  OUTPUT_VARIABLE XENOMAI_LD_FLAGS)  
  set(CMAKE_C_FLAGS "${XENOMAI_C_FLAGS} ${CMAKE_C_FLAGS}")
  set(CMAKE_CPP_FLAGS "${XENOMAI_C_FLAGS} ${CMAKE_CPP_FLAGS}")
  set(LAB_STD_LIBS native xenomai pthread rtdk analogy rtdm edit curses nsl glut GL GLU X11 Xmu m)

  link_directories(${XENOMAI_ROOT}/lib /usr/X11/lib64 /usr/X11/lib /usr/lib64 ${CMAKE_LIBRARY_PATH})
  include_directories(BEFORE ${XENOMAI_C_FLAGS})

else()
  
  add_definitions(-D__LINUX_RT__)
  include_directories()
  link_directories(/usr/X11/lib64 /usr/X11/lib /usr/lib64 ${CMAKE_LIBRARY_PATH})
  set(LAB_STD_LIBS pthread rt edit curses nsl glut GL GLU X11 Xmu m)

endif()


