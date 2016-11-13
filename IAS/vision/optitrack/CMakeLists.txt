message ( STATUS "" )
message ( STATUS " _..::: Configuring Optitrack for ${NAME} :::.._ " )
message ( STATUS "" )

set(SRCS_XVISION
        ${CMAKE_SOURCE_DIR}/IAS/vision/optitrack/OptiTrack.cpp
        ${CMAKE_SOURCE_DIR}/IAS/vision/optitrack/OptiTrackParser.cpp
        ${CMAKE_SOURCE_DIR}/IAS/vision/optitrack/OptitrackProc.c
        ${SRCS_COMMON}
)