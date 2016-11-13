cmake_minimum_required(VERSION 2.8)

function(detect_machtype  )

	exec_program("uname -m" OUTPUT_VARIABLE MACHTYPE)
	exec_program("uname -a | grep -c Darwin" OUTPUT_VARIABLE ISDARWIN)

	if ( ${ISDARWIN} GREATER 0 )
		set( MACHTYPE ${MACHTYPE}mac )
	endif()
	
	exec_program("uname -a | grep -c ipipe" OUTPUT_VARIABLE ISIPIPE)
	exec_program("echo ${MACHTYPE} | grep -cv xeno" OUTPUT_VARIABLE ISNOTXENO)


	if ( ${ISIPIPE} GREATER 0 AND ISNOTXENO GREATER 0 )
		set( MACHTYPE ${MACHTYPE}xeno )
	endif()

	exec_program(hostname OUTPUT_VARIABLE HOST_NAME)
	#exec_program(hostname ARGS "-f" OUTPUT_VARIABLE HOST_NAME)

	set( MACHTYPE ${MACHTYPE} PARENT_SCOPE)
	set( HOST_NAME ${HOST_NAME} PARENT_SCOPE)
endfunction()

