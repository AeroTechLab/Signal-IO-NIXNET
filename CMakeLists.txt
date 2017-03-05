cmake_minimum_required( VERSION 2.8 )
project( Robot-Control-OpenSim )

set( CMAKE_C_STANDARD 99 )
set( CMAKE_C_STANDARD_REQUIRED ON )

set( CONTROL_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../Robot-Control-Library CACHE PATH "Robot Control Library base directory" )
set( UTILS_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../Platform-Utils CACHE PATH "Platform Utils library base directory" )
set( MODULES_DIR ${CONTROL_LIBRARY_DIR} CACHE PATH "Plug-in output directory" )

add_library( NIXNET MODULE ni_can_epos.c )

set_target_properties( NIXNET PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${MODULES_DIR}/signal_io )
set_target_properties( NIXNET PROPERTIES PREFIX "" )
target_include_directories( NIXNET PUBLIC ${CMAKE_SOURCE_DIR} ${CONTROL_LIBRARY_DIR} ${UTILS_LIBRARY_DIR} )
#target_link_libraries( NIXNET -lnixnet )
