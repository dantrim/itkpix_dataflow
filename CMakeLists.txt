cmake_minimum_required(VERSION 3.12)
project(itkpix_dataflow)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/YARR/cmake")

if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_compile_options(-g)
else()
    #add_compile_options(-O2)
endif()

set(CMAKE_CXX_STANDARD 17)

##
## CONFIGURATION
##
set(YARR_CONTROLLERS_TO_BUILD "Spec")
set(YARR_FRONT_ENDS_TO_BUILD "Fei4;Rd53a;Star;Rd53b")

##
## SOURCE CODE
##
add_subdirectory(exts)
add_subdirectory(src)
