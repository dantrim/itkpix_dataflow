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
## SOURCE CODE
##
add_subdirectory(YARR)
add_subdirectory(src)
