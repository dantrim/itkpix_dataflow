##
## GLOB all files
##
file(GLOB execfiles [a-zA-Z]*.cpp)

foreach(target ${execfiles})
    get_filename_component(execname ${target} NAME_WE)
    get_filename_component(srcfile ${target} NAME)
    add_executable(${execname} ${srcfile})
    target_link_libraries(${execname} Yarr)
endforeach()
