##
## GLOB all files
##
file(GLOB execfiles [a-zA-Z]*.cpp)

set(YARR_CONTROLLERS_TO_BUILD "Spec")
set(YARR_FRONT_ENDS_TO_BUILD "Fei4;Rd53a;Star;Fe65p2;Rd53b")

set(YARRPATH ${PROJECT_SOURCE_DIR}/YARR/src)
list(APPEND libs Yarr Util)
list(APPEND libs ${YARR_CONTROLLERS_TO_BUILD} )
list(APPEND libs ${YARR_FRONT_ENDS_TO_BUILD} )

list(APPEND LibHeaderFiles "")
foreach(lib ${libs})
    include_directories(${YARRPATH}/lib${lib}/include)
    file(GLOB HeaderFiles
         CONFIGURE_DEPENDS
         ${YARRPATH}/lib${lib}/include/)
    list(APPEND LibHeaderFiles ${HeaderFiles})
endforeach()

foreach(target ${execfiles})
    get_filename_component(execname ${target} NAME_WE)
    get_filename_component(srcfile ${target} NAME)
    add_executable(${execname} ${srcfile})
    target_link_libraries(${execname} Yarr)
    target_include_directories(${execname} PUBLIC ${LibHeaderFiles})
endforeach()
