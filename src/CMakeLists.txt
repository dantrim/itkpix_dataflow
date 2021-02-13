list(APPEND wblibs libHelpers)

set(YARRPATH ${PROJECT_SOURCE_DIR}/YARR)
foreach(lib ${wblibs})
    include_directories(${lib}/include)
    file(GLOB SrcFiles
        CONFIGURE_DEPENDS
        ${lib}/*.cpp
    )
    list(APPEND LibSrcFiles ${SrcFiles})
endforeach()


list(APPEND YARRLIBS "Fei4;Fe65p2;Rd53a;Rd53b;Spec;Util;Yarr")


add_library(itkpix_dataflow SHARED)
target_sources(itkpix_dataflow PRIVATE ${LibSrcFiles})
set(YARR_LINK_COMMAND -Wl,--whole-archive Yarr pthread rt -Wl,--no-whole-archive rt)
foreach(lib ${YARRLIBS})
    set(libpath ${YARRPATH}/src/lib${lib}/include)
    target_include_directories(itkpix_dataflow PUBLIC ${libpath})
    file(GLOB srcfiles
        ${YARRPATH}/src/lib${lib}/*.cpp)
    list(APPEND YARRSRC ${srcfiles})
endforeach()
target_link_libraries(itkpix_dataflow ${YARR_LINK_COMMAND})


##
## EXECUTABLES
##
add_subdirectory(util)
