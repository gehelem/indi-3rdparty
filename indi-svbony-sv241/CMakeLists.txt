cmake_minimum_required(VERSION 3.16)
PROJECT(indi_svbony_sv241pro CXX C)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")

include(GNUInstallDirs)

set (SVBONY_VERSION_MAJOR 1)
set (SVBONY_VERSION_MINOR 0)

find_package(INDI REQUIRED)
find_package(ZLIB REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_svbony_sv241pro.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_svbony_sv241pro.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${CFITSIO_INCLUDE_DIR})
include_directories( ${SVBONY_INCLUDE_DIR})

include(CMakeCommon)

############# SVBONY SVBONY SV241 PRO ###############

add_executable(indi_svbony_sv241pro
                indi_svbony_sv241pro.cpp)
                

target_link_libraries(
    indi_svbony_sv241pro
    ${INDI_LIBRARIES}
    ${NOVA_LIBRARIES}
    ${GSL_LIBRARIES}
)

install(TARGETS indi_svbony_sv241pro RUNTIME DESTINATION bin)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_svbony_sv241pro.xml DESTINATION ${INDI_DATA_DIR})
