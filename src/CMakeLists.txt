
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#


orocos_component( kinova_gen3 kinova_gen3_component.hpp kinova_gen3_component.cpp) #

target_compile_definitions(kinova_gen3 PRIVATE -D_OS_UNIX)

target_link_libraries(kinova_gen3 ${catkin_LIBRARIES})
# target_link_libraries(kinova_gen3 orocos-kdl)

set(KORTEX_DIR "${PROJECT_SOURCE_DIR}/include/kortex_api/")
# target_link_libraries(kinova_gen3 ${KORTEX_DIR}lib/release/libCppKinovaApi.a) #Old API
# target_link_libraries(kinova_gen3 ${KORTEX_DIR}lib/release/libKortexApi.a) # API 2.0
target_link_libraries(kinova_gen3 ${KORTEX_DIR}lib/release/libKortexApiCpp.a) # API 2.2
# target_link_libraries(kinova_gen3 ${KORTEX_DIR}lib/debug/libKortexApi.a)

include_directories(${KORTEX_DIR}include)
include_directories(${KORTEX_DIR}include/client)
include_directories(${KORTEX_DIR}include/common)
include_directories(${KORTEX_DIR}include/messages)
include_directories(${KORTEX_DIR}include/client_stubs)

orocos_install_headers(kinova_gen3_component.hpp)
