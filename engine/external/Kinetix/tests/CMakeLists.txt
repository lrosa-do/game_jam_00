# Regression tests - one executable per tests/*.cpp, each registered with CTest.
# A test returns non-zero on failure; check.h gives the CHECK macros.
#
# The file prefix picks the libraries it links: core_*, 2d_*, 3d_*, ai_*.

enable_testing()

file(GLOB KINETIX_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

foreach(source ${KINETIX_TEST_SOURCES})
    get_filename_component(name ${source} NAME_WE)
    set(libs kinetix-core)
    if(name MATCHES "^2d_")
        if(NOT KINETIX_BUILD_2D)
            continue()
        endif()
        list(APPEND libs kinetix-2d)
    elseif(name MATCHES "^3d_")
        if(NOT KINETIX_BUILD_3D)
            continue()
        endif()
        list(APPEND libs kinetix-3d)
    elseif(name MATCHES "^ai_")
        if(NOT KINETIX_BUILD_AI)
            continue()
        endif()
        list(APPEND libs kinetix-ai)
    endif()
    add_executable(test_${name} ${source})
    target_include_directories(test_${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(test_${name} PRIVATE ${libs})
    add_test(NAME ${name} COMMAND test_${name})
endforeach()
