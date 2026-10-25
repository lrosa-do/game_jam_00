# Three testbeds, one per library, over shared shell code.
#
# Separate binaries are not just tidiness: kinetix-testbed-2d links only
# kinetix-2d, so if it builds and runs, the 2D library genuinely does not need
# the other two. A single binary linking all three could never show that.
#
# None of this is part of any library - the testbeds link them, never the
# reverse. KINETIX_BUILD_TESTBED is ON by default, but without SDL2/OpenGL
# the directory is skipped with a warning, so building Kinetix never needs SDL.

find_package(SDL2 QUIET)
find_package(OpenGL QUIET)

if(NOT SDL2_FOUND OR NOT OPENGL_FOUND)
    message(WARNING "kinetix testbeds need SDL2 and OpenGL - skipping")
    return()
endif()

# Shell: window, camera, scene registry, headless runner. Knows nothing about
# any physics or AI type.
add_library(testbed-core STATIC
    core/main.cpp
    core/renderer.cpp
    core/scene.cpp
    vendor/imgui/imgui.cpp
    vendor/imgui/imgui_draw.cpp
    vendor/imgui/imgui_tables.cpp
    vendor/imgui/imgui_widgets.cpp
    vendor/imgui/backends/imgui_impl_sdl2.cpp
    vendor/imgui/backends/imgui_impl_opengl2.cpp
)

target_include_directories(testbed-core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui
    ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(testbed-core PUBLIC
    kinetix-core
    ${SDL2_LIBRARIES}
    OpenGL::GL
    OpenGL::GLU
)

# The three land in bin/ at the top of the repo, next to the source rather
# than buried in whatever build directory happened to be used - the same place
# Radion and Kinetix2D put theirs.
set(KINETIX_TESTBED_BIN ${CMAKE_SOURCE_DIR}/bin)

# One executable per library. Adding a demo is dropping a .cpp beside its
# siblings and naming it here - the scene registers itself.
if(KINETIX_BUILD_2D)
    add_executable(kinetix-testbed-2d
        2d/debugdraw_gl.cpp
        2d/scenes_2d.cpp
        2d/demos_2d.cpp
        2d/demo_joints_2d.cpp
        2d/demo_benchmark_2d.cpp
        2d/demo_explode_slice_2d.cpp
    )
    target_link_libraries(kinetix-testbed-2d PRIVATE testbed-core kinetix-2d)
    set_target_properties(kinetix-testbed-2d PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${KINETIX_TESTBED_BIN})
    target_include_directories(kinetix-testbed-2d PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/2d)
endif()

if(KINETIX_BUILD_3D)
    add_executable(kinetix-testbed-3d
        3d/scenes_3d.cpp
        3d/demos_3d.cpp
        3d/demos_vehicle.cpp
        3d/demo_joints_3d.cpp
        3d/demo_benchmark_3d.cpp
        3d/demo_fracture_3d.cpp
        3d/demo_airplane_3d.cpp
        3d/demo_boat_3d.cpp
        3d/demo_ground_vehicles_3d.cpp
        3d/demo_actuators_3d.cpp
        3d/demo_machinery_3d.cpp
        3d/demo_helicopter_3d.cpp
        3d/render3d.cpp
    )
    target_link_libraries(kinetix-testbed-3d PRIVATE testbed-core kinetix-3d)
    target_include_directories(kinetix-testbed-3d PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/3d)
    set_target_properties(kinetix-testbed-3d PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${KINETIX_TESTBED_BIN})
endif()

if(KINETIX_BUILD_AI)
    add_executable(kinetix-testbed-ai
        ai/scenes_ai.cpp
        ai/render_ai.cpp
        ai/scenes_navmesh.cpp
        ai/demo_steering_fsm.cpp
        ai/demo_benchmark_ai.cpp
    )
    target_link_libraries(kinetix-testbed-ai PRIVATE testbed-core kinetix-ai)
    target_include_directories(kinetix-testbed-ai PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ai)
    set_target_properties(kinetix-testbed-ai PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${KINETIX_TESTBED_BIN})
endif()
