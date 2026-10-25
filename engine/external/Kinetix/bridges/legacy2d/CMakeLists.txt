add_library(legacy2d-bridge STATIC
    src/CharacterBody2D.cpp
    src/Physics2DSerializer.cpp
    src/PhysicsWorld2D.cpp
    src/RigidBody2D.cpp
)

target_include_directories(legacy2d-bridge PUBLIC include)
target_link_libraries(legacy2d-bridge PUBLIC kinetix-2d)

if(NOT KINETIX2D_ENGINE_INCLUDE_DIR)
    message(FATAL_ERROR "legacy2d-bridge needs KINETIX2D_ENGINE_INCLUDE_DIR (GameObject.h, Component.h, CanvasRenderer.h, RenderQueue.h, TileMapComponent.h, Profiler.h, Serializer.h)")
endif()
target_include_directories(legacy2d-bridge PUBLIC ${KINETIX2D_ENGINE_INCLUDE_DIR})
