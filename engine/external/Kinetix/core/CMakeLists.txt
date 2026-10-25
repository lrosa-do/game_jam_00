add_library(kinetix-core
    src/Math.cpp
    src/Log.cpp
    src/Color.cpp
    src/BoundsTree.cpp
    src/DebugDraw3D.cpp
)

target_include_directories(kinetix-core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(kinetix-core PUBLIC ct mathc)

add_library(kinetix::core ALIAS kinetix-core)
