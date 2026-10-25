cmake_minimum_required(VERSION 3.16)

add_library(kinetix-2d STATIC
    src/shapes.cpp
    src/manifold.cpp
    src/collide.cpp
    src/raycast.cpp
    src/shapecast.cpp
    src/slice.cpp
    src/body.cpp
    src/joint.cpp
    src/dynamictree.cpp
    src/world.cpp
    src/mousejoint.cpp
    src/wheeljoint.cpp
    src/revolutejoint.cpp
    src/distancejoint.cpp
    src/gearjoint.cpp
    src/motorjoint.cpp
    src/triangulate.cpp
    src/imageshape.cpp
    src/debugdraw.cpp
    src/tilemapcollider.cpp
)

target_include_directories(kinetix-2d PUBLIC include)
target_link_libraries(kinetix-2d PUBLIC kinetix-core)

add_library(kinetix::2d ALIAS kinetix-2d)
