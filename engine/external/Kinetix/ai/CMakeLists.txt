add_library(kinetix-ai
    src/AgentAvoidance.cpp
    src/Behavior.cpp
    src/Entity.cpp
    src/NavMesh.cpp
    src/NavMeshBuilder.cpp
    src/NavMeshQuery.cpp
    src/NavMeshBehavior.cpp
    src/FormationBehavior.cpp
    src/GridMap.cpp
    src/GridPathfinder.cpp
    src/Group.cpp
    src/Obstacle.cpp
    src/PathfindBehavior.cpp
    src/PointOfInterest.cpp
    src/ProximityGrid.cpp
    src/SquadAI.cpp
    src/SquadEntity.cpp
    src/State.cpp
    src/StateMachine.cpp
    src/Steering.cpp
    src/VoxelMap.cpp
    src/Waypoint.cpp
    src/WaypointNetwork.cpp
    src/World.cpp
)

target_include_directories(kinetix-ai PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kinetix-ai PUBLIC kinetix-core)

add_library(kinetix::ai ALIAS kinetix-ai)
