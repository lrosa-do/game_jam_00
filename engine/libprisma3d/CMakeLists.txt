option(PRISMA3D_SHARED "Build prisma3d as a shared library, so a game relinks without rebuilding the engine" OFF)
if(PRISMA3D_SHARED)
  set(PRISMA3D_LIBRARY_TYPE SHARED)
else()
  set(PRISMA3D_LIBRARY_TYPE STATIC)
endif()
add_library(prisma3d ${PRISMA3D_LIBRARY_TYPE}
  src/prisma3d/Device.cpp
  src/prisma3d/Input.cpp
  src/prisma3d/InputMap.cpp
  src/prisma3d/core/BoundsTree.cpp
  src/prisma3d/core/ChaCha20.cpp
  src/prisma3d/core/Color.cpp
  src/prisma3d/core/FilePack.cpp
  src/prisma3d/core/FileSystem.cpp
  src/prisma3d/core/Pixmap.cpp
  src/prisma3d/core/Random.cpp
  src/prisma3d/core/ZipArchive.cpp
  src/prisma3d/audio/AudioServer.cpp
  src/prisma3d/audio/AudioPlayback.cpp
  src/prisma3d/render/TextureStore.cpp
  src/prisma3d/render/OceanSimulation.cpp
  src/prisma3d/render/AnimatedTexture.cpp
  src/prisma3d/render/MaterialStore.cpp
  src/prisma3d/render/MaterialLibrary.cpp
  src/prisma3d/render/LoadedMaterialLibrary.cpp
  src/prisma3d/render/AssetLoader.cpp
  src/prisma3d/render/MeshStore.cpp
  src/prisma3d/render/MeshSlice.cpp
  src/prisma3d/render/MultiMeshStorage.cpp
  src/prisma3d/render/ImpostorManager.cpp
  src/prisma3d/render/SceneMaterial.cpp
  src/prisma3d/render/Batch.cpp
  src/prisma3d/render/DecalAtlas.cpp
  src/prisma3d/render/ShadowAtlas.cpp
  src/prisma3d/render/DecalData.cpp
  src/prisma3d/render/ReflectionProbeData.cpp
  src/prisma3d/render/RenderingServer.cpp
  src/prisma3d/render/ClusterBuilder.cpp
  src/prisma3d/render/ScreenSpaceEffects.cpp
  src/prisma3d/render/DepthOfField.cpp
  src/prisma3d/main/Engine.cpp
  src/prisma3d/main/MainLoop.cpp
  src/prisma3d/main/MainTimerSync.cpp
  src/prisma3d/scene/Node.cpp
  src/prisma3d/scene/ScriptServer.cpp
  src/prisma3d/script/ZenScripting.cpp
  src/prisma3d/script/ZenBindings.cpp
  src/prisma3d/script/ZenValues.cpp
  src/prisma3d/scene/MissingNode.cpp
  src/prisma3d/scene/NodeClassDB.cpp
  src/prisma3d/scene/SceneTypes.cpp
  src/prisma3d/scene/SceneJson.cpp
  src/prisma3d/scene/SceneResources.cpp
  src/prisma3d/scene/Prefab.cpp
  src/prisma3d/scene/SceneSerializer.cpp
  src/prisma3d/scene/Node3D.cpp
  src/prisma3d/scene/Collision.cpp
  src/prisma3d/scene/Animation.cpp
  src/prisma3d/scene/Skeleton3D.cpp
  src/prisma3d/scene/BoneAttachment3D.cpp
  src/prisma3d/scene/SkeletonModifier3D.cpp
  src/prisma3d/scene/SpringBoneSimulator3D.cpp
  src/prisma3d/scene/SpringBoneCollision3D.cpp
  src/prisma3d/scene/RibbonTrail3D.cpp
  src/prisma3d/scene/ParticleCollision3D.cpp
  src/prisma3d/scene/ParticleSystem3D.cpp
  src/prisma3d/scene/HairStrands.cpp
  src/prisma3d/scene/Hair3D.cpp
  src/prisma3d/scene/PlanarReflector3D.cpp
  src/prisma3d/scene/Water3D.cpp
  src/prisma3d/scene/Ocean3D.cpp
  src/prisma3d/scene/Curve3D.cpp
  src/prisma3d/scene/Decal3D.cpp
  src/prisma3d/scene/ReflectionProbe3D.cpp
  src/prisma3d/scene/Path3D.cpp
  src/prisma3d/scene/Road3D.cpp
  src/prisma3d/scene/TreeGenerator.cpp
  src/prisma3d/scene/Tree3D.cpp
  src/prisma3d/scene/MultiMesh.cpp
  src/prisma3d/scene/MultiMeshInstance3D.cpp
  src/prisma3d/scene/Forest3D.cpp
  src/prisma3d/scene/Grass3D.cpp
  src/prisma3d/scene/AnimationPlayer.cpp
  src/prisma3d/scene/Tween.cpp
  src/prisma3d/scene/SceneTree.cpp
  src/prisma3d/scene/Viewport.cpp
  src/prisma3d/scene/Camera3D.cpp
  src/prisma3d/scene/Light3D.cpp
  src/prisma3d/scene/LightmapGI.cpp
  src/prisma3d/scene/WorldEnvironment.cpp
  src/prisma3d/scene/VisualInstance3D.cpp
  src/prisma3d/scene/ModelInstance.cpp
  src/prisma3d/scene/PrimitiveMeshes.cpp
  src/prisma3d/scene/SurfaceTool.cpp
  src/prisma3d/scene/ManualMesh3D.cpp
  src/prisma3d/scene/AudioListener3D.cpp
  src/prisma3d/scene/Control.cpp
  src/prisma3d/scene/Label.cpp
  src/prisma3d/scene/ColorRect.cpp
  src/prisma3d/scene/TextureRect.cpp
  src/prisma3d/scene/Button.cpp
  src/prisma3d/scene/LineEdit.cpp
  src/prisma3d/scene/AudioStreamPlayer3D.cpp
  src/prisma3d/scene/AudioStreamPlayer.cpp
  src/prisma3d/main/Main.cpp
  src/prisma3d/main/Project.cpp
  src/prisma3d/main/StatsOverlay.cpp
  src/prisma3d/physics/PhysicsServer3D.cpp
  src/prisma3d/import/ModelImporter.cpp
  src/prisma3d/import/GltfImporter.cpp
  src/prisma3d/import/OgreMeshImporter.cpp
  src/prisma3d/import/B3DImporter.cpp
  src/prisma3d/import/MS3DImporter.cpp
  src/prisma3d/import/ObjImporter.cpp
  src/prisma3d/import/C3DSImporter.cpp
  src/prisma3d/import/FbxImporter.cpp
  src/prisma3d/import/PMeshIO.cpp
  src/prisma3d/import/PMeshImporter.cpp
  src/prisma3d/import/PMeshWriter.cpp
  src/prisma3d/gui/GuiBackend.cpp
  src/prisma3d/gui/Gui.cpp
  src/prisma3d/gui/ProfilerPanel.cpp
  src/prisma3d/render/RenderProfiler.cpp
  src/prisma3d/render/ShaderCompiler.cpp
  src/prisma3d/render/ShaderVariants.cpp
  third_party/cgltf/cgltf_impl.c
  third_party/meshoptimizer/allocator.cpp
  third_party/meshoptimizer/simplifier.cpp
  third_party/meshoptimizer/tangentspace.cpp
  third_party/miniaudio/miniaudio_impl.c
  third_party/miniz/miniz.c
  third_party/msf_gif/msf_gif_impl.c
  third_party/ofbx/ofbx.cpp
  third_party/smaa/SmaaTextures.cpp
  third_party/stb/stb_impl.cpp)
target_include_directories(prisma3d PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_executable(prisma3d_shaderpack ${CMAKE_CURRENT_SOURCE_DIR}/../tools/shaderpack/main.cpp
  third_party/miniz/miniz.c)
target_include_directories(prisma3d_shaderpack PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/miniz)
target_link_libraries(prisma3d_shaderpack PRIVATE ct)
target_compile_definitions(prisma3d_shaderpack PRIVATE MINIZ_NO_ZLIB_COMPATIBLE_NAMES)
if(NOT MSVC)
  target_compile_options(prisma3d_shaderpack PRIVATE -w -fno-exceptions -fno-rtti)
endif()

file(GLOB PRISMA3D_SHADER_SOURCE_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.glsl
  ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.vert
  ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.frag
  ${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.comp)
set(PRISMA3D_SHADER_SOURCE_ARGS)
foreach(file ${PRISMA3D_SHADER_SOURCE_FILES})
  get_filename_component(name ${file} NAME)
  list(APPEND PRISMA3D_SHADER_SOURCE_ARGS ${name}=${file})
endforeach()
set(PRISMA3D_SHADER_SOURCES_CPP ${CMAKE_CURRENT_BINARY_DIR}/shaders/shader_sources.cpp)
add_custom_command(
  OUTPUT ${PRISMA3D_SHADER_SOURCES_CPP}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/shaders
  COMMAND prisma3d_shaderpack --sources ${PRISMA3D_SHADER_SOURCE_ARGS} --output ${PRISMA3D_SHADER_SOURCES_CPP}
  DEPENDS prisma3d_shaderpack ${PRISMA3D_SHADER_SOURCE_FILES}
  VERBATIM)
target_sources(prisma3d PRIVATE ${PRISMA3D_SHADER_SOURCES_CPP})

if(NOT TARGET spirv-cross-glsl)
  set(SPIRV_CROSS_CLI OFF CACHE BOOL "" FORCE)
  set(SPIRV_CROSS_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
  set(SPIRV_CROSS_ENABLE_C_API OFF CACHE BOOL "" FORCE)
  set(SPIRV_CROSS_SHARED OFF CACHE BOOL "" FORCE)
  add_subdirectory(${PRISMA_ROOT_DIR}/external/SPIRV-Cross ${CMAKE_BINARY_DIR}/spirv-cross-runtime EXCLUDE_FROM_ALL)
endif()


set(PRISMA3D_SCENE_VARIANTS 256)
target_sources(prisma3d PRIVATE
  src/prisma3d/terrain/Terrain.cpp
  src/prisma3d/terrain/TerrainMaps.cpp
  src/prisma3d/terrain/TerrainLayerBlendMap.cpp
  src/prisma3d/terrain/TerrainBrush.cpp
  src/prisma3d/terrain/TerrainPath.cpp
  src/prisma3d/terrain/TerrainQuadTreeNode.cpp
  src/prisma3d/render/TerrainComposite.cpp
  src/prisma3d/scene/Terrain3D.cpp
  src/prisma3d/scene/CollisionShape3D.cpp
  src/prisma3d/scene/PhysicsBody3D.cpp
  src/prisma3d/scene/Joint3D.cpp
  src/prisma3d/scene/Vehicle3D.cpp
  src/prisma3d/scene/GroundVehicle3D.cpp
  src/prisma3d/scene/CarDamage3D.cpp
  src/prisma3d/scene/SkidMarks3D.cpp
  src/prisma3d/scene/ImpactFx3D.cpp
  src/prisma3d/scene/VehicleDriver3D.cpp
  src/prisma3d/scene/BoostJet3D.cpp
  src/prisma3d/scene/Traffic3D.cpp
  src/prisma3d/scene/vehicle/CarDamage.cpp
  src/prisma3d/scene/Ragdoll3D.cpp
  src/prisma3d/scene/Dismember3D.cpp
  src/prisma3d/scene/NavigationRegion3D.cpp
  src/prisma3d/scene/Destructible3D.cpp
  src/prisma3d/scene/SoftBody3D.cpp
  src/prisma3d/scene/CharacterBody3D.cpp
  src/prisma3d/scene/CharacterRigidBody3D.cpp
  src/prisma3d/scene/TerrainPlacement.cpp)
target_include_directories(prisma3d PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/prisma3d/core
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/stb
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/smaa
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/msf_gif
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/miniz
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/dds
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/meshoptimizer
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cgltf
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ofbx
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/wicked)
target_include_directories(prisma3d SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/miniaudio)
if(UNIX AND NOT APPLE)
  find_package(Threads REQUIRED)
  target_link_libraries(prisma3d PRIVATE Threads::Threads ${CMAKE_DL_LIBS} m)
endif()
target_link_libraries(prisma3d PUBLIC prisma platform ct mathc igui::igui igui::font)
target_link_libraries(prisma3d PRIVATE kinetix-3d kinetix-ai)
target_link_libraries(prisma3d PRIVATE $<LINK_ONLY:zen_static>)
set_source_files_properties(
  src/prisma3d/script/ZenScripting.cpp
  src/prisma3d/script/ZenBindings.cpp
  src/prisma3d/script/ZenValues.cpp
  PROPERTIES INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/external/zenpy/libzen/include)
if(PRISMA_VULKAN)
  target_compile_definitions(prisma3d PUBLIC PRISMA3D_VULKAN)
endif()
find_library(PRISMA3D_EGL EGL)
if(PRISMA3D_EGL)
  target_compile_definitions(prisma3d PUBLIC PRISMA3D_HEADLESS)
  target_link_libraries(prisma3d PRIVATE ${PRISMA3D_EGL})
  if(PRISMA_VULKAN)
    find_package(Vulkan REQUIRED)
    target_link_libraries(prisma3d PRIVATE Vulkan::Vulkan)
  endif()
endif()
target_compile_definitions(prisma3d PRIVATE MINIZ_NO_ZLIB_COMPATIBLE_NAMES)
target_link_libraries(prisma3d PRIVATE glslang SPIRV glslang-default-resource-limits spirv-cross-glsl)
target_include_directories(prisma3d PRIVATE ${CMAKE_SOURCE_DIR}/external/glslang ${PRISMA_ROOT_DIR}/external/SPIRV-Cross)
set_source_files_properties(src/prisma3d/render/ShaderCompiler.cpp PROPERTIES COMPILE_OPTIONS "-fexceptions")
if(NOT MSVC)
  target_compile_options(prisma3d PRIVATE ${PRISMA3D_WARNINGS} -fno-exceptions -fno-rtti)
  set_source_files_properties(third_party/stb/stb_impl.cpp third_party/miniz/miniz.c third_party/ofbx/ofbx.cpp
                              third_party/cgltf/cgltf_impl.c
                              third_party/meshoptimizer/allocator.cpp
                              third_party/meshoptimizer/simplifier.cpp
                              third_party/meshoptimizer/tangentspace.cpp
                              third_party/miniaudio/miniaudio_impl.c
                              third_party/msf_gif/msf_gif_impl.c
                              PROPERTIES COMPILE_OPTIONS "-w")
endif()
