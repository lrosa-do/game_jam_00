add_library(prisma3d_editor_core STATIC
  src/ActivatableTree.cpp
  src/AnimationLibraryEditor.cpp
  src/AssetGrid.cpp
  src/AssetImport.cpp
  src/AssetsPanel.cpp
  src/AssetTypes.cpp
  src/ClassIcons.cpp
  src/ConsolePanel.cpp
  src/CreateMenu.cpp
  src/CreateNodeDialog.cpp
  src/CreatePrimitiveDialog.cpp
  src/CreateTerrainDialog.cpp
  src/DefaultProject.cpp
  src/EditorCamera.cpp
  src/EditorAbout.cpp
  src/EditorCameraGizmo.cpp
  src/EditorDocument.cpp
  src/EditorGizmo.cpp
  src/EditorGrid.cpp
  src/EditorIcons.cpp
  src/EditorInput.cpp
  src/EditorLightGizmo.cpp
  src/EditorLog.cpp
  src/EditorNodeIcons.cpp
  src/EditorMenus.cpp
  src/EditorPlay.cpp
  src/EditorSettings.cpp
  src/EditorShapeGizmo.cpp
  src/EditorShapeHandles.cpp
  src/EditorSelection.cpp
  src/EditorShell.cpp
  src/EditorStatusBar.cpp
  src/EditorTreeEdits.cpp
  src/EditorUndoRedoManager.cpp
  src/EditorView.cpp
  src/EditorViewportPanel.cpp
  src/FileDialogMemory.cpp
  src/GamePanel.cpp
  src/HierarchyActions.cpp
  src/ImportDialog.cpp
  src/InspectorPanel.cpp
  src/LightmapBakePass.cpp
  src/LightmapBaker.cpp
  src/LightmapMeshIO.cpp
  src/LightmapPanel.cpp
  src/LightmapUnwrapJob.cpp
  src/LightmapUnwrapper.cpp
  src/MaterialPanel.cpp
  src/MaterialPreview.cpp
  src/MaterialProperties.cpp
  src/MaterialTextureRow.cpp
  src/McpPanel.cpp
  src/McpServer.cpp
  src/McpTools.cpp
  src/MeshPropertyEditor.cpp
  src/NodeNaming.cpp
  src/NumberField.cpp
  src/PrimitiveParameterForm.cpp
  src/PropertyInspector.cpp
  src/PropertyNames.cpp
  src/PropertyRow.cpp
  src/PropertyWidgets.cpp
  src/ProjectSettingsEditor.cpp
  src/ProjectUi.cpp
  src/ReparentDialog.cpp
  src/SceneTreePanel.cpp
  src/ScriptEditorPanel.cpp
  src/SectionedInspector.cpp
  src/TerrainEditTool.cpp
  src/UndoRedo.cpp
  src/ViewportRotationControl.cpp
  src/WidgetHost.cpp
)
configure_file(include/prisma3d/editor/EditorVersion.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/prisma3d/editor/EditorVersion.h @ONLY)
target_include_directories(prisma3d_editor_core PUBLIC include ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_compile_definitions(prisma3d_editor_core PRIVATE
  PRISMA3D_EDITOR_ICONS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/icons")
find_package(Threads REQUIRED)
add_library(prisma3d_xatlas STATIC third_party/xatlas/xatlas.cpp)
target_include_directories(prisma3d_xatlas PUBLIC third_party/xatlas)
target_link_libraries(prisma3d_xatlas PUBLIC Threads::Threads)
if(NOT MSVC)
  target_compile_options(prisma3d_xatlas PRIVATE -w)
endif()
target_link_libraries(prisma3d_editor_core PUBLIC prisma3d igui::widgets prisma3d_xatlas)
prisma_shaders(prisma3d_editor_core shaders/lightmap_shadow.vert shaders/lightmap_shadow.frag
  shaders/lightmap_bake.vert shaders/lightmap_bake.frag shaders/lightmap_pack.vert
  shaders/lightmap_pack.frag)

add_executable(prisma3d_editor src/main.cpp)
target_link_libraries(prisma3d_editor PRIVATE prisma3d_editor_core)
set_target_properties(prisma3d_editor PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PRISMA3D_BIN_DIR}
  OUTPUT_NAME editor)
if(TARGET prisma3d_export)
  add_dependencies(prisma3d_editor prisma3d_export)
endif()
add_custom_command(TARGET prisma3d_editor POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/icons
          $<TARGET_FILE_DIR:prisma3d_editor>/icons)

if(NOT MSVC)
  target_compile_options(prisma3d_editor_core PRIVATE ${PRISMA3D_WARNINGS} -fno-exceptions -fno-rtti)
  target_compile_options(prisma3d_editor PRIVATE ${PRISMA3D_WARNINGS} -fno-exceptions -fno-rtti)
endif()
