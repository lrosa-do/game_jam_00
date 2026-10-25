add_executable(prisma3d_runner src/main.cpp)
target_link_libraries(prisma3d_runner PRIVATE prisma3d)
set_target_properties(prisma3d_runner PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY $<TARGET_FILE_DIR:prisma3d_editor>
  OUTPUT_NAME runner)
add_dependencies(prisma3d_editor prisma3d_runner)

if(NOT MSVC)
  target_compile_options(prisma3d_runner PRIVATE ${PRISMA3D_WARNINGS} -fno-exceptions -fno-rtti)
endif()
if(NOT MSVC)
  target_link_options(prisma3d_runner PRIVATE $<$<CONFIG:Release>:-s>)
endif()
