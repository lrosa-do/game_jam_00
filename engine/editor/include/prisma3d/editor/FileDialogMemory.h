#pragma once

#include "prisma3d/editor/EditorSettings.h"

#include <igui/widgets/FileDialog.hpp>

#include <ct/string.hpp>

namespace prisma3d
{

class Project;

namespace FileDialogMemory
{

void prepare(ig::retained::FileDialog& dialog, EditorSettings* settings, const char* purpose,
        const Project* project);
void remember(EditorSettings* settings, const char* purpose, const ct::String& chosen);

} // namespace FileDialogMemory

} // namespace prisma3d
