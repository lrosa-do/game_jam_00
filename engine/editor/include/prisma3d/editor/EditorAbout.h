#pragma once

#include <igui/widgets/ViewWidgets.hpp>

#include <ct/string.hpp>

namespace prisma3d
{

class EditorAbout
{
public:
    static const char* name();
    static const char* version();
    static const char* vendor();
    static ct::String signature();
    static ct::String windowTitle(const ct::String& document);

    static ig::retained::FloatWindow* open();
    static ig::retained::FloatWindow* dialog();
};

} // namespace prisma3d
