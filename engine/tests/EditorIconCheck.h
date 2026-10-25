#pragma once

#include <igui/widgets/Retained.hpp>

inline bool isAtlasIcon(ig::retained::IconId icon)
{
    ig::retained::TextureHandle texture;
    ig::retained::Rect uv;
    return ig::retained::imageIcon(icon, texture, uv) && static_cast<bool>(texture) &&
           uv.w > 0.0f && uv.h > 0.0f;
}
