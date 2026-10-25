#pragma once

#include <mathc.h>
#include <ct/vector.hpp>
#include <cstdint>

namespace k2d
{

    enum class ShapeKind2D
    {
        Box,
        Circle,
        Polygon,
        Edge,
        Chain,
    };

    struct ShapeDesc2D
    {
        ShapeKind2D kind = ShapeKind2D::Box;
        Math::Vec2 offset = Math::Vec2(0.0f, 0.0f);
        float density = 1.0f;
        bool sensor = false;
        uint16_t category = 1;
        uint16_t mask = 0xFFFF;

        Math::Vec2 size = Math::Vec2(32.0f, 32.0f);
        float radius = 16.0f;
        ct::Vector<Math::Vec2> points;
        Math::Vec2 start = Math::Vec2(-32.0f, 0.0f);
        Math::Vec2 end = Math::Vec2(32.0f, 0.0f);
        bool loop = false;
    };

}
