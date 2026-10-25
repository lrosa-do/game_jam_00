#pragma once

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PropertyValue.h"

#include <ct/string.hpp>

#include <stddef.h>
#include <stdint.h>

namespace
{

using namespace prisma3d;

PropertyValue withAlpha(const Math::Vec4& color, bool noAlpha)
{
    return PropertyValue::fromColor(
            Math::Vec4(color.x, color.y, color.z, noAlpha ? 1.0f : color.w));
}

double quarterStep(const PropertyInfo& info)
{
    const double quarter = (info.rangeMax - info.rangeMin) * 0.25;
    return quarter < 7.5 ? quarter : 7.5;
}

double clampToRange(const PropertyInfo& info, double value)
{
    if (info.hint != PropertyHint::Range) return value;
    if (!info.rangeOrLess && value < info.rangeMin) value = info.rangeMin;
    if (!info.rangeOrGreater && value > info.rangeMax) value = info.rangeMax;
    return value;
}

bool overrideValue(const ct::String& className, const ct::String& property, PropertyValue& out)
{
    if (className == "Grass3D" && property == "wind_direction")
    {
        out = PropertyValue::fromVec3(Math::Vec3(0.0f, 0.0f, 1.0f));
        return true;
    }
    if (className == "ParticleSystem3D" && property == "emission_direction")
    {
        out = PropertyValue::fromVec3(Math::Vec3(1.0f, 0.0f, 0.0f));
        return true;
    }
    if ((className == "Viewport" || className == "Window") &&
            (property == "size" || property == "scaling_3d_size"))
    {
        out = PropertyValue::fromVec2(Math::Vec2(640.0f, 360.0f));
        return true;
    }
    // A Control never gets smaller than its minimum size, so it is given one well above it.
    if (property == "size" && NodeClassDB::isParentClass(className, "Control"))
    {
        out = PropertyValue::fromVec2(Math::Vec2(240.0f, 120.0f));
        return true;
    }
    return false;
}

bool makeValue(const ct::String& className, const PropertyInfo& info, const PropertyValue& current,
        PropertyValue& out)
{
    if (overrideValue(className, info.name, out)) return true;

    switch (info.type)
    {
        case PropertyType::Bool:
            out = PropertyValue::fromBool(!current.asBool());
            return true;
        case PropertyType::Int:
        {
            const int64_t base = current.asInt();
            if (info.hint == PropertyHint::Enum)
            {
                const size_t count = info.enumCount();
                if (count < 2) return false;
                out = PropertyValue::fromInt(
                        static_cast<int64_t>((static_cast<size_t>(base) + 1) % count));
                return true;
            }
            if (info.hint == PropertyHint::Layers3DRender ||
                    info.hint == PropertyHint::Layers3DPhysics || info.hint == PropertyHint::Flags)
            {
                out = PropertyValue::fromInt(base ^ 0x6);
                return true;
            }
            if (info.hint == PropertyHint::Range)
            {
                if (static_cast<double>(base + 1) <= info.rangeMax)
                    out = PropertyValue::fromInt(base + 1);
                else if (static_cast<double>(base - 1) >= info.rangeMin)
                    out = PropertyValue::fromInt(base - 1);
                else
                    return false;
                return true;
            }
            out = PropertyValue::fromInt(base + 3);
            return true;
        }
        case PropertyType::Float:
        {
            const double base = current.asFloat();
            double candidates[4] = { base + 1.5, 0.0, 0.0, 0.0 };
            if (info.hint == PropertyHint::Range)
            {
                const double quarter = quarterStep(info);
                candidates[0] = base + quarter;
                candidates[1] = base - quarter;
                candidates[2] = (info.rangeMin + info.rangeMax) * 0.5;
                candidates[3] = info.rangeMin + quarter;
            }
            else
            {
                candidates[1] = base - 1.5;
                candidates[2] = base + 3.0;
                candidates[3] = base - 3.0;
            }
            for (size_t i = 0; i < 4; ++i)
            {
                double value = candidates[i];
                if (info.hint == PropertyHint::Range)
                {
                    if (value < info.rangeMin && !info.rangeOrLess) continue;
                    if (value > info.rangeMax && !info.rangeOrGreater) continue;
                }
                value = static_cast<double>(static_cast<float>(value));
                if (value == base) continue;
                out = PropertyValue::fromFloat(value);
                return true;
            }
            return false;
        }
        case PropertyType::Vec2:
        {
            const Math::Vec2 base = current.asVec2();
            Math::Vec2 value(base.x + 0.5f, base.y - 0.25f);
            if (info.hint == PropertyHint::Range)
            {
                value.x = static_cast<float>(clampToRange(info, value.x));
                value.y = static_cast<float>(clampToRange(info, value.y));
            }
            out = PropertyValue::fromVec2(value);
            return out != current;
        }
        case PropertyType::Vec3:
        {
            const Math::Vec3 base = current.asVec3();
            Math::Vec3 value(base.x + 0.5f, base.y - 0.25f, base.z + 0.75f);
            if (info.hint == PropertyHint::Range)
            {
                value.x = static_cast<float>(clampToRange(info, value.x));
                value.y = static_cast<float>(clampToRange(info, value.y));
                value.z = static_cast<float>(clampToRange(info, value.z));
            }
            out = PropertyValue::fromVec3(value);
            return out != current;
        }
        case PropertyType::Vec4:
        {
            const Math::Vec4 base = current.asVec4();
            out = PropertyValue::fromVec4(
                    Math::Vec4(base.x + 0.5f, base.y - 0.25f, base.z + 0.75f, base.w + 0.125f));
            return true;
        }
        case PropertyType::Quat:
            out = PropertyValue::fromQuat(Math::Quaternion(0.5f, 0.5f, 0.5f, 0.5f));
            return out != current;
        case PropertyType::Color:
            out = withAlpha(Math::Vec4(0.25f, 0.5f, 0.75f, 0.5f),
                    info.hint == PropertyHint::ColorNoAlpha);
            return out != current;
        case PropertyType::String:
            out = PropertyValue::fromString("changed");
            return true;
        case PropertyType::NodePath:
            out = PropertyValue::fromNodePath("../Other/Node");
            return true;
        case PropertyType::ResourcePath:
            return false;
        case PropertyType::Nil:
            return false;
    }
    return false;
}


bool approximate(const ct::String& property)
{
    return property == "rotation_degrees" || property == "global_rotation_degrees" ||
           property == "global_rotation" || property == "shape_cone_angle" ||
           property == "emission_spread_angle";
}

} // namespace
