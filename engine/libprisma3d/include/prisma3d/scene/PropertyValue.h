#pragma once

#include "mathc.h"

#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

enum class PropertyType : std::uint8_t
{
    Nil,
    Bool,
    Int,
    Float,
    Vec2,
    Vec3,
    Vec4,
    Quat,
    Color,
    String,
    NodePath,
    ResourcePath
};

class PropertyValue
{
public:
    PropertyValue()
            : mType(PropertyType::Nil),
              mText(),
              mInt(0),
              mSingle(false)
    {
        mNumbers[0] = mNumbers[1] = mNumbers[2] = mNumbers[3] = 0.0f;
    }

    static PropertyValue fromBool(bool value)
    {
        PropertyValue result;
        result.mType = PropertyType::Bool;
        result.mInt = value ? 1 : 0;
        return result;
    }

    static PropertyValue fromInt(std::int64_t value)
    {
        PropertyValue result;
        result.mType = PropertyType::Int;
        result.mInt = value;
        return result;
    }

    static PropertyValue fromFloat(double value)
    {
        PropertyValue result;
        result.mType = PropertyType::Float;
        result.mFloat = value;
        return result;
    }

    static PropertyValue fromSingle(float value)
    {
        PropertyValue result = fromFloat(static_cast<double>(value));
        result.mSingle = true;
        return result;
    }

    bool isSingle() const
    {
        return mSingle;
    }

    static PropertyValue fromVec2(const Math::Vec2& value)
    {
        return fromNumbers(PropertyType::Vec2, value.x, value.y, 0.0f, 0.0f);
    }

    static PropertyValue fromVec3(const Math::Vec3& value)
    {
        return fromNumbers(PropertyType::Vec3, value.x, value.y, value.z, 0.0f);
    }

    static PropertyValue fromVec4(const Math::Vec4& value)
    {
        return fromNumbers(PropertyType::Vec4, value.x, value.y, value.z, value.w);
    }

    static PropertyValue fromQuat(const Math::Quaternion& value)
    {
        return fromNumbers(PropertyType::Quat, value.x, value.y, value.z, value.w);
    }

    static PropertyValue fromColor(const Math::Vec4& value)
    {
        return fromNumbers(PropertyType::Color, value.x, value.y, value.z, value.w);
    }

    static PropertyValue fromString(const ct::String& value)
    {
        return fromText(PropertyType::String, value);
    }

    static PropertyValue fromNodePath(const ct::String& value)
    {
        return fromText(PropertyType::NodePath, value);
    }

    static PropertyValue fromResourcePath(const ct::String& value)
    {
        return fromText(PropertyType::ResourcePath, value);
    }

    PropertyType type() const
    {
        return mType;
    }

    bool isNil() const
    {
        return mType == PropertyType::Nil;
    }

    bool asBool() const
    {
        switch (mType)
        {
            case PropertyType::Bool:
            case PropertyType::Int:
                return mInt != 0;
            case PropertyType::Float:
                return mFloat != 0.0;
            default:
                return false;
        }
    }

    std::int64_t asInt() const
    {
        switch (mType)
        {
            case PropertyType::Bool:
            case PropertyType::Int:
                return mInt;
            case PropertyType::Float:
                return static_cast<std::int64_t>(mFloat);
            default:
                return 0;
        }
    }

    double asFloat() const
    {
        switch (mType)
        {
            case PropertyType::Bool:
            case PropertyType::Int:
                return static_cast<double>(mInt);
            case PropertyType::Float:
                return mFloat;
            default:
                return 0.0;
        }
    }

    Math::Vec2 asVec2() const
    {
        if (mType != PropertyType::Vec2) return Math::Vec2(0.0f, 0.0f);
        return Math::Vec2(mNumbers[0], mNumbers[1]);
    }

    Math::Vec3 asVec3() const
    {
        if (mType != PropertyType::Vec3) return Math::Vec3(0.0f, 0.0f, 0.0f);
        return Math::Vec3(mNumbers[0], mNumbers[1], mNumbers[2]);
    }

    Math::Vec4 asVec4() const
    {
        if (mType != PropertyType::Vec4) return Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f);
        return Math::Vec4(mNumbers[0], mNumbers[1], mNumbers[2], mNumbers[3]);
    }

    Math::Quaternion asQuat() const
    {
        if (mType != PropertyType::Quat) return Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
        return Math::Quaternion(mNumbers[0], mNumbers[1], mNumbers[2], mNumbers[3]);
    }

    Math::Vec4 asColor() const
    {
        if (mType != PropertyType::Color) return Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
        return Math::Vec4(mNumbers[0], mNumbers[1], mNumbers[2], mNumbers[3]);
    }

    ct::String asString() const
    {
        if (mType != PropertyType::String && mType != PropertyType::NodePath &&
                mType != PropertyType::ResourcePath)
            return ct::String();
        return mText;
    }

    static bool canConvert(PropertyType from, PropertyType to)
    {
        if (from == to || to == PropertyType::Nil) return true;
        switch (to)
        {
            case PropertyType::Bool:
                return from == PropertyType::Int || from == PropertyType::Float;
            case PropertyType::Int:
                return from == PropertyType::Bool || from == PropertyType::Float;
            case PropertyType::Float:
                return from == PropertyType::Bool || from == PropertyType::Int;
            case PropertyType::String:
                return from == PropertyType::NodePath || from == PropertyType::ResourcePath;
            case PropertyType::NodePath:
            case PropertyType::ResourcePath:
                return from == PropertyType::String;
            default:
                return false;
        }
    }

    bool convertTo(PropertyType to, PropertyValue& out) const
    {
        if (!canConvert(mType, to)) return false;
        if (mType == to || to == PropertyType::Nil)
        {
            out = *this;
            return true;
        }
        switch (to)
        {
            case PropertyType::Bool:
                out = fromBool(asBool());
                break;
            case PropertyType::Int:
                out = fromInt(asInt());
                break;
            case PropertyType::Float:
                out = fromFloat(asFloat());
                break;
            case PropertyType::String:
                out = fromString(mText);
                break;
            case PropertyType::NodePath:
                out = fromNodePath(mText);
                break;
            case PropertyType::ResourcePath:
                out = fromResourcePath(mText);
                break;
            default:
                return false;
        }
        return true;
    }

    bool operator==(const PropertyValue& other) const
    {
        if (mType != other.mType) return false;
        switch (mType)
        {
            case PropertyType::Nil:
                return true;
            case PropertyType::Bool:
            case PropertyType::Int:
                return mInt == other.mInt;
            case PropertyType::Float:
                return mFloat == other.mFloat;
            case PropertyType::Vec2:
                return mNumbers[0] == other.mNumbers[0] && mNumbers[1] == other.mNumbers[1];
            case PropertyType::Vec3:
                return mNumbers[0] == other.mNumbers[0] && mNumbers[1] == other.mNumbers[1] &&
                       mNumbers[2] == other.mNumbers[2];
            case PropertyType::Vec4:
            case PropertyType::Quat:
            case PropertyType::Color:
                return mNumbers[0] == other.mNumbers[0] && mNumbers[1] == other.mNumbers[1] &&
                       mNumbers[2] == other.mNumbers[2] && mNumbers[3] == other.mNumbers[3];
            case PropertyType::String:
            case PropertyType::NodePath:
            case PropertyType::ResourcePath:
                return mText == other.mText;
        }
        return false;
    }

    bool operator!=(const PropertyValue& other) const
    {
        return !(*this == other);
    }

    static const char* typeName(PropertyType type)
    {
        switch (type)
        {
            case PropertyType::Nil:
                return "Nil";
            case PropertyType::Bool:
                return "Bool";
            case PropertyType::Int:
                return "Int";
            case PropertyType::Float:
                return "Float";
            case PropertyType::Vec2:
                return "Vec2";
            case PropertyType::Vec3:
                return "Vec3";
            case PropertyType::Vec4:
                return "Vec4";
            case PropertyType::Quat:
                return "Quat";
            case PropertyType::Color:
                return "Color";
            case PropertyType::String:
                return "String";
            case PropertyType::NodePath:
                return "NodePath";
            case PropertyType::ResourcePath:
                return "ResourcePath";
        }
        return "Nil";
    }

private:
    static PropertyValue fromNumbers(PropertyType type, float a, float b, float c, float d)
    {
        PropertyValue result;
        result.mType = type;
        result.mNumbers[0] = a;
        result.mNumbers[1] = b;
        result.mNumbers[2] = c;
        result.mNumbers[3] = d;
        return result;
    }

    static PropertyValue fromText(PropertyType type, const ct::String& text)
    {
        PropertyValue result;
        result.mType = type;
        result.mText = text;
        return result;
    }

    PropertyType mType;
    ct::String mText;
    union
    {
        std::int64_t mInt;
        double mFloat;
    };
    float mNumbers[4];
    bool mSingle;
};

} // namespace prisma3d
