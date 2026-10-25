#pragma once

#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Animation.h"
#include "prisma3d/scene/PropertyValue.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/TerrainPlacement.h"

#include "mathc.h"

#include <ct/json.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

namespace scenejson
{

ct::Json fromFloat(float value);
ct::Json fromFloat(double value);
ct::Json fromVec2(const Math::Vec2& value);
ct::Json fromVec3(const Math::Vec3& value);
ct::Json fromVec4(const Math::Vec4& value);
ct::Json fromQuat(const Math::Quaternion& value);
ct::Json fromFloats(const float* values, std::size_t count);
ct::Json fromMat4(const Math::Mat4& value);
ct::Json fromValue(const PropertyValue& value);

bool toDouble(const ct::Json& json, double& out);
bool toFloat(const ct::Json& json, float& out);
bool toInt(const ct::Json& json, std::int64_t& out);
bool toBool(const ct::Json& json, bool& out);
bool toVec2(const ct::Json& json, Math::Vec2& out);
bool toVec3(const ct::Json& json, Math::Vec3& out);
bool toVec4(const ct::Json& json, Math::Vec4& out);
bool toQuat(const ct::Json& json, Math::Quaternion& out);
bool toFloats(const ct::Json& json, float* out, std::size_t count);
bool toFloatVector(const ct::Json& json, ct::Vector<float>& out);
bool toMat4(const ct::Json& json, Math::Mat4& out);
bool toValue(const ct::Json& json, PropertyType type, PropertyValue& out, ct::String& error);

float getFloat(const ct::Json& object, const char* key, float fallback);
double getDouble(const ct::Json& object, const char* key, double fallback);
std::int64_t getInt(const ct::Json& object, const char* key, std::int64_t fallback);
bool getBool(const ct::Json& object, const char* key, bool fallback);
ct::String getString(const ct::Json& object, const char* key, const ct::String& fallback);
bool getVec3(const ct::Json& object, const char* key, Math::Vec3& out);
bool getVec4(const ct::Json& object, const char* key, Math::Vec4& out);

ct::Json fromFloatBlock(const float* values, std::size_t count);
bool toFloatBlock(const ct::Json& json, ct::Vector<float>& out);
ct::Json fromU32Block(const std::uint32_t* values, std::size_t count);
bool toU32Block(const ct::Json& json, ct::Vector<std::uint32_t>& out);
ct::Json fromVec2Block(const ct::Vector<Math::Vec2>& values);
bool toVec2Block(const ct::Json& json, ct::Vector<Math::Vec2>& out);
ct::Json fromVec3Block(const ct::Vector<Math::Vec3>& values);
bool toVec3Block(const ct::Json& json, ct::Vector<Math::Vec3>& out);
ct::Json fromVec4Block(const ct::Vector<Math::Vec4>& values);
bool toVec4Block(const ct::Json& json, ct::Vector<Math::Vec4>& out);
ct::Json fromSurface(const SurfaceData& surface);
bool toSurface(const ct::Json& json, SurfaceData& out, SceneResult& result, const ct::String& path);

ct::Json fromExclusions(const TerrainPlacement& placement);
void toExclusions(const ct::Json& json, TerrainPlacement& placement, SceneResult& result,
        const ct::String& path);

ct::Json fromClip(const AnimationClip& clip);
bool toClip(const ct::Json& json, AnimationClip& out, SceneResult& result, const ct::String& path);

ct::String encodeBase64(const void* data, std::size_t size);
bool decodeBase64(const ct::String& text, ct::Vector<std::uint8_t>& out);
ct::Json fromBlock(const void* data, std::size_t size);
bool toBlock(const ct::Json& json, ct::Vector<std::uint8_t>& out);

inline bool exact(const Math::Vec2& a, const Math::Vec2& b)
{
    return a.x == b.x && a.y == b.y;
}

inline bool exact(const Math::Vec3& a, const Math::Vec3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool exact(const Math::Vec4& a, const Math::Vec4& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

inline bool exact(const Math::Quaternion& a, const Math::Quaternion& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

void fail(ct::String& error, const ct::String& message);

ct::String field(const ct::String& base, const char* name);
ct::String element(const ct::String& base, const char* name, std::size_t index);
ct::String item(const ct::String& base, std::size_t index);
ct::Json header(std::int64_t version);
bool checkVersion(const ct::Json& data, std::int64_t current, SceneResult& result,
        const ct::String& path);
const ct::Json* arrayField(const ct::Json& object, const char* key, SceneResult& result,
        const ct::String& path);
const ct::Json* objectField(const ct::Json& object, const char* key, SceneResult& result,
        const ct::String& path);
bool readFloatField(const ct::Json& object, const char* key, float& out, SceneResult& result,
        const ct::String& path);
bool readIntField(const ct::Json& object, const char* key, std::int64_t& out, SceneResult& result,
        const ct::String& path);
bool readBoolField(const ct::Json& object, const char* key, bool& out, SceneResult& result,
        const ct::String& path);
bool readVec3Field(const ct::Json& object, const char* key, Math::Vec3& out, SceneResult& result,
        const ct::String& path);
bool readVec4Field(const ct::Json& object, const char* key, Math::Vec4& out, SceneResult& result,
        const ct::String& path);
bool readStringField(const ct::Json& object, const char* key, ct::String& out, SceneResult& result,
        const ct::String& path);

} // namespace scenejson

} // namespace prisma3d
