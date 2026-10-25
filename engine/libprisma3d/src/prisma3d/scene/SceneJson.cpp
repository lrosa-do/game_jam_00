#include "prisma3d/scene/SceneJson.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace prisma3d
{

namespace scenejson
{

namespace
{

const char kBase64Alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int base64Value(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

} // namespace

ct::Json fromFloat(float value)
{
    if (value != value) return ct::Json("nan");
    if (value == INFINITY) return ct::Json("inf");
    if (value == -INFINITY) return ct::Json("-inf");

    char buffer[48];
    for (int precision = 1; precision <= 9; ++precision)
    {
        snprintf(buffer, sizeof(buffer), "%.*g", precision, static_cast<double>(value));
        if (strtof(buffer, nullptr) == value) break;
    }
    return ct::Json(strtod(buffer, nullptr));
}

ct::Json fromFloat(double value)
{
    if (value != value) return ct::Json("nan");
    if (value == INFINITY) return ct::Json("inf");
    if (value == -INFINITY) return ct::Json("-inf");
    return ct::Json(value);
}

ct::Json fromVec2(const Math::Vec2& value)
{
    ct::Json array = ct::Json::array();
    array.push_back(fromFloat(value.x));
    array.push_back(fromFloat(value.y));
    return array;
}

ct::Json fromVec3(const Math::Vec3& value)
{
    ct::Json array = ct::Json::array();
    array.push_back(fromFloat(value.x));
    array.push_back(fromFloat(value.y));
    array.push_back(fromFloat(value.z));
    return array;
}

ct::Json fromVec4(const Math::Vec4& value)
{
    ct::Json array = ct::Json::array();
    array.push_back(fromFloat(value.x));
    array.push_back(fromFloat(value.y));
    array.push_back(fromFloat(value.z));
    array.push_back(fromFloat(value.w));
    return array;
}

ct::Json fromQuat(const Math::Quaternion& value)
{
    ct::Json array = ct::Json::array();
    array.push_back(fromFloat(value.x));
    array.push_back(fromFloat(value.y));
    array.push_back(fromFloat(value.z));
    array.push_back(fromFloat(value.w));
    return array;
}

ct::Json fromFloats(const float* values, std::size_t count)
{
    ct::Json array = ct::Json::array();
    array.reserve(count);
    for (std::size_t i = 0; i < count; ++i) array.push_back(fromFloat(values[i]));
    return array;
}

ct::Json fromMat4(const Math::Mat4& value)
{
    return fromFloats(value.Data(), 16);
}

ct::Json fromValue(const PropertyValue& value)
{
    switch (value.type())
    {
        case PropertyType::Nil:
            return ct::Json();
        case PropertyType::Bool:
            return ct::Json(value.asBool());
        case PropertyType::Int:
            return ct::Json(static_cast<long long>(value.asInt()));
        case PropertyType::Float:
            if (value.isSingle()) return fromFloat(static_cast<float>(value.asFloat()));
            return fromFloat(value.asFloat());
        case PropertyType::Vec2:
            return fromVec2(value.asVec2());
        case PropertyType::Vec3:
            return fromVec3(value.asVec3());
        case PropertyType::Vec4:
            return fromVec4(value.asVec4());
        case PropertyType::Quat:
            return fromQuat(value.asQuat());
        case PropertyType::Color:
            return fromVec4(value.asColor());
        case PropertyType::String:
        case PropertyType::NodePath:
        case PropertyType::ResourcePath:
            return ct::Json(value.asString());
    }
    return ct::Json();
}

bool toDouble(const ct::Json& json, double& out)
{
    if (json.is_number())
    {
        out = json.as_double();
        return true;
    }
    if (json.is_string())
    {
        const ct::String& text = json.str();
        if (text == "nan")
        {
            out = NAN;
            return true;
        }
        if (text == "inf")
        {
            out = INFINITY;
            return true;
        }
        if (text == "-inf")
        {
            out = -INFINITY;
            return true;
        }
    }
    return false;
}

bool toFloat(const ct::Json& json, float& out)
{
    double value = 0.0;
    if (!toDouble(json, value)) return false;
    out = static_cast<float>(value);
    return true;
}

bool toInt(const ct::Json& json, std::int64_t& out)
{
    if (json.is_int())
    {
        out = json.as_int();
        return true;
    }
    if (json.is_uint())
    {
        const std::uint64_t value = json.as_uint();
        if (value > static_cast<std::uint64_t>(INT64_MAX)) return false;
        out = static_cast<std::int64_t>(value);
        return true;
    }
    if (json.is_real())
    {
        const double value = json.as_double();
        if (!(value >= -9007199254740992.0 && value <= 9007199254740992.0)) return false;
        if (value != floor(value)) return false;
        out = static_cast<std::int64_t>(value);
        return true;
    }
    return false;
}

bool toBool(const ct::Json& json, bool& out)
{
    if (json.is_bool())
    {
        out = json.as_bool();
        return true;
    }
    std::int64_t number = 0;
    if (json.is_number() && toInt(json, number))
    {
        out = number != 0;
        return true;
    }
    return false;
}

bool toFloats(const ct::Json& json, float* out, std::size_t count)
{
    if (!json.is_array() || json.size() != count) return false;
    for (std::size_t i = 0; i < count; ++i)
        if (!toFloat(json.at(i), out[i])) return false;
    return true;
}

bool toVec2(const ct::Json& json, Math::Vec2& out)
{
    float values[2];
    if (!toFloats(json, values, 2)) return false;
    out = Math::Vec2(values[0], values[1]);
    return true;
}

bool toVec3(const ct::Json& json, Math::Vec3& out)
{
    float values[3];
    if (!toFloats(json, values, 3)) return false;
    out = Math::Vec3(values[0], values[1], values[2]);
    return true;
}

bool toVec4(const ct::Json& json, Math::Vec4& out)
{
    float values[4];
    if (!toFloats(json, values, 4)) return false;
    out = Math::Vec4(values[0], values[1], values[2], values[3]);
    return true;
}

bool toQuat(const ct::Json& json, Math::Quaternion& out)
{
    float values[4];
    if (!toFloats(json, values, 4)) return false;
    out = Math::Quaternion(values[0], values[1], values[2], values[3]);
    return true;
}

bool toFloatVector(const ct::Json& json, ct::Vector<float>& out)
{
    if (!json.is_array()) return false;
    out.clear();
    out.reserve(json.size());
    for (std::size_t i = 0; i < json.size(); ++i)
    {
        float value = 0.0f;
        if (!toFloat(json.at(i), value)) return false;
        out.push_back(value);
    }
    return true;
}

bool toMat4(const ct::Json& json, Math::Mat4& out)
{
    float values[16];
    if (!toFloats(json, values, 16)) return false;
    memcpy(out.Data(), values, sizeof(values));
    return true;
}

bool toValue(const ct::Json& json, PropertyType type, PropertyValue& out, ct::String& error)
{
    switch (type)
    {
        case PropertyType::Nil:
            fail(error, "the property has no type");
            return false;
        case PropertyType::Bool:
        {
            bool value = false;
            if (!toBool(json, value))
            {
                fail(error, "expected true or false");
                return false;
            }
            out = PropertyValue::fromBool(value);
            return true;
        }
        case PropertyType::Int:
        {
            std::int64_t value = 0;
            if (!toInt(json, value))
            {
                fail(error, "expected a whole number");
                return false;
            }
            out = PropertyValue::fromInt(value);
            return true;
        }
        case PropertyType::Float:
        {
            double value = 0.0;
            if (!toDouble(json, value))
            {
                fail(error, "expected a number");
                return false;
            }
            out = PropertyValue::fromFloat(value);
            return true;
        }
        case PropertyType::Vec2:
        {
            Math::Vec2 value;
            if (!toVec2(json, value))
            {
                fail(error, "expected [x, y] of numbers");
                return false;
            }
            out = PropertyValue::fromVec2(value);
            return true;
        }
        case PropertyType::Vec3:
        {
            Math::Vec3 value;
            if (!toVec3(json, value))
            {
                fail(error, "expected [x, y, z] of numbers");
                return false;
            }
            out = PropertyValue::fromVec3(value);
            return true;
        }
        case PropertyType::Vec4:
        {
            Math::Vec4 value;
            if (!toVec4(json, value))
            {
                fail(error, "expected [x, y, z, w] of numbers");
                return false;
            }
            out = PropertyValue::fromVec4(value);
            return true;
        }
        case PropertyType::Quat:
        {
            Math::Quaternion value;
            if (!toQuat(json, value))
            {
                fail(error, "expected [x, y, z, w] of numbers");
                return false;
            }
            out = PropertyValue::fromQuat(value);
            return true;
        }
        case PropertyType::Color:
        {
            Math::Vec4 value;
            if (!toVec4(json, value))
            {
                fail(error, "expected [r, g, b, a] of numbers");
                return false;
            }
            out = PropertyValue::fromColor(value);
            return true;
        }
        case PropertyType::String:
        case PropertyType::NodePath:
        case PropertyType::ResourcePath:
        {
            if (!json.is_string())
            {
                fail(error, "expected a text");
                return false;
            }
            if (type == PropertyType::String) out = PropertyValue::fromString(json.str());
            else if (type == PropertyType::NodePath)
                out = PropertyValue::fromNodePath(json.str());
            else
                out = PropertyValue::fromResourcePath(json.str());
            return true;
        }
    }
    return false;
}

float getFloat(const ct::Json& object, const char* key, float fallback)
{
    const ct::Json* field = object.find(key);
    float value = 0.0f;
    if (field && toFloat(*field, value)) return value;
    return fallback;
}

double getDouble(const ct::Json& object, const char* key, double fallback)
{
    const ct::Json* field = object.find(key);
    double value = 0.0;
    if (field && toDouble(*field, value)) return value;
    return fallback;
}

std::int64_t getInt(const ct::Json& object, const char* key, std::int64_t fallback)
{
    const ct::Json* field = object.find(key);
    std::int64_t value = 0;
    if (field && toInt(*field, value)) return value;
    return fallback;
}

bool getBool(const ct::Json& object, const char* key, bool fallback)
{
    const ct::Json* field = object.find(key);
    bool value = false;
    if (field && toBool(*field, value)) return value;
    return fallback;
}

ct::String getString(const ct::Json& object, const char* key, const ct::String& fallback)
{
    const ct::Json* field = object.find(key);
    if (field && field->is_string()) return field->str();
    return fallback;
}

bool getVec3(const ct::Json& object, const char* key, Math::Vec3& out)
{
    const ct::Json* field = object.find(key);
    return field && toVec3(*field, out);
}

bool getVec4(const ct::Json& object, const char* key, Math::Vec4& out)
{
    const ct::Json* field = object.find(key);
    return field && toVec4(*field, out);
}

ct::String encodeBase64(const void* data, std::size_t size)
{
    const std::uint8_t* bytes = static_cast<const std::uint8_t*>(data);
    ct::String text;
    text.reserve((size + 2) / 3 * 4);
    std::size_t i = 0;
    for (; i + 2 < size; i += 3)
    {
        const std::uint32_t group = (static_cast<std::uint32_t>(bytes[i]) << 16) |
                                    (static_cast<std::uint32_t>(bytes[i + 1]) << 8) | bytes[i + 2];
        text.push_back(kBase64Alphabet[(group >> 18) & 63]);
        text.push_back(kBase64Alphabet[(group >> 12) & 63]);
        text.push_back(kBase64Alphabet[(group >> 6) & 63]);
        text.push_back(kBase64Alphabet[group & 63]);
    }
    if (i + 1 == size)
    {
        const std::uint32_t group = static_cast<std::uint32_t>(bytes[i]) << 16;
        text.push_back(kBase64Alphabet[(group >> 18) & 63]);
        text.push_back(kBase64Alphabet[(group >> 12) & 63]);
        text.push_back('=');
        text.push_back('=');
    }
    else if (i + 2 == size)
    {
        const std::uint32_t group = (static_cast<std::uint32_t>(bytes[i]) << 16) |
                                    (static_cast<std::uint32_t>(bytes[i + 1]) << 8);
        text.push_back(kBase64Alphabet[(group >> 18) & 63]);
        text.push_back(kBase64Alphabet[(group >> 12) & 63]);
        text.push_back(kBase64Alphabet[(group >> 6) & 63]);
        text.push_back('=');
    }
    return text;
}

bool decodeBase64(const ct::String& text, ct::Vector<std::uint8_t>& out)
{
    out.clear();
    if (text.size() % 4 != 0) return false;
    out.reserve(text.size() / 4 * 3);
    for (std::size_t i = 0; i < text.size(); i += 4)
    {
        const bool last = i + 4 == text.size();
        int values[4];
        int padding = 0;
        for (int j = 0; j < 4; ++j)
        {
            const char c = text[i + static_cast<std::size_t>(j)];
            if (c == '=')
            {
                if (!last || j < 2) return false;
                values[j] = 0;
                ++padding;
            }
            else
            {
                if (padding) return false;
                values[j] = base64Value(c);
                if (values[j] < 0) return false;
            }
        }
        const std::uint32_t group = (static_cast<std::uint32_t>(values[0]) << 18) |
                                    (static_cast<std::uint32_t>(values[1]) << 12) |
                                    (static_cast<std::uint32_t>(values[2]) << 6) |
                                    static_cast<std::uint32_t>(values[3]);
        out.push_back(static_cast<std::uint8_t>(group >> 16));
        if (padding < 2) out.push_back(static_cast<std::uint8_t>(group >> 8));
        if (padding < 1) out.push_back(static_cast<std::uint8_t>(group));
    }
    return true;
}

ct::Json fromBlock(const void* data, std::size_t size)
{
    return ct::Json(encodeBase64(data, size));
}

bool toBlock(const ct::Json& json, ct::Vector<std::uint8_t>& out)
{
    return json.is_string() && decodeBase64(json.str(), out);
}

ct::Json fromFloatBlock(const float* values, std::size_t count)
{
    return fromBlock(values, count * sizeof(float));
}

bool toFloatBlock(const ct::Json& json, ct::Vector<float>& out)
{
    ct::Vector<std::uint8_t> bytes;
    if (!toBlock(json, bytes) || bytes.size() % sizeof(float) != 0) return false;
    out.resize(bytes.size() / sizeof(float));
    if (!bytes.empty()) memcpy(out.data(), bytes.data(), bytes.size());
    return true;
}

ct::Json fromU32Block(const std::uint32_t* values, std::size_t count)
{
    return fromBlock(values, count * sizeof(std::uint32_t));
}

bool toU32Block(const ct::Json& json, ct::Vector<std::uint32_t>& out)
{
    ct::Vector<std::uint8_t> bytes;
    if (!toBlock(json, bytes) || bytes.size() % sizeof(std::uint32_t) != 0) return false;
    out.resize(bytes.size() / sizeof(std::uint32_t));
    if (!bytes.empty()) memcpy(out.data(), bytes.data(), bytes.size());
    return true;
}

static_assert(sizeof(Math::Vec2) == 2 * sizeof(float), "Vec2 must be packed");
static_assert(sizeof(Math::Vec3) == 3 * sizeof(float), "Vec3 must be packed");
static_assert(sizeof(Math::Vec4) == 4 * sizeof(float), "Vec4 must be packed");

ct::Json fromVec2Block(const ct::Vector<Math::Vec2>& values)
{
    return fromBlock(values.data(), values.size() * sizeof(Math::Vec2));
}

bool toVec2Block(const ct::Json& json, ct::Vector<Math::Vec2>& out)
{
    ct::Vector<std::uint8_t> bytes;
    if (!toBlock(json, bytes) || bytes.size() % sizeof(Math::Vec2) != 0) return false;
    out.resize(bytes.size() / sizeof(Math::Vec2));
    if (!bytes.empty()) memcpy(out.data(), bytes.data(), bytes.size());
    return true;
}

ct::Json fromVec3Block(const ct::Vector<Math::Vec3>& values)
{
    return fromBlock(values.data(), values.size() * sizeof(Math::Vec3));
}

bool toVec3Block(const ct::Json& json, ct::Vector<Math::Vec3>& out)
{
    ct::Vector<std::uint8_t> bytes;
    if (!toBlock(json, bytes) || bytes.size() % sizeof(Math::Vec3) != 0) return false;
    out.resize(bytes.size() / sizeof(Math::Vec3));
    if (!bytes.empty()) memcpy(out.data(), bytes.data(), bytes.size());
    return true;
}

ct::Json fromVec4Block(const ct::Vector<Math::Vec4>& values)
{
    return fromBlock(values.data(), values.size() * sizeof(Math::Vec4));
}

bool toVec4Block(const ct::Json& json, ct::Vector<Math::Vec4>& out)
{
    ct::Vector<std::uint8_t> bytes;
    if (!toBlock(json, bytes) || bytes.size() % sizeof(Math::Vec4) != 0) return false;
    out.resize(bytes.size() / sizeof(Math::Vec4));
    if (!bytes.empty()) memcpy(out.data(), bytes.data(), bytes.size());
    return true;
}

ct::Json fromSurface(const SurfaceData& surface)
{
    ct::Json json = ct::Json::object();
    json["topology"] = ct::Json(static_cast<long long>(surface.topology));
    json["positions"] = fromVec3Block(surface.positions);
    if (!surface.normals.empty()) json["normals"] = fromVec3Block(surface.normals);
    if (!surface.tangents.empty()) json["tangents"] = fromVec4Block(surface.tangents);
    if (!surface.colors.empty()) json["colors"] = fromVec4Block(surface.colors);
    if (!surface.uvs.empty()) json["uvs"] = fromVec2Block(surface.uvs);
    if (!surface.uvs2.empty()) json["uvs2"] = fromVec2Block(surface.uvs2);
    if (!surface.joints.empty())
        json["joints"] = fromU32Block(surface.joints.data(), surface.joints.size());
    if (!surface.weights.empty()) json["weights"] = fromVec4Block(surface.weights);
    if (!surface.indices.empty())
        json["indices"] = fromU32Block(surface.indices.data(), surface.indices.size());
    return json;
}

namespace
{

template<typename T, typename Reader>
bool readAttribute(const ct::Json& json, const char* key, ct::Vector<T>& out, std::size_t vertices,
        Reader reader, SceneResult& result, const ct::String& path)
{
    const ct::Json* field = json.find(key);
    if (!field) return true;
    if (!reader(*field, out))
    {
        result.addError(scenejson::field(path, key), "not a valid block of numbers");
        return false;
    }
    if (out.size() != vertices)
    {
        result.addError(scenejson::field(path, key), "the count differs from the positions");
        return false;
    }
    return true;
}

} // namespace

bool toSurface(const ct::Json& json, SurfaceData& out, SceneResult& result, const ct::String& path)
{
    if (!json.is_object())
    {
        result.addError(path, "expected an object");
        return false;
    }
    SurfaceData surface;
    std::int64_t topology = 0;
    if (json.contains("topology"))
    {
        if (!readIntField(json, "topology", topology, result, path)) return false;
        if (topology < 0 || topology > static_cast<std::int64_t>(prisma::Topology::Patches))
        {
            result.addError(field(path, "topology"), "not a known topology");
            return false;
        }
        surface.topology = static_cast<prisma::Topology>(topology);
    }
    const ct::Json* positions = json.find("positions");
    if (!positions || !toVec3Block(*positions, surface.positions))
    {
        result.addError(field(path, "positions"), "missing or not a valid block of numbers");
        return false;
    }
    const std::size_t count = surface.positions.size();
    if (!readAttribute(json, "normals", surface.normals, count, toVec3Block, result, path) ||
            !readAttribute(json, "tangents", surface.tangents, count, toVec4Block, result, path) ||
            !readAttribute(json, "colors", surface.colors, count, toVec4Block, result, path) ||
            !readAttribute(json, "uvs", surface.uvs, count, toVec2Block, result, path) ||
            !readAttribute(json, "uvs2", surface.uvs2, count, toVec2Block, result, path) ||
            !readAttribute(json, "joints", surface.joints, count, toU32Block, result, path) ||
            !readAttribute(json, "weights", surface.weights, count, toVec4Block, result, path))
        return false;
    if (const ct::Json* indices = json.find("indices"))
    {
        if (!toU32Block(*indices, surface.indices))
        {
            result.addError(field(path, "indices"), "not a valid block of numbers");
            return false;
        }
        for (std::size_t i = 0; i < surface.indices.size(); ++i)
        {
            if (surface.indices[i] >= count)
            {
                result.addError(field(path, "indices"), "an index is past the last vertex");
                return false;
            }
        }
    }
    out = surface;
    return true;
}

namespace
{

ct::Json stringArray(const ct::Vector<ct::String>& values)
{
    ct::Json array = ct::Json::array();
    for (std::size_t i = 0; i < values.size(); ++i) array.push_back(ct::Json(values[i]));
    return array;
}

bool readStringArray(const ct::Json& json, const char* key, ct::Vector<ct::String>& out,
        std::size_t count)
{
    const ct::Json* field = json.find(key);
    if (!field || !field->is_array() || field->size() != count) return false;
    for (std::size_t i = 0; i < count; ++i)
    {
        if (!field->at(i).is_string()) return false;
        out.push_back(field->at(i).str());
    }
    return true;
}

bool readTimes(const ct::Json& json, const char* key, ct::Vector<float>& out)
{
    const ct::Json* field = json.find(key);
    return field && toFloatBlock(*field, out);
}

bool readVec3Keys(const ct::Json& json, const char* key, ct::Vector<Math::Vec3>& out)
{
    const ct::Json* field = json.find(key);
    return field && toVec3Block(*field, out);
}

bool readQuatKeys(const ct::Json& json, const char* key, ct::Vector<Math::Quaternion>& out)
{
    const ct::Json* field = json.find(key);
    ct::Vector<Math::Vec4> raw;
    if (!field || !toVec4Block(*field, raw)) return false;
    for (std::size_t i = 0; i < raw.size(); ++i)
        out.push_back(Math::Quaternion(raw[i].x, raw[i].y, raw[i].z, raw[i].w));
    return true;
}

ct::Json quatBlock(const ct::Vector<Math::Quaternion>& values)
{
    ct::Vector<Math::Vec4> raw;
    raw.reserve(values.size());
    for (std::size_t i = 0; i < values.size(); ++i)
        raw.push_back(Math::Vec4(values[i].x, values[i].y, values[i].z, values[i].w));
    return fromVec4Block(raw);
}

} // namespace

ct::Json fromExclusions(const TerrainPlacement& placement)
{
    ct::Json list = ct::Json::array();
    for (int i = 0; i < placement.getExclusionCount(); ++i)
    {
        TerrainPath terrainPath;
        float margin = 0.0f;
        if (!placement.getExclusion(i, terrainPath, margin)) continue;
        ct::Json entry = ct::Json::object();
        entry["margin"] = fromFloat(margin);
        entry["closed"] = ct::Json(terrainPath.closed);
        entry["points"] = fromVec3Block(terrainPath.points);
        entry["half_widths"] =
                fromFloatBlock(terrainPath.halfWidths.data(), terrainPath.halfWidths.size());
        list.push_back(entry);
    }
    return list;
}

void toExclusions(const ct::Json& json, TerrainPlacement& placement, SceneResult& result,
        const ct::String& path)
{
    placement.clearExclusions();
    if (!json.is_array())
    {
        result.addError(path, "must be an array");
        return;
    }
    for (std::size_t i = 0; i < json.size(); ++i)
    {
        const ct::Json& entry = json.at(i);
        const ct::String entryPath = item(path, i);
        TerrainPath terrainPath;
        const ct::Json* points = entry.is_object() ? entry.find("points") : nullptr;
        const ct::Json* widths = entry.is_object() ? entry.find("half_widths") : nullptr;
        if (!points || !widths || !toVec3Block(*points, terrainPath.points) ||
                !toFloatBlock(*widths, terrainPath.halfWidths) ||
                terrainPath.points.size() != terrainPath.halfWidths.size())
        {
            result.addError(entryPath, "needs points and one half width for each point");
            continue;
        }
        terrainPath.closed = getBool(entry, "closed", false);
        placement.addExclusion(terrainPath, getFloat(entry, "margin", 0.0f));
    }
}

ct::Json fromClip(const AnimationClip& clip)
{
    ct::Json json = ct::Json::object();
    json["name"] = ct::Json(clip.name());
    json["duration"] = fromFloat(clip.duration());
    json["skeleton"] = ct::Json(static_cast<long long>(clip.skeleton()));
    json["loop_mode"] = ct::Json(static_cast<long long>(clip.loopMode()));

    ct::Json bones = ct::Json::array();
    for (std::size_t i = 0; i < clip.tracks().size(); ++i)
    {
        const BoneTrack& track = clip.tracks()[i];
        ct::Json entry = ct::Json::object();
        entry["bone"] = ct::Json(static_cast<long long>(track.bone));
        entry["times"] = fromFloatBlock(track.times.data(), track.times.size());
        entry["positions"] = fromVec3Block(track.positions);
        entry["rotations"] = quatBlock(track.rotations);
        entry["scales"] = fromVec3Block(track.scales);
        bones.push_back(entry);
    }
    if (bones.size() > 0) json["bone_tracks"] = bones;

    ct::Json nodes = ct::Json::array();
    for (std::size_t i = 0; i < clip.nodeTracks().size(); ++i)
    {
        const NodeTrack& track = clip.nodeTracks()[i];
        ct::Json entry = ct::Json::object();
        entry["path"] = ct::Json(track.path);
        entry["position_times"] =
                fromFloatBlock(track.positionTimes.data(), track.positionTimes.size());
        entry["positions"] = fromVec3Block(track.positions);
        entry["rotation_times"] =
                fromFloatBlock(track.rotationTimes.data(), track.rotationTimes.size());
        entry["rotations"] = quatBlock(track.rotations);
        entry["scale_times"] = fromFloatBlock(track.scaleTimes.data(), track.scaleTimes.size());
        entry["scales"] = fromVec3Block(track.scales);
        nodes.push_back(entry);
    }
    if (nodes.size() > 0) json["node_tracks"] = nodes;

    ct::Json methods = ct::Json::array();
    for (std::size_t i = 0; i < clip.methodTracks().size(); ++i)
    {
        const MethodTrack& track = clip.methodTracks()[i];
        ct::Json entry = ct::Json::object();
        entry["path"] = ct::Json(track.path);
        entry["times"] = fromFloatBlock(track.times.data(), track.times.size());
        entry["methods"] = stringArray(track.methods);
        methods.push_back(entry);
    }
    if (methods.size() > 0) json["method_tracks"] = methods;

    ct::Json values = ct::Json::array();
    for (std::size_t i = 0; i < clip.valueTracks().size(); ++i)
    {
        const ValueTrack& track = clip.valueTracks()[i];
        ct::Json entry = ct::Json::object();
        entry["path"] = ct::Json(track.path);
        entry["property"] = ct::Json(track.property);
        entry["times"] = fromFloatBlock(track.times.data(), track.times.size());
        entry["values"] = fromFloatBlock(track.values.data(), track.values.size());
        values.push_back(entry);
    }
    if (values.size() > 0) json["value_tracks"] = values;

    ct::Json animations = ct::Json::array();
    for (std::size_t i = 0; i < clip.animationTracks().size(); ++i)
    {
        const AnimationTrack& track = clip.animationTracks()[i];
        ct::Json entry = ct::Json::object();
        entry["path"] = ct::Json(track.path);
        entry["times"] = fromFloatBlock(track.times.data(), track.times.size());
        entry["animations"] = stringArray(track.animations);
        animations.push_back(entry);
    }
    if (animations.size() > 0) json["animation_tracks"] = animations;

    ct::Json events = ct::Json::array();
    for (std::size_t i = 0; i < clip.events().size(); ++i)
    {
        ct::Json entry = ct::Json::object();
        entry["time"] = fromFloat(clip.events()[i].time);
        entry["name"] = ct::Json(clip.events()[i].name);
        events.push_back(entry);
    }
    if (events.size() > 0) json["events"] = events;
    return json;
}

bool toClip(const ct::Json& json, AnimationClip& out, SceneResult& result, const ct::String& path)
{
    if (!json.is_object())
    {
        result.addError(path, "expected an object");
        return false;
    }
    AnimationClip clip;
    clip.setName(getString(json, "name", ct::String()));
    clip.setDuration(getFloat(json, "duration", 0.0f));
    clip.setSkeleton(static_cast<int>(getInt(json, "skeleton", 0)));
    const std::int64_t loop = getInt(json, "loop_mode", 0);
    if (loop < 0 || loop > static_cast<std::int64_t>(LoopMode::PingPong))
    {
        result.addError(field(path, "loop_mode"), "not a known loop mode");
        return false;
    }
    clip.setLoopMode(static_cast<LoopMode>(loop));

    if (const ct::Json* bones = arrayField(json, "bone_tracks", result, path))
    {
        for (std::size_t i = 0; i < bones->size(); ++i)
        {
            const ct::Json& entry = bones->at(i);
            BoneTrack track;
            track.bone = static_cast<int>(getInt(entry, "bone", -1));
            if (!entry.is_object() || !readTimes(entry, "times", track.times) ||
                    !readVec3Keys(entry, "positions", track.positions) ||
                    !readQuatKeys(entry, "rotations", track.rotations) ||
                    !readVec3Keys(entry, "scales", track.scales))
            {
                result.addError(element(path, "bone_tracks", i),
                        "the times and the keys must be blocks of numbers");
                return false;
            }
            clip.tracks().push_back(track);
        }
    }
    if (const ct::Json* nodes = arrayField(json, "node_tracks", result, path))
    {
        for (std::size_t i = 0; i < nodes->size(); ++i)
        {
            const ct::Json& entry = nodes->at(i);
            NodeTrack track;
            if (!entry.is_object() || !readTimes(entry, "position_times", track.positionTimes) ||
                    !readVec3Keys(entry, "positions", track.positions) ||
                    !readTimes(entry, "rotation_times", track.rotationTimes) ||
                    !readQuatKeys(entry, "rotations", track.rotations) ||
                    !readTimes(entry, "scale_times", track.scaleTimes) ||
                    !readVec3Keys(entry, "scales", track.scales))
            {
                result.addError(element(path, "node_tracks", i),
                        "the times and the keys must be blocks of numbers");
                return false;
            }
            track.path = getString(entry, "path", ct::String());
            clip.nodeTracks().push_back(track);
        }
    }
    if (const ct::Json* methods = arrayField(json, "method_tracks", result, path))
    {
        for (std::size_t i = 0; i < methods->size(); ++i)
        {
            const ct::Json& entry = methods->at(i);
            MethodTrack track;
            if (!entry.is_object() || !readTimes(entry, "times", track.times) ||
                    !readStringArray(entry, "methods", track.methods, track.times.size()))
            {
                result.addError(element(path, "method_tracks", i),
                        "the times and the method names must have the same length");
                return false;
            }
            track.path = getString(entry, "path", ct::String());
            clip.methodTracks().push_back(track);
        }
    }
    if (const ct::Json* values = arrayField(json, "value_tracks", result, path))
    {
        for (std::size_t i = 0; i < values->size(); ++i)
        {
            const ct::Json& entry = values->at(i);
            ValueTrack track;
            const ct::Json* valuesField = entry.is_object() ? entry.find("values") : nullptr;
            if (!entry.is_object() || !readTimes(entry, "times", track.times) || !valuesField ||
                    !toFloatBlock(*valuesField, track.values) ||
                    track.values.size() != track.times.size())
            {
                result.addError(element(path, "value_tracks", i),
                        "the times and the values must have the same length");
                return false;
            }
            track.path = getString(entry, "path", ct::String());
            track.property = getString(entry, "property", ct::String());
            clip.valueTracks().push_back(track);
        }
    }
    if (const ct::Json* animations = arrayField(json, "animation_tracks", result, path))
    {
        for (std::size_t i = 0; i < animations->size(); ++i)
        {
            const ct::Json& entry = animations->at(i);
            AnimationTrack track;
            if (!entry.is_object() || !readTimes(entry, "times", track.times) ||
                    !readStringArray(entry, "animations", track.animations, track.times.size()))
            {
                result.addError(element(path, "animation_tracks", i),
                        "the times and the animation names must have the same length");
                return false;
            }
            track.path = getString(entry, "path", ct::String());
            clip.animationTracks().push_back(track);
        }
    }
    if (const ct::Json* events = arrayField(json, "events", result, path))
    {
        for (std::size_t i = 0; i < events->size(); ++i)
        {
            const ct::Json& entry = events->at(i);
            if (!entry.is_object())
            {
                result.addError(element(path, "events", i), "expected an object");
                return false;
            }
            clip.addEvent(getFloat(entry, "time", 0.0f), getString(entry, "name", ct::String()));
        }
    }
    out = clip;
    return true;
}

void fail(ct::String& error, const ct::String& message)
{
    if (error.size() > 0) error += "; ";
    error += message;
}

ct::String field(const ct::String& base, const char* name)
{
    ct::String path = base;
    path += '.';
    path += name;
    return path;
}

ct::String element(const ct::String& base, const char* name, std::size_t index)
{
    ct::String path = field(base, name);
    path += '[';
    path.append_number(static_cast<unsigned long long>(index));
    path += ']';
    return path;
}

ct::String item(const ct::String& base, std::size_t index)
{
    ct::String path = base;
    path += '[';
    path.append_number(static_cast<unsigned long long>(index));
    path += ']';
    return path;
}

ct::Json header(std::int64_t version)
{
    ct::Json object = ct::Json::object();
    object["version"] = ct::Json(static_cast<long long>(version));
    return object;
}

bool checkVersion(const ct::Json& data, std::int64_t current, SceneResult& result,
        const ct::String& path)
{
    const ct::Json* version = data.find("version");
    if (!version) return true;
    std::int64_t number = 0;
    if (!toInt(*version, number) || number <= 0)
    {
        result.addError(field(path, "version"), "not a positive whole number");
        return false;
    }
    if (number > current)
    {
        ct::String message = "the data is version ";
        message.append_number(static_cast<long long>(number));
        message += ", newer than this build's ";
        message.append_number(static_cast<long long>(current));
        result.addError(field(path, "version"), message);
        return false;
    }
    return true;
}

const ct::Json* arrayField(const ct::Json& object, const char* key, SceneResult& result,
        const ct::String& path)
{
    const ct::Json* found = object.find(key);
    if (!found) return nullptr;
    if (!found->is_array())
    {
        result.addError(field(path, key), "must be an array");
        return nullptr;
    }
    return found;
}

const ct::Json* objectField(const ct::Json& object, const char* key, SceneResult& result,
        const ct::String& path)
{
    const ct::Json* found = object.find(key);
    if (!found) return nullptr;
    if (!found->is_object())
    {
        result.addError(field(path, key), "must be an object");
        return nullptr;
    }
    return found;
}

bool readFloatField(const ct::Json& object, const char* key, float& out, SceneResult& result,
        const ct::String& path)
{
    const ct::Json* found = object.find(key);
    if (!found) return false;
    if (!toFloat(*found, out))
    {
        result.addError(field(path, key), "expected a number");
        return false;
    }
    return true;
}

bool readIntField(const ct::Json& object, const char* key, std::int64_t& out, SceneResult& result,
        const ct::String& path)
{
    const ct::Json* found = object.find(key);
    if (!found) return false;
    if (!toInt(*found, out))
    {
        result.addError(field(path, key), "expected a whole number");
        return false;
    }
    return true;
}

bool readBoolField(const ct::Json& object, const char* key, bool& out, SceneResult& result,
        const ct::String& path)
{
    const ct::Json* found = object.find(key);
    if (!found) return false;
    if (!toBool(*found, out))
    {
        result.addError(field(path, key), "expected true or false");
        return false;
    }
    return true;
}

bool readVec3Field(const ct::Json& object, const char* key, Math::Vec3& out, SceneResult& result,
        const ct::String& path)
{
    const ct::Json* found = object.find(key);
    if (!found) return false;
    if (!toVec3(*found, out))
    {
        result.addError(field(path, key), "expected [x, y, z] of numbers");
        return false;
    }
    return true;
}

bool readVec4Field(const ct::Json& object, const char* key, Math::Vec4& out, SceneResult& result,
        const ct::String& path)
{
    const ct::Json* found = object.find(key);
    if (!found) return false;
    if (!toVec4(*found, out))
    {
        result.addError(field(path, key), "expected [x, y, z, w] of numbers");
        return false;
    }
    return true;
}

bool readStringField(const ct::Json& object, const char* key, ct::String& out, SceneResult& result,
        const ct::String& path)
{
    const ct::Json* found = object.find(key);
    if (!found) return false;
    if (!found->is_string())
    {
        result.addError(field(path, key), "expected a text");
        return false;
    }
    out = found->str();
    return true;
}

} // namespace scenejson

} // namespace prisma3d
