#pragma once

#include "mathc.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PropertyValue.h"

#include <ct/string.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

enum class MaterialFeature : std::int8_t
{
    None = -1,
    Emission,
    Clearcoat,
    Backlight,
    Refraction,
    Count
};

struct MaterialFeatures
{
    bool enabled[static_cast<int>(MaterialFeature::Count)] = {};
    Math::Vec3 emissionColor = Math::Vec3(1.0f, 1.0f, 1.0f);
    float clearcoatAmount = 1.0f;
    Math::Vec3 backlightColor = Math::Vec3(0.5f, 0.5f, 0.5f);
    float refractionScale = 0.05f;

    static MaterialFeatures of(const MaterialInfo& info);
    bool on(MaterialFeature feature) const;
    bool set(MaterialStore& store, MaterialId material, MaterialFeature feature, bool value);
};

enum class MaterialFieldKind : std::uint8_t
{
    Bool,
    Float,
    Enum,
    Vec2,
    Vec3,
    Color,
    Texture
};

struct MaterialField
{
    using Reader = PropertyValue (*)(const MaterialInfo&);
    using Writer = bool (*)(MaterialStore&, MaterialId, const PropertyValue&);
    using Visibility = bool (*)(const MaterialInfo&, const MaterialFeatures&);

    const char* group;
    const char* key;
    const char* label;
    MaterialFieldKind kind;
    const char* names;
    double minimum;
    double maximum;
    double step;
    bool ranged;
    bool noAlpha;
    MaterialTexture slot;
    MaterialFeature feature;
    Reader read;
    Writer write;
    Visibility visible;
};

class MaterialProperties
{
public:
    static constexpr const char* kLibraryExtension = ".mat";
    static constexpr const char* kBuiltInText = "<built-in from model>";
    static constexpr const char* kUnsavedText = "<built-in>";
    static constexpr const char* kNoneText = "<none>";

    static std::size_t count();
    static const MaterialField& field(std::size_t index);
    static const MaterialField* find(const char* key);
    static std::size_t indexOf(const char* key);

    static std::size_t groupCount();
    static const char* groupName(std::size_t index);
    static bool groupOpenByDefault(std::size_t index);
    static std::size_t groupOf(std::size_t fieldIndex);

    static PropertyInfo infoOf(const MaterialField& field);
    static PropertyValue read(const MaterialInfo& info, const MaterialFeatures& features,
            const MaterialField& field);
    static bool write(MaterialStore& store, MaterialId material, MaterialFeatures& features,
            const MaterialField& field, const PropertyValue& value);
    static bool visible(const MaterialInfo& info, const MaterialFeatures& features,
            const MaterialField& field);

    static bool editable(const MaterialInfo& info);
    static ct::String sourceOf(const MaterialInfo* info);
    static ct::String sourceOf(MaterialId material);
    static bool srgb(MaterialTexture slot);
    static bool usesOrm(const MaterialInfo& info);
    static bool endsWithNoCase(const ct::String& text, const char* suffix);
};

} // namespace prisma3d
