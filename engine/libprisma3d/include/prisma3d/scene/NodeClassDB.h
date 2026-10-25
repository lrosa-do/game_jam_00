#pragma once

#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/PropertyValue.h"

#include <ct/function.hpp>
#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace prisma3d
{

enum class PropertyHint : std::uint8_t
{
    None,
    Range,
    Enum,
    Flags,
    Layers3DRender,
    Layers3DPhysics,
    File,
    ResourceType,
    ColorNoAlpha,
    NodePathValidTypes,
    GroupEnable
};

struct PropertyUsage
{
    static constexpr std::uint32_t None = 0;
    static constexpr std::uint32_t Storage = 1u << 1;
    static constexpr std::uint32_t Editor = 1u << 2;
    static constexpr std::uint32_t Group = 1u << 6;
    static constexpr std::uint32_t Subgroup = 1u << 8;
    static constexpr std::uint32_t GroupClosed = 1u << 9;
    static constexpr std::uint32_t Default = Storage | Editor;
    static constexpr std::uint32_t NoEditor = Storage;
};

struct PropertyInfo
{
    PropertyType type = PropertyType::Nil;
    ct::String name;
    PropertyHint hint = PropertyHint::None;
    ct::String hintString;
    double rangeMin = 0.0;
    double rangeMax = 0.0;
    double rangeStep = 0.0;
    bool rangeOrGreater = false;
    bool rangeOrLess = false;
    bool radiansAsDegrees = false;
    std::uint32_t usage = PropertyUsage::Default;

    PropertyInfo() = default;

    PropertyInfo(PropertyType propertyType, const char* propertyName,
            PropertyHint propertyHint = PropertyHint::None, const char* propertyHintString = "",
            std::uint32_t propertyUsage = PropertyUsage::Default)
            : type(propertyType),
              name(propertyName),
              hint(propertyHint),
              hintString(propertyHintString),
              usage(propertyUsage)
    {
    }

    static PropertyInfo range(PropertyType propertyType, const char* propertyName, double minimum,
            double maximum, double step, bool orGreater = false, bool orLess = false,
            std::uint32_t propertyUsage = PropertyUsage::Default)
    {
        PropertyInfo info(propertyType, propertyName, PropertyHint::Range, "", propertyUsage);
        info.rangeMin = minimum;
        info.rangeMax = maximum;
        info.rangeStep = step;
        info.rangeOrGreater = orGreater;
        info.rangeOrLess = orLess;
        return info;
    }

    static PropertyInfo enumeration(const char* propertyName, const char* names,
            std::uint32_t propertyUsage = PropertyUsage::Default)
    {
        return PropertyInfo(PropertyType::Int, propertyName, PropertyHint::Enum, names,
                propertyUsage);
    }

    static PropertyInfo flags(const char* propertyName, const char* names,
            std::uint32_t propertyUsage = PropertyUsage::Default)
    {
        return PropertyInfo(PropertyType::Int, propertyName, PropertyHint::Flags, names,
                propertyUsage);
    }

    static PropertyInfo layers3DRender(const char* propertyName)
    {
        return PropertyInfo(PropertyType::Int, propertyName, PropertyHint::Layers3DRender);
    }

    static PropertyInfo layers3DPhysics(const char* propertyName)
    {
        return PropertyInfo(PropertyType::Int, propertyName, PropertyHint::Layers3DPhysics);
    }

    static PropertyInfo color(const char* propertyName, bool noAlpha = false)
    {
        return PropertyInfo(PropertyType::Color, propertyName,
                noAlpha ? PropertyHint::ColorNoAlpha : PropertyHint::None);
    }

    static PropertyInfo resource(const char* propertyName, const char* resourceTypes)
    {
        return PropertyInfo(PropertyType::ResourcePath, propertyName, PropertyHint::ResourceType,
                resourceTypes);
    }

    static PropertyInfo file(const char* propertyName, const char* extensions)
    {
        return PropertyInfo(PropertyType::ResourcePath, propertyName, PropertyHint::File,
                extensions);
    }

    static PropertyInfo nodePath(const char* propertyName, const char* validTypes = "")
    {
        return PropertyInfo(PropertyType::NodePath, propertyName,
                validTypes[0] ? PropertyHint::NodePathValidTypes : PropertyHint::None, validTypes);
    }

    bool isGroup() const
    {
        return (usage & (PropertyUsage::Group | PropertyUsage::Subgroup)) != 0;
    }

    std::size_t enumCount() const;
    ct::String enumName(std::size_t index) const;
};

using PropertySetter = ct::Function<void(Node*, const PropertyValue&)>;
using PropertyGetter = ct::Function<PropertyValue(const Node*)>;
using PropertyUnsaved = ct::Function<bool(const Node*)>;

template<typename V, typename = void>
struct PropertyConvert;

template<>
struct PropertyConvert<bool>
{
    static bool from(const PropertyValue& value)
    {
        return value.asBool();
    }
    static PropertyValue to(bool value, PropertyType)
    {
        return PropertyValue::fromBool(value);
    }
};

template<typename V>
struct PropertyConvert<V,
        typename std::enable_if<std::is_integral<V>::value && !std::is_same<V, bool>::value>::type>
{
    static V from(const PropertyValue& value)
    {
        return static_cast<V>(value.asInt());
    }
    static PropertyValue to(V value, PropertyType)
    {
        return PropertyValue::fromInt(static_cast<std::int64_t>(value));
    }
};

template<typename V>
struct PropertyConvert<V, typename std::enable_if<std::is_enum<V>::value>::type>
{
    static V from(const PropertyValue& value)
    {
        return static_cast<V>(value.asInt());
    }
    static PropertyValue to(V value, PropertyType)
    {
        return PropertyValue::fromInt(static_cast<std::int64_t>(value));
    }
};

template<typename V>
struct PropertyConvert<V, typename std::enable_if<std::is_floating_point<V>::value>::type>
{
    static V from(const PropertyValue& value)
    {
        return static_cast<V>(value.asFloat());
    }
    static PropertyValue to(V value, PropertyType)
    {
        if constexpr (sizeof(V) == sizeof(float))
            return PropertyValue::fromSingle(static_cast<float>(value));
        else
            return PropertyValue::fromFloat(static_cast<double>(value));
    }
};

template<>
struct PropertyConvert<Math::Vec2>
{
    static Math::Vec2 from(const PropertyValue& value)
    {
        return value.asVec2();
    }
    static PropertyValue to(const Math::Vec2& value, PropertyType)
    {
        return PropertyValue::fromVec2(value);
    }
};

template<>
struct PropertyConvert<Math::Vec3>
{
    static Math::Vec3 from(const PropertyValue& value)
    {
        if (value.type() == PropertyType::Color)
        {
            const Math::Vec4 color = value.asColor();
            return Math::Vec3(color.x, color.y, color.z);
        }
        return value.asVec3();
    }
    static PropertyValue to(const Math::Vec3& value, PropertyType type)
    {
        if (type == PropertyType::Color)
            return PropertyValue::fromColor(Math::Vec4(value.x, value.y, value.z, 1.0f));
        return PropertyValue::fromVec3(value);
    }
};

template<>
struct PropertyConvert<Math::Vec4>
{
    static Math::Vec4 from(const PropertyValue& value)
    {
        if (value.type() == PropertyType::Color) return value.asColor();
        return value.asVec4();
    }
    static PropertyValue to(const Math::Vec4& value, PropertyType type)
    {
        if (type == PropertyType::Color) return PropertyValue::fromColor(value);
        return PropertyValue::fromVec4(value);
    }
};

template<>
struct PropertyConvert<Math::Quaternion>
{
    static Math::Quaternion from(const PropertyValue& value)
    {
        return value.asQuat();
    }
    static PropertyValue to(const Math::Quaternion& value, PropertyType)
    {
        return PropertyValue::fromQuat(value);
    }
};

template<>
struct PropertyConvert<ct::String>
{
    static ct::String from(const PropertyValue& value)
    {
        return value.asString();
    }
    static PropertyValue to(const ct::String& value, PropertyType type)
    {
        if (type == PropertyType::NodePath) return PropertyValue::fromNodePath(value);
        if (type == PropertyType::ResourcePath) return PropertyValue::fromResourcePath(value);
        return PropertyValue::fromString(value);
    }
};

class NodeClassDB
{
public:
    template<typename T>
    static void registerClass()
    {
        static_assert(std::is_same<typename T::SelfType, T>::value,
                "Class not declared properly, please use PRISMA3D_NODE_CLASS");
        initializeClass<T>();
        setCreation(T::classNameStatic(), &creator<T>);
    }

    template<typename T>
    static void registerAbstractClass()
    {
        static_assert(std::is_same<typename T::SelfType, T>::value,
                "Class not declared properly, please use PRISMA3D_NODE_CLASS");
        initializeClass<T>();
    }

    static void getClassList(ct::Vector<ct::String>& classes);
    static void getInheritersFromClass(const ct::String& className,
            ct::Vector<ct::String>& classes);
    static void getDirectInheritersFromClass(const ct::String& className,
            ct::Vector<ct::String>& classes);
    static ct::String getParentClass(const ct::String& className);
    static bool classExists(const ct::String& className);
    static bool isParentClass(const ct::String& className, const ct::String& inherits);
    static bool canInstantiate(const ct::String& className);
    static bool isAbstract(const ct::String& className);

    template<typename T>
    static void addPropertyGroup(const char* name, const char* prefix, bool closed = false)
    {
        addPropertyGroup(T::classNameStatic(), name, prefix, closed);
    }

    template<typename T>
    static void addPropertySubgroup(const char* name, const char* prefix)
    {
        addPropertySubgroup(T::classNameStatic(), name, prefix);
    }

    template<typename T, typename S, typename G>
    static void addProperty(const PropertyInfo& info, S setter, G getter)
    {
        addProperty(T::classNameStatic(), info, makeSetter<T>(setter),
                makeGetter<T>(getter, info.type));
    }

    template<typename T, typename Struct, typename SG, typename SS, typename F>
    static void addFieldProperty(const PropertyInfo& info, SG structGetter, SS structSetter,
            F Struct::*member)
    {
        using V = typename std::decay<F>::type;
        const PropertyType type = info.type;
        addProperty(T::classNameStatic(), info,
                PropertySetter(
                        [structGetter, structSetter, member](Node* node, const PropertyValue& value)
                        {
                            T* target = static_cast<T*>(node);
                            Struct copy = (target->*structGetter)();
                            copy.*member = PropertyConvert<V>::from(value);
                            (target->*structSetter)(copy);
                        }),
                PropertyGetter(
                        [structGetter, member, type](const Node* node)
                        {
                            const T* target = static_cast<const T*>(node);
                            auto&& current = (target->*structGetter)();
                            return PropertyConvert<V>::to(current.*member, type);
                        }));
    }

    static void addPropertyGroup(const char* className, const char* name, const char* prefix,
            bool closed = false);
    static void addPropertySubgroup(const char* className, const char* name, const char* prefix);
    static void addProperty(const char* className, const PropertyInfo& info,
            const PropertySetter& setter, const PropertyGetter& getter,
            const PropertyUnsaved& unsaved = PropertyUnsaved());
    static void setPropertyDefaultValue(const char* className, const char* name,
            const PropertyValue& value);

    static void getPropertyList(const ct::String& className, ct::Vector<PropertyInfo>& list,
            bool noInheritance = false);
    static void getInstancePropertyList(const Node* node, const ct::String& className,
            ct::Vector<PropertyInfo>& list, bool noInheritance = false);
    static bool getPropertyInfo(const ct::String& className, const ct::String& property,
            PropertyInfo& info, bool noInheritance = false);
    static bool setProperty(Node* node, const ct::String& property, const PropertyValue& value);
    static bool getProperty(const Node* node, const ct::String& property, PropertyValue& value);
    static bool isPropertyUnsaved(const Node* node, const ct::String& property);
    static bool getDefaultPropertyValue(const ct::String& className, const ct::String& property,
            PropertyValue& value);

private:
    friend class Node;
    friend class SceneSerializer;

    typedef Node* (*CreationFunc)();

    struct PropertyEntry
    {
        PropertyInfo info;
        PropertySetter setter;
        PropertyGetter getter;
        PropertyUnsaved unsaved;
    };

    struct ClassInfo
    {
        ct::String name;
        ct::String parent;
        void* classPtr;
        CreationFunc creation;
        ct::Vector<PropertyEntry> properties;
        ct::HashMap<ct::String, std::size_t> propertyIndex;
        ct::HashMap<ct::String, PropertyValue> defaults;
        bool defaultsCached = false;
    };

    template<typename M>
    struct MemberTraits;

    template<typename R, typename C, typename A>
    struct MemberTraits<R (C::*)(A)>
    {
        using Arg = typename std::decay<A>::type;
    };

    template<typename R, typename C>
    struct MemberTraits<R (C::*)() const>
    {
        using Ret = typename std::decay<R>::type;
    };

    template<typename T, typename S>
    static PropertySetter makeSetter(S setter)
    {
        if constexpr (std::is_null_pointer<S>::value)
        {
            return PropertySetter();
        }
        else if constexpr (std::is_member_function_pointer<S>::value)
        {
            using Arg = typename MemberTraits<S>::Arg;
            return PropertySetter([setter](Node* node, const PropertyValue& value)
                    { (static_cast<T*>(node)->*setter)(PropertyConvert<Arg>::from(value)); });
        }
        else
        {
            return PropertySetter([setter](Node* node, const PropertyValue& value)
                    { setter(static_cast<T*>(node), value); });
        }
    }

    template<typename T, typename G>
    static PropertyGetter makeGetter(G getter, PropertyType type)
    {
        if constexpr (std::is_null_pointer<G>::value)
        {
            return PropertyGetter();
        }
        else if constexpr (std::is_member_function_pointer<G>::value)
        {
            using Ret = typename MemberTraits<G>::Ret;
            return PropertyGetter(
                    [getter, type](const Node* node) {
                        return PropertyConvert<Ret>::to((static_cast<const T*>(node)->*getter)(),
                                type);
                    });
        }
        else
        {
            return PropertyGetter(
                    [getter](const Node* node) { return getter(static_cast<const T*>(node)); });
        }
    }

    struct Registry;

    template<typename T>
    static Node* creator()
    {
        return new T;
    }

    template<typename T>
    static void initializeClass()
    {
        if constexpr (std::is_same<T, Node>::value)
        {
            if (addClass(T::classNameStatic(), "", T::classPtrStatic())) T::bindProperties();
        }
        else
        {
            initializeClass<typename T::SuperType>();
            if (addClass(T::classNameStatic(), T::SuperType::classNameStatic(),
                        T::classPtrStatic()))
            {
                if (&T::bindProperties != &T::SuperType::bindProperties) T::bindProperties();
            }
        }
    }

    static Registry& registry();
    static void ensureSceneTypes();
    static const ClassInfo* find(const ct::String& className);
    static bool addClass(const char* name, const char* parent, void* classPtr);
    static void setCreation(const char* name, CreationFunc creation);
    static Node* instantiate(const ct::String& className);
};

} // namespace prisma3d
