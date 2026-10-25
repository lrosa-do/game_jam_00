#pragma once

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneResources.h"

#include <type_traits>

namespace prisma3d
{

namespace resourceprop
{

template<typename T, typename Id, typename G>
Id read(const T* node, G getter)
{
    if constexpr (std::is_member_function_pointer<G>::value) return (node->*getter)();
    else
        return getter(node);
}

template<typename T, typename Id, typename S>
void write(T* node, S setter, Id id)
{
    if constexpr (std::is_member_function_pointer<S>::value) (node->*setter)(id);
    else
        setter(node, id);
}

template<typename T, typename Id, typename S, typename G>
void add(const PropertyInfo& info, S setter, G getter, bool srgb = true)
{
    PropertySetter propertySetter(
            [setter, srgb](Node* node, const PropertyValue& value)
            {
                Id id;
                const ct::String path = value.asString();
                if (!path.empty())
                {
                    sceneres::load(path, srgb, id);
                    if (!sceneres::alive(id)) return;
                }
                write<T, Id, S>(static_cast<T*>(node), setter, id);
            });
    PropertyGetter propertyGetter(
            [getter](const Node* node)
            {
                const Id id = read<T, Id, G>(static_cast<const T*>(node), getter);
                return PropertyValue::fromResourcePath(
                        sceneres::alive(id) ? sceneres::pathOf(id) : ct::String());
            });
    PropertyUnsaved unsaved(
            [getter](const Node* node)
            {
                const Id id = read<T, Id, G>(static_cast<const T*>(node), getter);
                return sceneres::alive(id) && sceneres::pathOf(id).empty();
            });
    NodeClassDB::addProperty(T::classNameStatic(), info, propertySetter, propertyGetter, unsaved);
}

template<typename T, typename Struct, typename Id, typename SG, typename SS>
void addField(const PropertyInfo& info, SG structGetter, SS structSetter, Id Struct::*member,
        bool srgb = true)
{
    add<T, Id>(
            info,
            [structGetter, structSetter, member](T* node, Id id)
            {
                Struct copy = (node->*structGetter)();
                copy.*member = id;
                (node->*structSetter)(copy);
            },
            [structGetter, member](const T* node) { return ((node->*structGetter)()).*member; },
            srgb);
}

} // namespace resourceprop

} // namespace prisma3d
