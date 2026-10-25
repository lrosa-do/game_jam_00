#pragma once

#include <igui/widgets/Retained.hpp>

#include <ct/hashmap.hpp>
#include <ct/string.hpp>

namespace prisma3d
{

class Node;

class ClassIcons
{
public:
    ClassIcons();

    void set(const char* className, ig::retained::IconId icon);
    bool hasEntry(const ct::String& className) const;
    void setDisabled(ig::retained::IconId icon);
    ig::retained::IconId disabled() const;

    ig::retained::IconId icon(const ct::String& className) const;
    ig::retained::IconId icon(const Node* node) const;

    static const char* defaultIconName(const char* className);

private:
    ig::retained::IconId lookup(const ct::String& className, bool skipDisabled) const;
    void refresh() const;

    ct::HashMap<ct::String, ig::retained::IconId> mOverrides;
    mutable ct::HashMap<ct::String, ig::retained::IconId> mCache;
    ig::retained::IconId mDisabled;
    bool mDisabledSet;
    mutable unsigned int mGeneration;
};

} // namespace prisma3d
