#pragma once

#include "prisma3d/editor/AssetTypes.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>

namespace prisma3d
{

class Project;

struct ImportFile
{
    ct::String source;
    ct::String relative;
    ct::String destination;
    AssetKind kind = AssetKind::Other;
    bool exists = false;
    bool primary = false;
};

struct ImportPlan
{
    ct::Vector<ImportFile> files;
    ct::Vector<ct::String> skipped;

    std::size_t clashCount() const;
    int firstModel() const;
    bool empty() const;
};

class AssetImport
{
public:
    static bool plan(const Project& project, const ct::Vector<ct::String>& sources,
            const ct::String& folder, ImportPlan& out);
    static std::size_t copy(const ImportPlan& plan, bool overwrite, ct::Vector<ct::String>& failed);
    static void gltfCompanions(const ct::String& gltfPath, ct::Vector<ct::String>& uris);
    static ct::String decodeUri(const ct::String& uri);
    static bool safeRelative(const ct::String& path);
    static ct::String joinPath(const ct::String& folder, const ct::String& name);
    static ct::String parentOf(const ct::String& path);
    static ct::String nameOf(const ct::String& path);
    static ct::String stemOf(const ct::String& name);
};

} // namespace prisma3d
