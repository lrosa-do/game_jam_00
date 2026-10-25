#pragma once

#include "platform.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

inline bool collectPath(const char* path, PathType type, void* user)
{
    (void) type;
    static_cast<ct::Vector<ct::String>*>(user)->push_back(ct::String(path));
    return true;
}

inline void removeTree(const ct::String& root)
{
    ct::Vector<ct::String> paths;
    fs_enumerate_directory(root.c_str(), true, &collectPath, &paths);
    for (size_t pass = 0; pass < 16; ++pass)
        for (size_t i = 0; i < paths.size(); ++i) fs_remove_path(paths[i].c_str());
    fs_remove_path(root.c_str());
}
