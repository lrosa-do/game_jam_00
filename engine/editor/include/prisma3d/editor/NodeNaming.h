#pragma once

#include <ct/string.hpp>

namespace prisma3d
{

class Node;

struct NodeNaming
{
    static constexpr const char* kInvalidNameMessage =
            "Invalid node name, the following characters are not allowed:\n. : @ / \" %";

    static ct::String resolve(const Node* node, const ct::String& requested, ct::String& error);
};

} // namespace prisma3d
