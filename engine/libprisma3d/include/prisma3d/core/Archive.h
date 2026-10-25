#pragma once

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

struct IoStream;

namespace prisma3d
{

class Archive
{
public:
    virtual ~Archive();

    virtual bool exists(const ct::String& name) const = 0;
    virtual ct::Vector<std::uint8_t> readBinary(const ct::String& name) const = 0;
    virtual IoStream* open(const ct::String& name) const = 0;
};

} // namespace prisma3d
